#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nvjpeg.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include "seeker.hpp"

// ── 单路摄像头标定参数 + 预计算 GPU 重映射表 ─────────────────────────────
struct CamParam {
  double xi{0}, fx{0}, fy{0}, cx{0}, cy{0};
  double k1{0}, k2{0}, p1{0}, p2{0};
  int src_w{0}, src_h{0};
  cv::cuda::GpuMat map_x, map_y;  // CV_32FC1，启动时上传一次
};

class SeekRosNode : public rclcpp::Node {
public:
  explicit SeekRosNode() : Node("seeker_node") {
    // ── nvJPEG 初始化 ──────────────────────────────────────────────────
    nvjpegCreateSimple(&nv_handle_);
    nvjpegJpegStateCreate(nv_handle_, &nv_state_);
    cudaStreamCreate(&cuda_stream_);
    nv_output_.channel[0] = nullptr;

    // ── 参数声明 ───────────────────────────────────────────────────────
    declare_parameter("pub_disparity_img", false);
    declare_parameter("pub_disparity", true);
    declare_parameter("pub_imu", true);
    declare_parameter("time_sync", true);
    declare_parameter("pub_fisheye_raw", false);
    declare_parameter("imu_link", "imu_link");
    declare_parameter("imu_topic", "imu_data_raw");
    declare_parameter("pub_rect", true);
    declare_parameter("calib_file", std::string("seeker_omni_depth/kalibr_cam_chain.yaml"));
    declare_parameter("undistort_scale", 0.5);
    declare_parameter("undistort_fov_scale", 1.5);
    declare_parameter("jpeg_quality", 80);

    pub_disparity_img_ = get_parameter("pub_disparity_img").as_bool();
    pub_disparity_      = get_parameter("pub_disparity").as_bool();
    pub_imu_            = get_parameter("pub_imu").as_bool();
    time_sync_          = get_parameter("time_sync").as_bool();
    pub_fisheye_raw_    = get_parameter("pub_fisheye_raw").as_bool();
    imu_link_           = get_parameter("imu_link").as_string();
    imu_topic_          = get_parameter("imu_topic").as_string();
    pub_rect_           = get_parameter("pub_rect").as_bool();
    undistort_scale_    = get_parameter("undistort_scale").as_double();
    undistort_fov_scale_= get_parameter("undistort_fov_scale").as_double();
    jpeg_quality_       = get_parameter("jpeg_quality").as_int();

    // ── 标定文件加载 + GPU 重映射表预计算 ─────────────────────────────
    if (pub_rect_) {
      std::string calib_rel = get_parameter("calib_file").as_string();
      std::string calib_path;
      try {
        calib_path = ament_index_cpp::get_package_share_directory("seeker")
                     + "/config/" + calib_rel;
      } catch (...) {
        calib_path = std::string(getenv("HOME"))
                     + "/ros2_ws/src/seeker1/config/" + calib_rel;
      }
      loadCalibration(calib_path);
    }

    // ── 设备初始化 ────────────────────────────────────────────────────
    std::vector<seeker_device_t> devices = seek_.find_devices();
    if (devices.empty()) {
      RCLCPP_ERROR(get_logger(), "No Seeker Devices Found");
      return;
    }
    seek_.open(devices[0]);
    seek_.set_event_callback([this](auto&& ph, auto&& e) { onEvent(ph, e); });
    seek_.set_mjpeg_callback([this](auto&& ph, auto&& d, auto&& l) { onMjpeg(ph, d, l); });
    seek_.set_depth_callback([this](auto&& ph, auto&& d, auto&& l) { onDepth(ph, d, l); });
    if (time_sync_)
      seek_.set_timer_callback([this](auto&& tg, auto&& ts) { return onTimer(tg, ts); });

    seek_.get_dev_info(sdev_);

    // ── 话题发布者 ────────────────────────────────────────────────────
    const std::vector<std::string> rect_topics = {
      "/fisheye_rect/left/image_raw/compressed",
      "/fisheye_rect/right/image_raw/compressed",
      "/fisheye_rect/bright/image_raw/compressed",
      "/fisheye_rect/bleft/image_raw/compressed",
    };
    const std::vector<std::string> raw_topics = {
      "/fisheye/left/image_raw",
      "/fisheye/right/image_raw",
      "/fisheye/bright/image_raw",
      "/fisheye/bleft/image_raw",
    };
    const std::vector<std::string> depth_topics = {
      "front/disparity/image_raw", "right/disparity/image_raw",
      "back/disparity/image_raw",  "left/disparity/image_raw"
    };
    const std::vector<std::string> disparity_topics = {
      "front/disparity", "right/disparity",
      "back/disparity",  "left/disparity"
    };

    compressed_image_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("all/compressed", 10);

    for (size_t i = 0; i < sdev_.dev_info.rgb_camera_number; ++i) {
      if (pub_rect_)
        rect_compressed_pubs_.push_back(
          create_publisher<sensor_msgs::msg::CompressedImage>(rect_topics[i], 10));
      if (pub_fisheye_raw_)
        image_pubs_ros_.push_back(
          create_publisher<sensor_msgs::msg::Image>(raw_topics[i], 10));
    }

    for (size_t i = 0; i < sdev_.dev_info.depth_camera_number; ++i) {
      if (pub_disparity_img_)
        depth_pubs_.push_back(
          create_publisher<sensor_msgs::msg::Image>(depth_topics[i], 10));
      if (pub_disparity_)
        disparity_pubs_.push_back(
          create_publisher<stereo_msgs::msg::DisparityImage>(disparity_topics[i], 10));
    }

    if (pub_imu_)
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 200);

    // ── 启动发布线程 ──────────────────────────────────────────────────
    publish_thread_ = std::thread(&SeekRosNode::publishLoop, this);

    seek_.start_event_stream();
    seek_.start_image_stream();
    seek_.start_depth_stream();
  }

  ~SeekRosNode() {
    seek_.stop_event_stream();
    seek_.stop_image_stream();
    seek_.stop_depth_stream();
    seek_.close();
    {
      std::lock_guard<std::mutex> lk(queue_mutex_);
      stop_publish_ = true;
    }
    queue_cv_.notify_one();
    if (publish_thread_.joinable()) publish_thread_.join();

    if (nv_output_.channel[0]) cudaFree(nv_output_.channel[0]);
    for (auto* p : buf_pool_) cudaFree(p);
    nvjpegJpegStateDestroy(nv_state_);
    nvjpegDestroy(nv_handle_);
    cudaStreamDestroy(cuda_stream_);
  }

private:
  // ── 标定加载 ───────────────────────���─────────────────────────────────
  void loadCalibration(const std::string& path) {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load calib file %s: %s", path.c_str(), e.what());
      pub_rect_ = false;
      return;
    }

    const std::vector<std::string> cam_names = {"cam0","cam1","cam2","cam3"};
    for (const auto& name : cam_names) {
      if (!root[name]) continue;
      auto cam = root[name];
      if (!cam["camera_model"] || cam["camera_model"].as<std::string>() != "omni") continue;

      CamParam p;
      auto intr = cam["intrinsics"].as<std::vector<double>>();
      p.xi = intr[0]; p.fx = intr[1]; p.fy = intr[2];
      p.cx = intr[3]; p.cy = intr[4];
      auto dist = cam["distortion_coeffs"].as<std::vector<double>>();
      p.k1 = dist[0]; p.k2 = dist[1]; p.p1 = dist[2]; p.p2 = dist[3];
      auto res = cam["resolution"].as<std::vector<int>>();
      p.src_h = res[0]; p.src_w = res[1];

      buildUndistortMap(p);
      cam_params_.push_back(std::move(p));
      RCLCPP_INFO(get_logger(), "%s maps built (%dx%d → %dx%d)",
                  name.c_str(), p.src_w, p.src_h,
                  static_cast<int>(p.src_w * undistort_scale_),
                  static_cast<int>(p.src_h * undistort_scale_));
    }

    if (cam_params_.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid omni cameras in calib file");
      pub_rect_ = false;
    }
  }

  // ── 重映射表计算（复现 Python init_undistort_rectify_map）────────────
  void buildUndistortMap(CamParam& p) {
    const int out_w = static_cast<int>(p.src_w * undistort_scale_);
    const int out_h = static_cast<int>(p.src_h * undistort_scale_);

    const double zoom = 1.0 / undistort_fov_scale_;
    const double out_fx = p.fx * zoom * undistort_scale_;
    const double out_fy = p.fy * zoom * undistort_scale_;
    const double out_cx = out_w / 2.0;
    const double out_cy = out_h / 2.0;

    // P[:3,:3] = [[out_fx,0,out_cx],[0,out_fy,out_cy],[0,0,1]]
    // iKR = inv(P) (R=I)
    const double iKR[3][3] = {
      {1.0/out_fx,  0,          -out_cx/out_fx},
      {0,           1.0/out_fy, -out_cy/out_fy},
      {0,           0,           1.0           }
    };

    cv::Mat map_x(out_h, out_w, CV_32FC1);
    cv::Mat map_y(out_h, out_w, CV_32FC1);

    for (int v = 0; v < out_h; ++v) {
      float* mx = map_x.ptr<float>(v);
      float* my = map_y.ptr<float>(v);
      for (int u = 0; u < out_w; ++u) {
        double x = iKR[0][0]*u + iKR[0][1]*v + iKR[0][2];
        double y = iKR[1][0]*u + iKR[1][1]*v + iKR[1][2];
        double w = iKR[2][0]*u + iKR[2][1]*v + iKR[2][2];

        double r = std::sqrt(x*x + y*y + w*w);
        double Xs = x/r, Ys = y/r, Zs = w/r;

        double denom = Zs + p.xi;
        double xu = Xs / denom;
        double yu = Ys / denom;

        double r2 = xu*xu + yu*yu;
        double r4 = r2*r2;
        double radial = 1.0 + p.k1*r2 + p.k2*r4;
        double xd = radial*xu + 2*p.p1*xu*yu + p.p2*(r2 + 2*xu*xu);
        double yd = radial*yu + p.p1*(r2 + 2*yu*yu) + 2*p.p2*xu*yu;

        mx[u] = static_cast<float>(p.fx*xd + p.cx);
        my[u] = static_cast<float>(p.fy*yd + p.cy);
      }
    }

    p.map_x.upload(map_x);
    p.map_y.upload(map_y);
  }

  // ── 深度回调 ─────────────────────────────────────��───────────────────
  void onDepth(const event_header_t& eheader, const uint8_t* data, int len) {
    const int depth_camera_number = sdev_.dev_info.depth_camera_number;
    const int height = sdev_.dev_info.depth_resolution_height / depth_camera_number;
    const int width  = sdev_.dev_info.depth_resolution_width;

    std::vector<cv::Mat> images;
    for (int i = 0; i < depth_camera_number; i++)
      images.emplace_back(height, width, CV_16UC1,
                          const_cast<uint8_t*>(data) + i * height * width * 2);

    auto header = std::make_shared<std_msgs::msg::Header>();
    header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);

    for (size_t i = 0; i < (size_t)depth_camera_number; ++i) {
      if (pub_disparity_img_ && i < depth_pubs_.size()) {
        header->frame_id = "depth" + std::to_string(i);
        depth_pubs_[i]->publish(
          *cv_bridge::CvImage(*header, "16UC1", images[i]).toImageMsg());
      }
    }

    for (size_t i = 0; i < (size_t)depth_camera_number; ++i) {
      if (!pub_disparity_ || i >= disparity_pubs_.size()) continue;
      const double inv_dpp = 1.0 / (256/4);
      auto msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
      auto& dimage = msg->image;
      dimage.header   = *header;
      dimage.height   = images[i].rows;
      dimage.width    = images[i].cols;
      dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      dimage.step     = dimage.width * sizeof(float);
      dimage.data.resize(dimage.step * dimage.height);
      cv::Mat_<float> dmat(images[i].rows, images[i].cols,
                           reinterpret_cast<float*>(dimage.data.data()));
      images[i].convertTo(dmat, CV_32F, inv_dpp, 0);
      msg->f = 320.0; msg->t = 0.04625;
      msg->min_disparity = 0.0; msg->max_disparity = 192;
      msg->delta_d = inv_dpp; msg->header = *header;
      disparity_pubs_[i]->publish(*msg);
    }
  }

  // ── IMU 回调 ──────────────────────────────────────────────────────────
  void onEvent(const event_header_t& header, const device_event_t& event) {
    if (event.type == EVENT_TYPE_SENSOR_CUSTOM && pub_imu_) {
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      msg->header.stamp    = rclcpp::Time(header.sec, header.nsec);
      msg->header.frame_id = imu_link_;
      msg->angular_velocity.x    = event.event.sensor_custom.angular_velocity_x;
      msg->angular_velocity.y    = event.event.sensor_custom.angular_velocity_y;
      msg->angular_velocity.z    = event.event.sensor_custom.angular_velocity_z;
      msg->linear_acceleration.x = event.event.sensor_custom.linear_acceleration_x;
      msg->linear_acceleration.y = event.event.sensor_custom.linear_acceleration_y;
      msg->linear_acceleration.z = event.event.sensor_custom.linear_acceleration_z;
      imu_pub_->publish(*msg);
    }
  }

  // ── MJPEG 回调（仅提交 GPU 解码，立即返回）───────────────────────────
  void onMjpeg(const event_header_t& pheader, const uint8_t* data, int len) {
    auto now = std::chrono::steady_clock::now();
    if (last_mjpeg_time_.time_since_epoch().count() != 0) {
      double gap_ms = std::chrono::duration<double, std::milli>(now - last_mjpeg_time_).count();
      if (gap_ms > 80.0)
        RCLCPP_WARN(get_logger(), "mjpeg callback gap %.1fms (device/USB stall?)", gap_ms);
    }
    last_mjpeg_time_ = now;

    if (pub_fisheye_raw_) {
      auto compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
      compressed->header.stamp = rclcpp::Time(pheader.sec, pheader.nsec);
      compressed->format = "jpeg";
      compressed->data.assign(data, data + len);
      compressed_image_pub_->publish(*compressed);
    }

    if (len < 2 || data[0] != 0xFF || data[1] != 0xD8) return;

    int nComponents = 0;
    nvjpegChromaSubsampling_t subsampling;
    int widths[NVJPEG_MAX_COMPONENT] = {0};
    int heights[NVJPEG_MAX_COMPONENT] = {0};
    nvjpegStatus_t st = nvjpegGetImageInfo(
      nv_handle_, data, len, &nComponents, &subsampling, widths, heights);
    if (st != NVJPEG_STATUS_SUCCESS) {
      RCLCPP_WARN(get_logger(),
        "nvjpegGetImageInfo failed: status=%d len=%d header=0x%02x%02x%02x%02x",
        st, len, data[0], data[1], data[2], data[3]);
      return;
    }

    const int w = widths[0], h = heights[0];
    const size_t needed = (size_t)w * h * 3;

    uint8_t* gpu_buf = nullptr;
    {
      std::lock_guard<std::mutex> lk(pool_mutex_);
      if (!buf_pool_.empty()) { gpu_buf = buf_pool_.back(); buf_pool_.pop_back(); }
    }
    if (!gpu_buf) cudaMalloc(reinterpret_cast<void**>(&gpu_buf), needed);

    nvjpegImage_t out{};
    out.channel[0] = gpu_buf;
    out.pitch[0]   = w * 3;

    if (nvjpegDecode(nv_handle_, nv_state_, data, len,
                     NVJPEG_OUTPUT_BGRI, &out, cuda_stream_) != NVJPEG_STATUS_SUCCESS) {
      RCLCPP_WARN(get_logger(), "nvjpegDecode failed");
      std::lock_guard<std::mutex> lk(pool_mutex_);
      buf_pool_.push_back(gpu_buf);
      return;
    }

    cudaEvent_t ev;
    cudaEventCreate(&ev);
    cudaEventRecord(ev, cuda_stream_);

    {
      std::lock_guard<std::mutex> lk(queue_mutex_);
      decode_queue_.push({pheader, gpu_buf, ev, w, h});
    }
    queue_cv_.notify_one();
  }

  // ── 发布线程：GPU remap → CPU JPEG 编码 → publish ────────────────────
  void publishLoop() {
    while (true) {
      DecodeTask task;
      {
        std::unique_lock<std::mutex> lk(queue_mutex_);
        queue_cv_.wait(lk, [this] { return !decode_queue_.empty() || stop_publish_; });
        if (stop_publish_ && decode_queue_.empty()) break;
        task = decode_queue_.front();
        decode_queue_.pop();
      }

      cudaEventSynchronize(task.event);
      cudaEventDestroy(task.event);

      const int cam_num = sdev_.dev_info.rgb_camera_number;
      const int cam_h   = task.h / cam_num;

      std_msgs::msg::Header ros_header;
      ros_header.stamp = rclcpp::Time(task.header.sec, task.header.nsec);

      for (int i = 0; i < cam_num; i++) {
        // 从大 GPU 缓冲里切出第 i 路（无拷贝）
        cv::cuda::GpuMat src_gpu(cam_h, task.w, CV_8UC3,
                                  task.gpu_buf + (size_t)i * cam_h * task.w * 3,
                                  task.w * 3);

        if (pub_rect_ && i < (int)cam_params_.size() && !cam_params_[i].map_x.empty()) {
          // GPU remap 去畸变
          cv::cuda::GpuMat dst_gpu;
          cv::cuda::remap(src_gpu, dst_gpu,
                          cam_params_[i].map_x, cam_params_[i].map_y,
                          cv::INTER_LINEAR, cv::BORDER_REPLICATE);
          // remap 在默认 CUDA 流上执行，同步后 download

          cv::Mat dst_cpu;
          dst_gpu.download(dst_cpu);

          std::vector<uchar> jpeg_buf;
          cv::imencode(".jpg", dst_cpu, jpeg_buf,
                       {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

          auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
          msg->header = ros_header;
          msg->header.frame_id = "cam" + std::to_string(i);
          msg->format = "jpeg";
          msg->data   = std::move(jpeg_buf);
          rect_compressed_pubs_[i]->publish(*msg);
        }

        // 可选：同时发布原始 raw（调试用）
        if (pub_fisheye_raw_ && i < (int)image_pubs_ros_.size()) {
          const size_t cam_bytes = (size_t)cam_h * task.w * 3;
          auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
          img_msg->header          = ros_header;
          img_msg->header.frame_id = "cam" + std::to_string(i);
          img_msg->height          = cam_h;
          img_msg->width           = task.w;
          img_msg->encoding        = "bgr8";
          img_msg->is_bigendian    = 0;
          img_msg->step            = task.w * 3;
          img_msg->data.resize(cam_bytes);
          cudaMemcpy(img_msg->data.data(),
                     task.gpu_buf + (size_t)i * cam_bytes,
                     cam_bytes, cudaMemcpyDeviceToHost);
          image_pubs_ros_[i]->publish(*img_msg);
        }
      }

      std::lock_guard<std::mutex> lk(pool_mutex_);
      buf_pool_.push_back(task.gpu_buf);
    }
  }

  bool onTimer(std::pair<uint64_t,uint64_t>& timer_get, std::pair<uint64_t,uint64_t>& timer_set) {
    auto now = this->now();
    timer_set.first  = now.seconds();
    timer_set.second = now.nanoseconds() % 1000000000;
    return true;
  }

  // ── 成员变量 ──────────────────────────────────────────────────────────
  // ROS 发布者
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> rect_compressed_pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_ros_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs_;
  std::vector<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr> disparity_pubs_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  // 设备
  SEEKNS::SEEKER seek_;
  seeker_device_t sdev_;

  // nvJPEG
  nvjpegHandle_t    nv_handle_{};
  nvjpegJpegState_t nv_state_{};
  nvjpegImage_t     nv_output_{};
  cudaStream_t      cuda_stream_{};

  // GPU 缓冲池
  std::vector<uint8_t*> buf_pool_;
  std::mutex pool_mutex_;

  // 异步队列
  struct DecodeTask {
    event_header_t header;
    uint8_t*       gpu_buf;
    cudaEvent_t    event;
    int w, h;
  };
  std::queue<DecodeTask>   decode_queue_;
  std::mutex               queue_mutex_;
  std::condition_variable  queue_cv_;
  std::thread              publish_thread_;
  bool                     stop_publish_{false};

  // 标定 + 重映射
  std::vector<CamParam> cam_params_;

  // 参数
  bool        pub_disparity_img_;
  bool        pub_disparity_;
  bool        pub_imu_;
  bool        pub_fisheye_raw_;
  bool        pub_rect_;
  bool        time_sync_;
  double      undistort_scale_;
  double      undistort_fov_scale_;
  int         jpeg_quality_;
  std::string imu_link_;
  std::string imu_topic_;

  std::chrono::steady_clock::time_point last_mjpeg_time_{};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeekRosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
