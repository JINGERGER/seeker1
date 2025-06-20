#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "seeker.hpp"

class SeekRosNode : public rclcpp::Node {
public:
  explicit SeekRosNode() : Node("seeker_node") {
    // 参数声明与获取
    // declare_parameter("use_image_transport", true);
    declare_parameter("pub_disparity_img", false);
    declare_parameter("pub_disparity", true);
    declare_parameter("pub_imu", true);
    declare_parameter("time_sync", true);
    declare_parameter("imu_link", "imu_link");
    declare_parameter("imu_topic", "imu_data_raw");

    // use_image_transport_ = get_parameter("use_image_transport").as_bool();
    pub_disparity_img_ = get_parameter("pub_disparity_img").as_bool();
    pub_disparity_ = get_parameter("pub_disparity").as_bool();
    pub_imu_ = get_parameter("pub_imu").as_bool();
    time_sync_ = get_parameter("time_sync").as_bool();
    imu_link_ = get_parameter("imu_link").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    // 初始化image_transport
    // if (use_image_transport_) {
    //   it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    // }

    // 初始化设备
    std::vector<seeker_device_t> devices = seek_.find_devices();
    if (devices.empty()) {
      RCLCPP_ERROR(get_logger(), "No Seeker Devices Found");
      return;
    }
    seek_.open(devices[0]);

    // 设置回调
    seek_.set_event_callback([this](auto&& ph, auto&& e) { onEvent(ph, e); });
    seek_.set_mjpeg_callback([this](auto&& ph, auto&& d, auto&& l) { onMjpeg(ph, d, l); });
    seek_.set_depth_callback([this](auto&& ph, auto&& d, auto&& l) { onDepth(ph, d, l); });
    
    if (time_sync_) {
      seek_.set_timer_callback([this](auto&& tg, auto&& ts) { return onTimer(tg, ts); });
    }

    // 获取设备信息
    seek_.get_dev_info(sdev_);

    // 初始化发布者
    const std::vector<std::string> image_topics = {
        "/fisheye/left/image_raw",
        "/fisheye/right/image_raw",
        "/fisheye/bright/image_raw",
        "/fisheye/bleft/image_raw",
    };
    const std::vector<std::string> depth_topics = {
      "front/disparity/image_raw",
      "right/disparity/image_raw",
      "back/disparity/image_raw",
      "left/disparity/image_raw"
    };
    const std::vector<std::string> disparity_topics = {
      "front/disparity",
      "right/disparity",
      "back/disparity",
      "left/disparity"
    };

    compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("all/compressed", 10);

    // 图像发布者
    for (size_t i = 0; i < sdev_.dev_info.rgb_camera_number; ++i) {
      // if (use_image_transport_) {
      //   image_pubs_it_.push_back(it_->advertise(image_topics[i], 1));
      // } else {
        image_pubs_ros_.push_back(create_publisher<sensor_msgs::msg::Image>(image_topics[i], 10));
      // }
    }

    // 深度和视差发布者
    for (size_t i = 0; i < sdev_.dev_info.depth_camera_number; ++i) {
      if (pub_disparity_img_) {
        depth_pubs_.push_back(create_publisher<sensor_msgs::msg::Image>(depth_topics[i], 10));
      }
      if (pub_disparity_) {
        disparity_pubs_.push_back(create_publisher<stereo_msgs::msg::DisparityImage>(disparity_topics[i], 10));
      }
    }

    // IMU发布者
    if (pub_imu_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 200);
    }

    // 启动设备流
    seek_.start_event_stream();
    seek_.start_image_stream();
    seek_.start_depth_stream();
  }

  ~SeekRosNode() {
    seek_.stop_event_stream();
    seek_.stop_image_stream();
    seek_.stop_depth_stream();
    seek_.close();
  }

private:
  void onDepth(const event_header_t& eheader, const uint8_t* data, int len) {
    const int depth_camera_number = sdev_.dev_info.depth_camera_number;
    const int height = sdev_.dev_info.depth_resolution_height / depth_camera_number;
    const int width = sdev_.dev_info.depth_resolution_width;
    
    std::vector<cv::Mat> images;
    for (int i = 0; i < depth_camera_number; i++) {
      images.emplace_back(height, width, CV_16UC1, const_cast<uint8_t*>(data) + i * height * width * 2);
    }

    // 创建消息头
    auto header = std::make_shared<std_msgs::msg::Header>();
    header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);

    // 发布视差图像
    for (size_t i = 0; i < depth_camera_number; ++i) {
      header->frame_id = "depth" + std::to_string(i);
      sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(*header, "16UC1", images[i]).toImageMsg();
      depth_pubs_[i]->publish(*img_msg);
    }

    // 发布视差消息
    for (size_t i = 0; i < depth_camera_number; ++i) {
      const int DPP = 256/4;
      const double inv_dpp = 1.0 / DPP;
      auto disparity_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
      
      // 设置视差图像
      auto& dimage = disparity_msg->image;
      dimage.header = *header;
      dimage.height = images[i].rows;
      dimage.width = images[i].cols;
      dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      dimage.step = dimage.width * sizeof(float);
      dimage.data.resize(dimage.step * dimage.height);
      
      cv::Mat_<float> dmat(images[i].rows, images[i].cols, 
                          reinterpret_cast<float*>(dimage.data.data()));
      images[i].convertTo(dmat, CV_32F, inv_dpp, 0);

      // 设置视差参数
      disparity_msg->f = 320.0;
      disparity_msg->t = 0.04625;
      disparity_msg->min_disparity = 0.0;
      disparity_msg->max_disparity = 192;
      disparity_msg->delta_d = inv_dpp;
      disparity_msg->header = *header;

      disparity_pubs_[i]->publish(*disparity_msg);
    }
  }

  void onEvent(const event_header_t& header, const device_event_t& event) {
    if (event.type == EVENT_TYPE_SENSOR_CUSTOM && pub_imu_) {
      auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
      imu_msg->header.stamp = rclcpp::Time(header.sec, header.nsec);
      imu_msg->header.frame_id = imu_link_;
      
      imu_msg->angular_velocity.x = event.event.sensor_custom.angular_velocity_x;
      imu_msg->angular_velocity.y = event.event.sensor_custom.angular_velocity_y;
      imu_msg->angular_velocity.z = event.event.sensor_custom.angular_velocity_z;

      imu_msg->linear_acceleration.x = event.event.sensor_custom.linear_acceleration_x;
      imu_msg->linear_acceleration.y = event.event.sensor_custom.linear_acceleration_y;
      imu_msg->linear_acceleration.z = event.event.sensor_custom.linear_acceleration_z;

      imu_pub_->publish(*imu_msg);
    }
  }

  void onImage(const event_header_t& eheader, const cv::Mat& frame) {
    const int cam_num = sdev_.dev_info.rgb_camera_number;
    const int h = frame.rows / cam_num;
    const int w = frame.cols;

    auto header = std::make_shared<std_msgs::msg::Header>();
    header->stamp = rclcpp::Time(eheader.sec, eheader.nsec);

    for (int i = 0; i < cam_num; i++) {
      header->frame_id = "cam" + std::to_string(i);
      auto img_msg = cv_bridge::CvImage(*header, "bgr8", frame(cv::Rect(0, i * h, w, h))).toImageMsg();

      // if (use_image_transport_) {
      //   image_pubs_it_[i].publish(img_msg);
      // } else {
        image_pubs_ros_[i]->publish(*img_msg);
      // }
    }
  }

  void onMjpeg(const event_header_t& pheader, const uint8_t* data, int len) {
    // publish compressed img
    auto compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
    compressed->header.stamp = rclcpp::Time(pheader.sec, pheader.nsec);
    compressed->format = "jpeg";
    
    compressed->data.resize(len);
    memcpy(compressed->data.data(), data, len);

    compressed_image_pub_->publish(*compressed);

    cv::Mat frame = cv::imdecode(
      cv::Mat(1, len, CV_8UC1, const_cast<uint8_t*>(data)), 
      cv::IMREAD_COLOR
    );
    
    if (!frame.empty()) {
      onImage(pheader, frame);
    }
  }

  bool onTimer(std::pair<uint64_t, uint64_t>& timer_get, std::pair<uint64_t, uint64_t>& timer_set) {
    auto now = this->now();
    timer_set.first = now.seconds();
    timer_set.second = now.nanoseconds() % 1000000000;
    return true;
  }

  // ROS2接口
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_ros_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_gray_pubs_ros_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rectify_pubs_ros_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs_;
  std::vector<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr> disparity_pubs_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::vector<image_transport::Publisher> image_pubs_it_;
  
  // 设备接口
  SEEKNS::SEEKER seek_;
  seeker_device_t sdev_;
  
  // 参数
  // bool use_image_transport_;
  bool pub_disparity_img_;
  bool pub_disparity_;
  bool pub_imu_;
  bool time_sync_;
  std::string imu_link_;
  std::string imu_topic_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeekRosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
