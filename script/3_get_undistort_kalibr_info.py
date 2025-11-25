import yaml
import ruamel.yaml
import numpy as np
import sys
import os

ruamelyaml = ruamel.yaml.YAML()
ruamelyaml.default_flow_style = None
np.set_printoptions(suppress =True)

def inv_T(T):
    # 求逆操作并保持左下三个元素为0
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = np.linalg.inv(R)
    t_inv = np.dot(-R_inv, t)
    T_inv = np.mat(np.zeros((4, 4)))
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    T_inv[3, 3] = 1
    return T_inv

def getcam0_Tcn_cm1(datain, cam_index_max):
    Tcam0_cncnm1 = np.mat(np.eye(4), dtype=np.float64)
    for i in range(1, cam_index_max+1):
        T_cn_cnm1 = np.mat(np.resize(np.array(datain['cam'+str(i)]['T_cn_cnm1'], dtype=np.float64), (4, 4)))
        Tcam0_cncnm1 = Tcam0_cncnm1.dot(inv_T(T_cn_cnm1))
    print("Tcam0_cncnm1=")
    print(Tcam0_cncnm1)
    return Tcam0_cncnm1
    # Rt0 = np.mat(np.resize(np.array(cam0T_cam_imu), (4, 4)))
    # datain[camright_namespace]['T_cam_imu']

def get_T_cam_imu(datain, cam_index):
    if 'cam' + str(cam_index) in datain:
        if 'T_cam_imu' in datain['cam'+str(cam_index)]:
            T_cam_imu = datain['cam'+str(cam_index)]['T_cam_imu']
            Rt = np.mat(np.resize(np.array(T_cam_imu, dtype=np.float64), (4, 4)))
            return Rt
        else :
            if 'cam' + str(cam_index - 1) in datain:
                Rt_pre = get_T_cam_imu(datain, cam_index - 1)
                T_cn_cnm1 = np.mat(np.resize(np.array(datain['cam'+str(cam_index)]['T_cn_cnm1'], dtype=np.float64), (4, 4)))
                Rt = T_cn_cnm1.dot(Rt_pre)
                return Rt
    print("error")
    return np.mat(np.eye(4), dtype=np.float64)

def generatestereoinfo(datain, dataout,camleft_namespace, camright_namespace, frame, out_left_namespace, out_right_namespace):
    # 处理cam0数据
    Rt0 = get_T_cam_imu(datain, camleft_namespace)
    Rt1 = get_T_cam_imu(datain, camright_namespace)

    cam1T_cn_cnm1 = Rt1.dot(inv_T(Rt0))

    x = inv_T(cam1T_cn_cnm1)[0:3,3].flatten()
    #x = np.array([0, 0, 0]) - cam1T_cn_cnm1[0:3,3].flatten()
    y = np.cross(np.eye(4, dtype=np.float64)[0:3, 2], x)
    z = np.cross(x, y)

    T = np.mat(np.eye(4))
    T[0:3,0] = np.resize(x / np.linalg.norm(x), (3,1))
    T[0:3,1] = np.resize(y / np.linalg.norm(y), (3,1))
    T[0:3,2] = np.resize(z / np.linalg.norm(z), (3,1))
    
    Tinv = inv_T(T)
    print("Tinv")
    print(Tinv)

    Rtl = inv_T(T).dot(Rt0)
    Rtr = T.dot(Rt1)

    print("aaaaaaaaaaa")
    print(cam1T_cn_cnm1.dot(T))
    print(T)
    print(inv_T(T))


    cam1T_cn_cnm1_ = Rtr.dot(inv_T(Rtl))
    print("cam1T_cn_cnm1_\n", cam1T_cn_cnm1_)
    
    Rtr = inv_T(cam1T_cn_cnm1).dot(Rtl)
    cam1T_cn_cnm1_ = Rtr.dot(inv_T(Rtl))
    print("cam1T_cn_cnm1_\n", cam1T_cn_cnm1_)

    Rt_cnm1 = np.mat(np.eye(4))
    Rt_cnm1[0,3] = -np.linalg.norm(cam1T_cn_cnm1[0:3,3]) #-np.linalg.norm(Rt0[0:3,3]-Rt1[0:3,3])
    #print( Rt_cnm1)
    Rtr = Rt_cnm1.dot(Rtl)
    cam1T_cn_cnm1_ = Rtr.dot(inv_T(Rtl))
    print("cam1T_cn_cnm1_\n", cam1T_cn_cnm1_)



    #Rt_cnm1 = np.mat(np.eye(4))
    #Rt_cnm1[0,3] = -np.linalg.norm(cam1T_cn_cnm1[0:3,3]) #-np.linalg.norm(Rt0[0:3,3]-Rt1[0:3,3])
    ##print( Rt_cnm1)
    #Rtr = Rt_cnm1.dot(Rtl)
    ##Rtr = Tinv.dot(Rt1)

    print(out_left_namespace+' T=')
    print(Rtl)
    print(out_right_namespace+' T=')
    print(Rtr)
    dataout[out_left_namespace] = dict()
    dataout[out_left_namespace]['T_cam_imu'] = Rtl.tolist()
    dataout[out_left_namespace]['T_imu_cam'] = inv_T(Rtl).tolist()
    
    dataout[out_left_namespace]['camera_model'] = 'pinhole'
    dataout[out_left_namespace]['distortion_coeffs'] = [0, 0, 0, 0]
    dataout[out_left_namespace]['distortion_model']= 'radtan'
    dataout[out_left_namespace]['intrinsics']= [320.0,320.0,640.0/2.0,480.0/2.0]
    dataout[out_left_namespace]['resolution']= [640, 480]
    dataout[out_left_namespace]['rostopic'] = '/' + frame + '/left/image_raw'
    dataout[out_left_namespace]['timeshift_cam_imu'] = 0.0
    dataout[out_left_namespace]['frame'] = 'cam' + str(camleft_namespace)+'r'

    dataout[out_right_namespace] = dict()
    dataout[out_right_namespace]['T_cam_imu'] = Rtr.tolist()
    dataout[out_right_namespace]['T_imu_cam'] = inv_T(Rtr).tolist()
    
    dataout[out_right_namespace]['camera_model'] = 'pinhole'
    dataout[out_right_namespace]['distortion_coeffs'] = [0,0,0,0]
    dataout[out_right_namespace]['distortion_model']= 'radtan'
    dataout[out_right_namespace]['intrinsics']= [320.0,320.0,640.0/2.0,480.0/2.0]
    dataout[out_right_namespace]['resolution']= [640, 480]
    dataout[out_right_namespace]['rostopic'] = '/' + frame + '/right/image_raw'
    dataout[out_right_namespace]['timeshift_cam_imu'] = 0.0
    dataout[out_right_namespace]['frame'] = 'cam' + str(camright_namespace)+'l'

def main():
    args = sys.argv
    if (len(args) < 2):
        print("input kalibr yaml path!")
        return

    # with open('2024-01-31-17-26-37-camchain.yaml', 'r') as f:
    with open(args[1], 'r') as f:
        datain = yaml.load(f, Loader=yaml.FullLoader)
        dataout=dict()
        print(datain)

        if 'T_cn_cnm1' not in datain['cam0']:
            datain['cam0']['T_cn_cnm1'] = getcam0_Tcn_cm1(datain, 3).tolist()
            with open('kalibr_cam_chain.yaml', 'w+') as g:
                ruamelyaml.dump(datain, g)
        if 'T_cam_imu' not in datain['cam2']:
            datain['cam2']['T_cam_imu'] = get_T_cam_imu(datain, 2).tolist()
            with open('kalibr_cam_chain.yaml', 'w+') as g:
                ruamelyaml.dump(datain, g)
        if 'T_cam_imu' not in datain['cam3']:
            datain['cam3']['T_cam_imu'] = get_T_cam_imu(datain, 3).tolist()
            with open('kalibr_cam_chain.yaml', 'w+') as g:
                ruamelyaml.dump(datain, g)

        generatestereoinfo(datain, dataout, 0, 1, 'front', 'cam0', 'cam1')
        generatestereoinfo(datain, dataout, 1, 2, 'right', 'cam2', 'cam3')
        generatestereoinfo(datain, dataout, 2, 3, 'back', 'cam4', 'cam5')
        generatestereoinfo(datain, dataout, 3, 0, 'left', 'cam6', 'cam7')

        with open('kalibr_imucam_chain.yaml', 'w+') as g:
            ruamelyaml.dump(dataout, g)

        # 打开源文件和目标文件
        with open('kalibr_imucam_chain.yaml', 'r') as source_file, open('.tmpkalibr_imucam_chain.yaml', 'w') as target_file:
            target_file.write("%YAML:1.0\n")
            target_file.write("\n")
            # 遍历源文件的每一行
            for line in source_file:
                # 将需要替换的字符串用新的字符串替换
                new_line = line.replace('  - [', '    - [')
                # 将替换后的行写入目标文件
                target_file.write(new_line)
        # 将目标文件重命名为源文件
        os.rename('.tmpkalibr_imucam_chain.yaml', 'kalibr_imucam_chain.yaml')

if __name__ == '__main__':
    main()
