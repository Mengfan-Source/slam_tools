# SLAM_TOOLS(SLAM开发过程中的一些工具)
## 内容
### 1.双激光雷达标定 
### 2.NDT点云配准算法
### 3.双雷达数据融合（双livox雷达数据融合输出livox格式，或者输出pointcloud2格式）
### 4.地面点去除
### 5.根据时间戳和位姿拼接PCD点云（11所）
### 6.体素滤波
### 7.测试几个库旋转矩阵、四元数、欧拉角、李代数的互相转换关系
### 8.配准相邻两帧相机输出的点云，配准之前滤波（11所给的数据包做验证）
### 9.测试rs_lidar数据转换
## 编译安装
### 1.下载源码
```bash
git clone https://github.com/Mengfan-Source/slam_tools.git
```
### 2.准备文件目录
```bash
mkdir build
mkdir log 
mkdir data
```
### 3.相关依赖
```bash
sudo apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev
```
### 4.通过cmake, make安装包里面自带的thirdparty/g2o库(如果之前安装过g2o，则这一步省略)
```bash
cd src/thirdparty/g2o
mkdir build
cd build
cmake ..
make
sudo make install#如果之前安装过g2o就不需要安装了
```
### 5.编译执行
```bash
cd build
cmake ..
make -j8
#执行在build目录下执行
```

## 更新日志
- 20250120:在slam_test的基础上添加了11所拼接点云的程序，并编译成功
- 20250210:改进标定相关内容，并编译成功
    - 采用数学方法计算初值，方案思路：https://uw7f7qxdyrb.feishu.cn/docx/YxjGdQiv4ovVWmxZVkcc1W6WnKe?from=from_copylink  
    - 遍历两组初值分别配准：使用平均配准误差来衡量配准结果  
    - 使用yaml-cpp来规范参数读取过程  
    - 原来手动赋初值的版本备份在了：/home/xmf/xmf_slam/slam_tools/back_up/pcd_align手动提供初值版本.cpp  
- 20250316:添加NDT配准相邻两帧点云程序（11所给的数据用相邻两帧相机输出的点云数据做配准，配准初值设置为0）pcd_align_filtered.cpp
    - 添加了滤波程序，滤波操作并行处理，但是报错尚未解决
- 20250319:添加rslidar激光雷达测试程序:/home/xmf/xmf_slam/slam_tools/src/src/rslidar_test.cpp
    - 主要为了测试rslidar的pointcloud2数据字段格式及其转换
    - 适配的数据格式：XYZRTF   ----->这里需要修改驱动/home/xmf/XMF_Driver/rslidar_sdk_ws/src/rslidar_sdk/CMakeLists.txt，第八行，默认设置的是XYZI，现在设置为XYZRTF
        rslidar_sdk的readme中说明的数据格式如下：
        ```C++
        struct PointXYZIRTF
        {
        float x;
        float y;
        float z;
        uint8_t intensity;
        uint16_t ring;
        double timestamp;
        uint8_t feature;
        };
        ```
        但是实际测试时候发现只有将intensity和feature字段改为float类型才可以正常接收使用，具体见/home/xmf/xmf_slam/slam_tools/src/src/rslidar_test.cpp中PCL数据点的定义
    - 一帧数据中每个点的偏移时间与其他雷达不同，rs_lidar的偏移时间是运行时刻递增的，而不是一个单纯的偏移量，因此在接入使用IMU进行运动畸变去除的SLAM算法时需要将每个点的偏移时间设置为当前点的时间减去第一个点的相对时间。
    - 这个激光雷达一帧数据中每个点的偏移时间单位是秒
- 20250327:添加在线激光雷达数据点拼接程序（11所项目需要）merge_points_online.cpp
    - 在merge_single_point_11基础上修改为点云在线拼接版本:订阅fast_lio的Odometry话题消息，拼接相机点云
    - 插值方法尝试没成功，使用变量更新方式更新实时位姿
    - 对于相机点云拖尾和噪点现象的滤波处理(以下滤波器件顺序处理)
        - 跳点滤波
        - StatisticalOutlierRemoval利群点去除
        - RadiusOutlierRemoval半径滤波
        - 体素滤波
        - 直通滤波、滤除固定角度的点(LOAM思想)
    - 启动步骤
    ```bash
    roscore

    cd /home/xmf/xmf_slam/slam_tools/build
    ./merge_points_online
    
    cd /home/xmf/xmf_project/cetc11/fast_lio_cetc
    roslaunch fast_lio my_test_x30.launch

    rviz :/home/xmf/xmf_project/cetc11/fast_lio_cetc/test.rviz
    
    ```
- 20251118:添加从bag包查看pcl::pointcloud2类型点云数据字段格式
    - 使用方法: python/pcl2_msg_check.py文件中修改点云bag包路径
                python3 pcl2_msg_check.py
