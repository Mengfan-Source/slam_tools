# SLAM_TOOLS(SLAM开发过程中的一些工具)
## 内容
### 1.双激光雷达标定 
### 2.NDT点云配准算法
### 3.双雷达数据融合（双livox雷达数据融合输出livox格式，或者输出pointcloud2格式）
### 4.地面点去除
### 5.根据时间戳和位姿拼接PCD点云（11所）
### 6.体素滤波
### 7.测试几个库旋转矩阵、四元数、欧拉角、李代数的互相转换关系

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
20250120:在slam_test的基础上添加了11所拼接点云的程序，并编译成功
20250210:改进标定相关内容，并编译成功
    采用数学方法计算初值，方案思路：https://uw7f7qxdyrb.feishu.cn/docx/YxjGdQiv4ovVWmxZVkcc1W6WnKe?from=from_copylink
    遍历两组初值分别配准：使用平均配准误差来衡量配准结果
    使用yaml-cpp来规范参数读取过程

    原来手动赋初值的版本备份在了：/home/xmf/xmf_slam/slam_tools/back_up/pcd_align手动提供初值版本.cpp