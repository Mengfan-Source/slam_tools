#include <iostream>	
#include <cmath>       
#include <pcl/point_types.h>	    
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iomanip> 
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include "eigen_types.h"
#include "point_types.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <deque>
#include <nav_msgs/Odometry.h>
ros::Subscriber sub_camera_points;
ros::Subscriber sub_odom;
ros::Publisher pub_points;
std::deque<double> time_buffer_;
std::deque<std::vector<double>> odom_buffer_; 
std::deque<xmf::CloudPtr> camera_buffer_;
SE3 findpose_in_time(std::deque<std::vector<double>> &lio_poses, double time_s)
{
//     double time_s = time/1000.0;
    // std::cout<<"time_s: "<<time_s<<std::endl;
    double time_diff = fabs(lio_poses[0][0]-time_s);
    if(time_diff<1e-6){//第一帧返回的不是单位阵而是第一帧存进去的位姿
        Quatd quat(lio_poses[0][7],lio_poses[0][4],lio_poses[0][5],lio_poses[0][6]);
        Eigen::Matrix<double, 3, 1> vector3d(lio_poses[0][1],lio_poses[0][2],lio_poses[0][3]);
        return(SE3(quat,vector3d));
    }

    Quatd quat(1,0,0,0);
    Mat3d rotationMatrix = quat.toRotationMatrix();
    Mat4d curpose = Mat4d::Identity(); 
    SE3 curposese3;

    for(int i  = 0;i<lio_poses.size();i++) 
    {
        if(lio_poses[i][0]-time_s>0.2)
            break;
        else if(fabs(lio_poses[i][0]-time_s)<=0.2)
        {
            if(time_diff>fabs(lio_poses[i][0]-time_s))
            {
                // std::cout<<"time diff: "<<time_diff<<std::endl;
                time_diff = fabs(lio_poses[i][0]-time_s);
                quat.w() = lio_poses[i][7];
                quat.x() = lio_poses[i][4];
                quat.y() = lio_poses[i][5];
                quat.z() = lio_poses[i][6];
                rotationMatrix = quat.toRotationMatrix();
                Eigen::Matrix<double, 3, 1> vector3d(lio_poses[i][1],lio_poses[i][2],lio_poses[i][3]);
                curpose.block<3, 3>(0, 0) = rotationMatrix;
                curpose.block<3,1>(0,3) = vector3d;
                SE3 temp(quat,vector3d);
                curposese3 = temp;
            }
        }
        else 
            continue;
        
    }
    return curposese3;
}
xmf::CloudPtr ProcessCameraCloud(xmf::CloudPtr cloud){
    pcl::VoxelGrid<xmf::PointType> sor_source;  
    xmf::CloudPtr source_filtered(new xmf::PointCloudType);//体素滤波点云
    sor_source.setInputCloud(cloud);  
    sor_source.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素的大小，这里是1cm  
    sor_source.filter(*source_filtered); 

    //滤除50米以外且在固定视角范围内的点
    xmf::CloudPtr source_valid(new xmf::PointCloudType);
    for(int i = 0;i<source_filtered->points.size(); i++){
        xmf::PointType point = source_filtered->points[i];
        float angleZ = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        float angleY = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
        if(sqrt(point.x*point.x+point.y*point.y+point.z*point.z)<20 && std::abs(angleY)<20 && std::abs(angleZ)<20){
            source_valid->push_back(point);
        }
    }
    return source_valid;
}
//位姿插值：平移线性差值、旋转球面线性差值
template <typename T, typename C, typename FT, typename FP>
bool PoseInterp(double query_time, C&& data, FT&& take_time_func, FP&& take_pose_func, SE3& result,
                       T& best_match, float time_th = 0.5) {
    if (data.empty()) {
        LOG(INFO) << "cannot interp because data is empty. ";
        return false;
    }
    //last_time+time_th>query_time>last_time时认为超时可以接受，直接返回最末尾时间的状态，否则返回truth
    double last_time = take_time_func(*data.rbegin());
    if (query_time > last_time) {
        if (query_time < (last_time + time_th)) {
            // 尚可接受
            result = take_pose_func(*data.rbegin());
            best_match = *data.rbegin();
            return true;
        }
        return false;
    }

    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (take_time_func(*iter) < query_time && take_time_func(*next_iter) >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;

    double dt = take_time_func(*match_iter_n) - take_time_func(*match_iter);
    double s = (query_time - take_time_func(*match_iter)) / dt;  // s=0 时为第一帧，s=1时为next
    // 出现了 dt为0的bug
    if (fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }

    SE3 pose_first = take_pose_func(*match_iter);
    SE3 pose_next = take_pose_func(*match_iter_n);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? *match_iter : *match_iter_n;
    return true;
}
void cameraHandler(const sensor_msgs::PointCloud2::ConstPtr &pc_msg){
    xmf::CloudPtr ori_points(new xmf::PointCloudType);
    xmf::CloudPtr points(new xmf::PointCloudType);
    xmf::CloudPtr points_transformed(new xmf::PointCloudType);
    pcl::fromROSMsg(*pc_msg,*ori_points);
    points = ProcessCameraCloud(ori_points);
    if(odom_buffer_.empty()){
        pcl::transformPointCloud(*points,*points_transformed,Eigen::Matrix4f::Identity());
    }
    else{
        // camera_buffer_.push_back(points);
        // time_buffer_.push_back(pc_msg->header.stamp.toSec());
        double curtime = pc_msg->header.stamp.toSec();
        SE3 cur_pose = findpose_in_time(odom_buffer_,curtime);
        std::vector<double> match;
        if(!PoseInterp<std::vector<double>>(curtime,odom_buffer_,[](const std::vector<double> &s) { return s[0]; },[](const std::vector<double> &s) { 
            Quatd quat(1,0,0,0);
            Mat4d curpose = Mat4d::Identity(); 
                        // Mat3d rotationMatrix = quat.toRotationMatrix();
            SE3 curposese3;
            quat.w() = s[7];
            quat.x() = s[4];
            quat.y() = s[5];
            quat.z() = s[6];
            Mat3d rotationMatrix = quat.toRotationMatrix();
            Eigen::Matrix<double, 3, 1> vector3d(s[1],s[2],s[3]);
            curpose.block<3, 3>(0, 0) = rotationMatrix;
            curpose.block<3,1>(0,3) = vector3d;
            SE3 temp(quat,vector3d);
            curposese3 = temp;
            return curposese3; 
            },cur_pose, match)){
                cur_pose = findpose_in_time(odom_buffer_,curtime);
        }
            // std::cout<<"found_pose: "<<cur_pose.matrix().cast<double>()<<std::endl;
        pcl::transformPointCloud(*points,*points_transformed,cur_pose.matrix().cast<double>());
    }
    sensor_msgs::PointCloud2 pubCloudmsg;
    pcl::toROSMsg(*points_transformed, pubCloudmsg);
    // pubCloudmsg.header.stamp = ros::Time::now();
    pubCloudmsg.header.stamp = pc_msg->header.stamp;
    pubCloudmsg.header.frame_id = "map";
    pub_points.publish(pubCloudmsg);
}
void odomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg){
    std::vector<double> o;
    o.push_back(odom_msg->header.stamp.toSec());
    o.push_back(odom_msg->pose.pose.position.x);
    o.push_back(odom_msg->pose.pose.position.y);
    o.push_back(odom_msg->pose.pose.position.z);
    o.push_back(odom_msg->pose.pose.orientation.x);
    o.push_back(odom_msg->pose.pose.orientation.y);
    o.push_back(odom_msg->pose.pose.orientation.z);
    o.push_back(odom_msg->pose.pose.orientation.w);
    odom_buffer_.push_back(o);
    if(odom_buffer_.size()>20){
        odom_buffer_.pop_front();
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rs_lidar_test");
    ros::NodeHandle nh;

    sub_odom = nh.subscribe("/Odometry",1,odomHandler);
    sub_camera_points = nh.subscribe("/camera_points",1,cameraHandler);
    pub_points = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 100000);

    ROS_INFO("Listening to /rslidar_points ......");
    ros::spin();
    return 0;
}
