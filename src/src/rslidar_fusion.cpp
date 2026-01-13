/**
 * @file lidar_fusion.cpp
 * @brief 多个rslidar数据融合
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2026-0108
 * @copyright Copyright (C) 2026 中国电科科技集团公司第二十一研究所
 */
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <execution>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <execution>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<message_filters/sync_policies/exact_time.h>

#include "eigen_types.h"
#include "point_types.h"
#include <iomanip>

#define blind 0.5
#define N_SCANS 4
#define point_filter_num 3
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0/M_PI ;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
ros::Publisher lidar_pub;
ros::Subscriber imu_sub;
ros::Publisher imu_pub;
Mat4d T_imu110_148;
Mat4d T_lidar2imu;
Mat4d T_imu110_148_inv ;
Mat4d T_lidar2imu_inv;
Mat4d T_lidar110_lidar148;
typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
  double timestamp;   /**< Timestamp of point*/
} LivoxPointXyzrtlt;
namespace airy_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint16_t ring = 0;
        double timestamp = 0;
        std::uint8_t feature;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace airy_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(airy_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
    (std::uint8_t, feature, feature)
)
void InitPointcloud2MsgHeader(sensor_msgs::PointCloud2& cloud, const std::string& frame_id) {
        cloud.header.frame_id.assign(frame_id);
        cloud.height = 1;
        cloud.width = 0;
        cloud.fields.resize (7);
        cloud.fields [0].offset = 0;
        cloud.fields [0].name = "x";
        cloud.fields [0].count = 1;
        cloud.fields [0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields [1].offset = 4;
        cloud.fields [1].name = "y";
        cloud.fields [1].count = 1;
        cloud.fields [1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields [2].offset = 8;
        cloud.fields [2].name = "z";
        cloud.fields [2].count = 1;
        cloud.fields [2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields [3].offset = 12;
        cloud.fields [3].name = "intensity";
        cloud.fields [3].count = 1;
        cloud.fields [3].datatype = sensor_msgs::PointField::FLOAT32;
        cloud.fields [4].offset = 16;
        cloud.fields [4].name = "ring";
        cloud.fields [4].count = 1;
        cloud.fields [4].datatype = sensor_msgs::PointField::UINT16;
        cloud.fields [5].offset = 18;
        cloud.fields [5].name = "timestamp";
        cloud.fields [5].count = 1;
        cloud.fields [5].datatype = sensor_msgs::PointField::FLOAT64;
        cloud.fields [6].offset = 26;
        cloud.fields [6].name = "feature";
        cloud.fields [6].count = 1;
        cloud.fields [6].datatype = sensor_msgs::PointField::UINT8;
        cloud.point_step = sizeof(airy_ros::Point);
}
void rs_callback(const sensor_msgs::PointCloud2::ConstPtr & rs_msg_front, const sensor_msgs::PointCloud2::ConstPtr& rs_msg_back) {
    unsigned int point_count =0;
    sensor_msgs::PointCloud2 ros_msg;
    // InitPointcloud2MsgHeader(ros_msg,"base_link");    
    pcl::PointCloud<airy_ros::Point> pcl_f;
    pcl::PointCloud<airy_ros::Point> pcl_b;
    pcl::PointCloud<airy_ros::Point> pcl_all;
    pcl::PointCloud<airy_ros::Point> pcl_sorted;
    pcl::fromROSMsg(*rs_msg_front, pcl_f);
    pcl::fromROSMsg(*rs_msg_back, pcl_b);
    std::vector<std::pair<int, double>> time_indices;
    time_indices.reserve(pcl_f.points.size()+pcl_b.points.size());
    pcl_sorted.reserve(pcl_f.points.size()+pcl_b.points.size());
    pcl_all.reserve(pcl_f.points.size()+pcl_b.points.size());
    if(rs_msg_front->header.stamp < rs_msg_back->header.stamp){
        double time_diff = rs_msg_back->header.stamp.toSec() - rs_msg_front->header.stamp.toSec();
        for(size_t i = 0;i<pcl_f.points.size();i++){
            airy_ros::Point p = pcl_f.points[i];
            pcl_all.push_back(p);
            time_indices.emplace_back(point_count,p.timestamp);
            point_count++;
        }
        for(size_t i = 0;i<pcl_b.points.size();i++){
            airy_ros::Point p = pcl_b.points[i];
            p.ring = p.ring+96;
            // p.timestamp = p.timestamp+time_diff;
            pcl_all.push_back(p);
            time_indices.emplace_back(point_count,p.timestamp);
            point_count++;
        }
    }
    else{
        double time_diff = rs_msg_front->header.stamp.toSec() - rs_msg_back->header.stamp.toSec();
        for(size_t i = 0;i<pcl_b.points.size();i++){
            airy_ros::Point p = pcl_b.points[i];
            p.ring = p.ring+96;
            pcl_all.push_back(p);
            time_indices.emplace_back(point_count,p.timestamp);
            point_count++;
        }
        for(size_t i = 0;i<pcl_f.points.size();i++){
            airy_ros::Point p = pcl_f.points[i];
            // p.timestamp = p.timestamp+time_diff;
            pcl_all.push_back(p);
            time_indices.emplace_back(point_count,p.timestamp);
            point_count++;
        }
    }
    std::sort(time_indices.begin(), time_indices.end(),[](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second < b.second;
    });
    std::vector<airy_ros::Point> temp_points;
    for(size_t i = 0; i < time_indices.size(); ++i){
        pcl_sorted.push_back(pcl_all.points[time_indices[i].first]);
        airy_ros::Point p = pcl_sorted.points[i];
        temp_points.push_back(p);
    }
    pcl::toROSMsg(pcl_sorted, ros_msg);
    ros_msg.header.frame_id = "base_link";
    ros_msg.header.stamp =  rs_msg_front->header.stamp.toSec() < rs_msg_back->header.stamp.toSec() ? rs_msg_front->header.stamp:rs_msg_back->header.stamp;
    lidar_pub.publish(ros_msg);
    pcl::PointCloud<airy_ros::Point> pcl_test;
    pcl::fromROSMsg(ros_msg, pcl_test);
    pcl_sorted.width    = pcl_sorted.size();
    pcl_sorted.height   = 1;
    pcl_sorted.is_dense = false;
    // for(size_t i = 0;i<pcl_test.points.size();i++){
    //     airy_ros::Point p = pcl_test.points[i];
    //     std::cout<< std::fixed << std::setprecision(25)<<p.timestamp<<std::endl;
    // }
}
void ProcessImu(const sensor_msgs::ImuConstPtr& msg_in){
    Eigen::Matrix4d lidar2imu = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_lidar2imu(0.0029166745953261852,0.7073081731796265,-0.7068824768066406,0.004880243446677923);
    Eigen::Matrix3d R_lidar2imu = q_lidar2imu.toRotationMatrix();
    lidar2imu.block<3,3>(0,0) = R_lidar2imu;
    lidar2imu.block<3,1>(0,3) = Eigen::Vector3d(0.00425,0.00418,-0.004460);
    Eigen::Vector3d rpy_lidar2base(0,-1.57079,-3.14159);
    Eigen::Vector3d t_lidar2base(0.32028,0,-0.013);
    Eigen::Matrix3d R_lidar2base;
    R_lidar2base = (Eigen::AngleAxisd(rpy_lidar2base(2), Eigen::Vector3d::UnitZ())//Z
      * Eigen::AngleAxisd(rpy_lidar2base(1), Eigen::Vector3d::UnitY())//Y
      * Eigen::AngleAxisd(rpy_lidar2base(0), Eigen::Vector3d::UnitX())).toRotationMatrix();//X
    Eigen::Matrix4d lidar2base = Eigen::Matrix4d::Identity();
    lidar2base.block<3,3>(0,0) = R_lidar2base;
    lidar2base.block<3,1>(0,3) = t_lidar2base;


    // Eigen::Vector4d acc(msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z,1.0);
    // Eigen::Vector4d acc_base = lidar2base*lidar2imu.inverse()*acc;
    Eigen::Vector3d acc(msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z);
    Eigen::Vector3d acc_base = R_lidar2base*R_lidar2imu.inverse()*acc;
    // Eigen::Vector3d acc_base = R_lidar2imu.inverse()*acc;
    Eigen::Vector3d gyr(msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z);
    Eigen::Vector3d gyr_base = R_lidar2base*R_lidar2imu.inverse()*gyr;
    // Eigen::Vector3d gyr_base = R_lidar2imu.inverse()*gyr;
    sensor_msgs::Imu msg;
    msg = *msg_in;
    msg.angular_velocity.x = gyr_base(0);
    msg.angular_velocity.y = gyr_base(1);
    msg.angular_velocity.z = gyr_base(2);
    msg.linear_acceleration.x = acc_base(0);
    msg.linear_acceleration.y = acc_base(1);
    msg.linear_acceleration.z = acc_base(2);
    msg.header.frame_id = "base_link";
    imu_pub.publish(msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_fusion");
    ros::NodeHandle nh;

    std::cout << "start to merge point cloud......" << std::endl;
    lidar_pub= nh.advertise<sensor_msgs::PointCloud2>("/lidar_fusion_pcl", 10);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/lidar_imu",10);
    imu_sub = nh.subscribe("/rslidar_imu_data_f",200000,ProcessImu);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sublidar_front(nh, "/rslidar_points_f", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sublidar_back(nh, "/rslidar_points_b", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> syncPolicy; //近似同步策略
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sublidar_front, sublidar_back);// 同步,允许最大时间差异为10毫秒
    sync.setMaxIntervalDuration(ros::Duration(0.05));
    sync.registerCallback(boost::bind(&rs_callback, _1, _2));
    ros::spin();
    return 0;
}
  