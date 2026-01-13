/**
 * @file lidar_fusion.cpp
 * @brief 多个mid360激光雷达点云数据融合
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2024-1021
 * @copyright Copyright (C) 2024 中国电科科技集团公司第二十一研究所
 */
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <execution>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
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

#define blind 0.5
#define N_SCANS 4
#define point_filter_num 3
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
PointCloudXYZI pl_full, pl_surf;
ros::Publisher pub;
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
ros::Subscriber livox_lidar_sub;
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
        cloud.fields [4].name = "tag";
        cloud.fields [4].count = 1;
        cloud.fields [4].datatype = sensor_msgs::PointField::UINT8;
        cloud.fields [5].offset = 17;
        cloud.fields [5].name = "line";
        cloud.fields [5].count = 1;
        cloud.fields [5].datatype = sensor_msgs::PointField::UINT8;
        cloud.fields [6].offset = 18;
        cloud.fields [6].name = "timestamp";
        cloud.fields [6].count = 1;
        cloud.fields [6].datatype = sensor_msgs::PointField::FLOAT64;
        cloud.point_step = sizeof (LivoxPointXyzrtlt);
}


void LivoxCbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg){
    int plsize = msg->point_num;
    pl_full.clear();
    pl_surf.reserve(plsize);
    sensor_msgs::PointCloud2 out;
    InitPointcloud2MsgHeader(out,"livox_frame");
    out.point_step = sizeof(LivoxPointXyzrtlt);
    out.width = plsize;
    out.row_step = out.point_step * out.width;
    out.is_bigendian=false;
    out.is_dense = true;
    std::vector<LivoxPointXyzrtlt> temp_points;
    for(size_t i = 0;i<plsize;i++){
        LivoxPointXyzrtlt temp_point;
        temp_point.x = msg->points[i].x;
        temp_point.y = msg->points[i].y;
        temp_point.z = msg->points[i].z;
        temp_point.reflectivity = msg->points[i].reflectivity;
        temp_point.tag = msg->points[i].tag;
        temp_point.line = msg->points[i].line;
        temp_point.timestamp = static_cast<double>(msg->points [i].offset_time);
        temp_points.push_back (std::move (temp_point));
    }
    out.data.resize (plsize*sizeof(LivoxPointXyzrtlt));
    memcpy(out.data.data (), temp_points.data (), plsize*sizeof (LivoxPointXyzrtlt));
    // for(uint i=1; i<plsize; i++){
    //     pl_full[i].x = msg->points[i].x;
    //     pl_full[i].y = msg->points[i].y;
    //     pl_full[i].z = msg->points[i].z;
    //     pl_full[i].intensity = msg->points[i].reflectivity;
    //     pl_full[i].curvature = msg->points[i].offset_time / float(1000000); 
    // }
    out.header.stamp = msg->header.stamp;
    pub.publish(out);
}
int main(int argc, char** argv){
        ros::init(argc, argv, "lidar_fusion");
        ros::NodeHandle nh;
        pub= nh.advertise<sensor_msgs::PointCloud2>("/lidar_pcl2", 10);
        livox_lidar_sub = nh.subscribe("/livox/lidar", 10,LivoxCbk);
        ros::spin();

        return 0;
}
  