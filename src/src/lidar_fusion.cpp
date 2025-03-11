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
//ivox_ros_driver2::CustomMsg::ConstPtr类型转为PointCloudXYZI::Ptr &pcl_out类型
void processmid360(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
        pl_surf.clear();
        pl_full.clear();
        int plsize = msg->point_num;
        pl_surf.reserve(plsize);
        pl_full.resize(plsize);
        unsigned int valid_num = 0;
        for(unsigned int i=1; i<plsize; i++)
        {
                if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
                {
                        valid_num ++;
                        if (valid_num % point_filter_num == 0)
                        {
                                pl_full[i].x = msg->points[i].x;
                                pl_full[i].y = msg->points[i].y;
                                pl_full[i].z = msg->points[i].z;
                                pl_full[i].intensity = msg->points[i].reflectivity;
                                pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 用这个存储每个激光点的时间，单位：ms
                                if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)|| (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))&& (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
                                {
                                        pl_surf.push_back(pl_full[i]);
                                }
                        }
                }
        }
        *pcl_out = pl_surf;
}
//融合两个livox_ros_driver2::CustomMsg类型的点云数据
void callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg110, const livox_ros_driver2::CustomMsg::ConstPtr& msg148) {
       /*直接livox_ros_driver2::CustomMsg数据类型相互转换，融合后的点云按照时间戳进行对齐*/
       livox_ros_driver2::CustomMsg msgout;
        //对重新的livox点云进行时间戳排序
       std::vector<std::pair<int, float>> time_indices;
       time_indices.reserve(msg110->points.size()+msg148->points.size());
       unsigned int point_count =0;

        if(msg110->header.stamp < msg148->header.stamp){//赋值给msgout的时间戳为两个点云时间戳中较小的那个
                double time_diff = msg148->header.stamp.toSec() - msg110->header.stamp.toSec();
                for(size_t i = 0;i<msg110->points.size();i++){
                        livox_ros_driver2::CustomPoint p = msg110->points[i];
                        Vec4d point(p.x,p.y,p.z,1.0);
                        Vec4d point_tras = T_lidar110_lidar148 * point;
                        p.x = point_tras(0);
                        p.y = point_tras(1);
                        p.z = point_tras(2);
                        p.line = p.line+N_SCANS;
                        msgout.points.push_back(p);
                        //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, p.offset_time);
                        point_count++;
                }
                for(size_t i = 0;i<msg148->points.size();i++){
                        livox_ros_driver2::CustomPoint p = msg148->points[i];
                        p.offset_time+= time_diff*1000000000;
                        msgout.points.push_back(p);
                        //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, p.offset_time);
                        point_count++;
                }
        }
        else{
                double time_diff = msg110->header.stamp.toSec() - msg148->header.stamp.toSec();
                for(size_t i = 0;i<msg148->points.size();i++){
                        msgout.points.push_back(msg148->points[i]);
                         //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, msg148->points[i].offset_time);
                        point_count++;
                }
                for(size_t i = 0;i<msg110->points.size();i++){
                        livox_ros_driver2::CustomPoint p = msg110->points[i];
                        p.offset_time+= time_diff*1000000000;
                        Vec4d point(p.x,p.y,p.z,1.0);
                        Vec4d point_tras = T_lidar110_lidar148 * point;
                        p.x = point_tras(0);
                        p.y = point_tras(1);
                        p.z = point_tras(2);
                        p.line = p.line+4;
                        msgout.points.push_back(p);
                        //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, p.offset_time);
                        point_count++;
                }
        }
        //按照时间戳进行索引排序
        std::sort(time_indices.begin(), time_indices.end(),[](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second < b.second;
         });
        //设置按照时间戳排序后的点云
        livox_ros_driver2::CustomMsg sorted_msgout;
        sorted_msgout.points.resize(msgout.points.size());
        for(size_t i = 0; i < time_indices.size(); ++i){
                sorted_msgout.points[i] = msgout.points[time_indices[i].first];
        }
        //设置按照时间戳排序后的点云属性
        sorted_msgout.point_num = sorted_msgout.points.size();
        sorted_msgout.header.frame_id = "livox_frame_148";
        sorted_msgout.header.stamp = msg110->header.stamp < msg148->header.stamp ? msg110->header.stamp:msg148->header.stamp;
        //设置按照时间排序前的点云
        msgout.point_num = msgout.points.size();
        msgout.header.frame_id = "livox_frame_148";
        msgout.header.stamp = msg110->header.stamp < msg148->header.stamp ? msg110->header.stamp:msg148->header.stamp;
        //发布按照时间戳排序之后的点云
        pub.publish(sorted_msgout);
}
//版本一：直接融合两个pcl2数据
void callback_pcl2(const sensor_msgs::PointCloud2::ConstPtr & msg110, const sensor_msgs::PointCloud2::ConstPtr& msg148) {
        xmf::PointCloudType pcl110, pcl148,pclout,pcl110_trans;
        pcl::fromROSMsg(*msg110, pcl110);
        pcl::fromROSMsg(*msg148, pcl148);
        pcl::transformPointCloud(pcl110, pcl110_trans, T_lidar110_lidar148);
        pclout= pcl110_trans + pcl148;
        sensor_msgs::PointCloud2 msgout;
        pcl::toROSMsg(pclout, msgout);
        msgout.header.frame_id = "livox_frame_148";
        msgout.header.stamp = msg110->header.stamp < msg148->header.stamp ? msg110->header.stamp:msg148->header.stamp;
        pub.publish(msgout);
}
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
//版本二：合并两个livox点云并转化为pcl2发布出去
void callback_livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr& msg110, const livox_ros_driver2::CustomMsg::ConstPtr& msg148) {
       livox_ros_driver2::CustomMsg msgout;
        //对重新的livox点云进行时间戳排序
       std::vector<std::pair<int, float>> time_indices;
       time_indices.reserve(msg110->points.size()+msg148->points.size());
       unsigned int point_count =0;

        if(msg110->header.stamp < msg148->header.stamp){//赋值给msgout的时间戳为两个点云时间戳中较小的那个
                double time_diff = msg148->header.stamp.toSec() - msg110->header.stamp.toSec();
                for(size_t i = 0;i<msg110->points.size();i++){
                        livox_ros_driver2::CustomPoint p = msg110->points[i];
                        Vec4d point(p.x,p.y,p.z,1.0);
                        Vec4d point_tras = T_lidar110_lidar148 * point;
                        p.x = point_tras(0);
                        p.y = point_tras(1);
                        p.z = point_tras(2);
                        p.line = p.line+N_SCANS;
                        msgout.points.push_back(p);
                        //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, p.offset_time);
                        point_count++;
                }
                for(size_t i = 0;i<msg148->points.size();i++){
                        livox_ros_driver2::CustomPoint p = msg148->points[i];
                        p.offset_time+= time_diff*1000000000;
                        msgout.points.push_back(p);
                        //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, p.offset_time);
                        point_count++;
                }
        }
        else{
                double time_diff = msg110->header.stamp.toSec() - msg148->header.stamp.toSec();
                for(size_t i = 0;i<msg148->points.size();i++){
                        msgout.points.push_back(msg148->points[i]);
                         //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, msg148->points[i].offset_time);
                        point_count++;
                }
                for(size_t i = 0;i<msg110->points.size();i++){
                        livox_ros_driver2::CustomPoint p = msg110->points[i];
                        p.offset_time+= time_diff*1000000000;//时间单位：纳秒
                        Vec4d point(p.x,p.y,p.z,1.0);
                        Vec4d point_tras = T_lidar110_lidar148 * point;
                        p.x = point_tras(0);
                        p.y = point_tras(1);
                        p.z = point_tras(2);
                        p.line = p.line+4;
                        msgout.points.push_back(p);
                        //存储时间戳用于给新的点云按照时间戳进行排序
                        time_indices.emplace_back(point_count, p.offset_time);
                        point_count++;
                }
        }
        //按照时间戳进行索引排序
        std::sort(time_indices.begin(), time_indices.end(),[](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second < b.second;
         });
        //设置按照时间戳排序后的点云
        livox_ros_driver2::CustomMsg sorted_msgout;
        sorted_msgout.points.resize(msgout.points.size());
        for(size_t i = 0; i < time_indices.size(); ++i){
                sorted_msgout.points[i] = msgout.points[time_indices[i].first];
        }
        //设置按照时间戳排序后的点云
        sorted_msgout.point_num = sorted_msgout.points.size();
        sorted_msgout.header.frame_id = "livox_frame_148";
        sorted_msgout.header.stamp = msg110->header.stamp < msg148->header.stamp ? msg110->header.stamp:msg148->header.stamp;
        //设置按照时间排序前的点云
        msgout.point_num = msgout.points.size();
        msgout.header.frame_id = "livox_frame_148";
        msgout.header.stamp = msg110->header.stamp < msg148->header.stamp ? msg110->header.stamp:msg148->header.stamp;
        //发布按照时间戳排序之后的点云
        // pub.publish(sorted_msgout);
        //将合并后的livox数据格式转换为pcl点云格式(参考livox_ros_driver2)
        sensor_msgs::PointCloud2 pcl_msgout;
        InitPointcloud2MsgHeader(pcl_msgout,"livox_frame_148");
        pcl_msgout.point_step = sizeof(LivoxPointXyzrtlt);
        pcl_msgout.width = sorted_msgout.point_num;
        // std::cout << "sorted_msgout.point_num: " << sorted_msgout.point_num << std::endl;
        pcl_msgout.row_step = pcl_msgout.point_step * pcl_msgout.width;
        pcl_msgout.is_bigendian=false;
        pcl_msgout.is_dense = true;
        std::vector<LivoxPointXyzrtlt> temp_points;
        for(size_t i = 0;i<sorted_msgout.points.size();i++){
                LivoxPointXyzrtlt temp_point;
                temp_point.x = sorted_msgout.points[i].x;
                temp_point.y = sorted_msgout.points[i].y;
                temp_point.z = sorted_msgout.points[i].z;
                temp_point.reflectivity = sorted_msgout.points[i].reflectivity;
                temp_point.tag = sorted_msgout.points[i].tag;
                temp_point.line = sorted_msgout.points[i].line;
                temp_point.timestamp = static_cast<double>(sorted_msgout.points [i].offset_time);
                temp_points.push_back (std::move (temp_point));
        }
        pcl_msgout.data.resize (sorted_msgout.point_num*sizeof(LivoxPointXyzrtlt));
        memcpy(pcl_msgout.data.data (), temp_points.data (), sorted_msgout.point_num*sizeof (LivoxPointXyzrtlt));
        pcl_msgout.header.stamp = msg110->header.stamp < msg148->header.stamp ? msg110->header.stamp:msg148->header.stamp;
        pub.publish(pcl_msgout);
}

int main(int argc, char** argv){
        ros::init(argc, argv, "lidar_fusion");
        ros::NodeHandle nh;
        //20241020版本
        // T_imu110_148<<-0.00907786,0.0142474,0.999857,0.113636,
        //                                 0.999954,0.00311245,0.0090344,0.0659437,
        //                                 -0.00298329,0.999894,-0.014275,-0.0354266,
        //                                 0,0,0,1;
         //20241024版本
        T_imu110_148<<-0.00355503,0.0196655,0.9998,0.105433,
                                        0.999876,-0.0152523,0.00385531,0.0388548,
                                        0.0153251,0.99969,-0.0196088,-0.0375546,
                                        0,0,0,1;
        // T_imu110_148<<1,0,0,0,
        //                                 0,1,0,0,
        //                                0,0,1,0,
        //                                 0,0,0,1;
        T_lidar2imu << 1, 0, 0, -0.011, 
                                0, 1, 0, -0.02329, 
                                0, 0, 1, 0.04412, 
                                0, 0, 0, 1;
        T_imu110_148_inv = T_imu110_148.inverse();
        T_lidar2imu_inv = T_lidar2imu.inverse();
        T_lidar110_lidar148=T_lidar2imu_inv*T_imu110_148*T_lidar2imu;
        if(argc != 2){
                std::cout << "please input the point type (livox or pcl2)" << std::endl;
                return 0;
        }
        else if(argc == 2 && strcmp(argv[1], "livox") == 0){
                //合并两个livox点云
                std::cout << "start to merge livox point cloud" << std::endl;
                pub= nh.advertise<livox_ros_driver2::CustomMsg>("/lidar_fusion_livox", 10);
                message_filters::Subscriber<livox_ros_driver2::CustomMsg> sub110(nh, "/livox/lidar_192_168_1_110", 10);
                message_filters::Subscriber<livox_ros_driver2::CustomMsg> sub148(nh, "/livox/lidar_192_168_1_148", 10);
                typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg,livox_ros_driver2::CustomMsg> syncPolicy; //近似同步策略
                message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub110, sub148);// 同步,允许最大时间差异为10毫秒
                sync.registerCallback(boost::bind(&callback, _1, _2));
                ros::spin();
        }
        else if(argc == 2 && strcmp(argv[1], "pcl2") == 0){
                //版本一：直接合并两个PCL2点云
                // std::cout << "start to merge pcl point cloud" << std::endl;
                // pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_fusion_pcl", 10);
                // message_filters::Subscriber<sensor_msgs::PointCloud2> sub110(nh, "/livox/lidar_192_168_1_110", 10);
                // message_filters::Subscriber<sensor_msgs::PointCloud2> sub148(nh, "/livox/lidar_192_168_1_148", 10);
                // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> syncPolicy; 
                // message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub110, sub148);// 同步,允许最大时间差异为10毫秒
                // sync.registerCallback(boost::bind(&callback_pcl2, _1, _2));
                // ros::spin();
                //版本二：合并两个livox点云并转化为pcl2发布出去
                std::cout << "start to merge livox point cloud and convert to pcl2" << std::endl;
                pub= nh.advertise<sensor_msgs::PointCloud2>("/lidar_fusion_pcl", 10);
                message_filters::Subscriber<livox_ros_driver2::CustomMsg> sub110(nh, "/livox/lidar_192_168_1_110", 10);
                message_filters::Subscriber<livox_ros_driver2::CustomMsg> sub148(nh, "/livox/lidar_192_168_1_148", 10);
                typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg,livox_ros_driver2::CustomMsg> syncPolicy; //近似同步策略
                message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub110, sub148);// 同步,允许最大时间差异为10毫秒
                sync.registerCallback(boost::bind(&callback_livox2pcl, _1, _2));
                ros::spin();
        }
        return 0;
}
  