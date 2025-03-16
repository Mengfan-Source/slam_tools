/**
 * @file pcd_align.cpp
 * @brief 11所项目NDT配准相邻两帧相机输出的点云，配准前滤波操作
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2025-0313
 * @copyright Copyright (C) 2025 中国电科科技集团公司第二十一研究所
 */
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include<pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ndt_3d.h"
#include "point_cloud_utils.h"
#include "sys_utils.h"
#include <algorithm> 
#include <execution> 
#include <cmath>
#include <mutex>
DEFINE_string(source, "/home/xmf/xmf_bags/cetc11/fromcetc11/2025-03-10-14-57-34/1741590024.629749298.pcd", "第1个点云路径");
DEFINE_string(target, "/home/xmf/xmf_bags/cetc11/fromcetc11/2025-03-10-14-57-34/1741590024.675841808.pcd", "第2个点云路径");//基准点云路径
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0/M_PI ;
// DEFINE_string(ground_truth_file, "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid360/kneeling_lady_pose.txt", "真值Pose");
/*
程序运行过程中可视化点云，按照强度
输入参数：要可视化点云的指针
*/
void visualize_point(const xmf::CloudPtr &cloud_in, const std:: string &name){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_intensity (cloud_in, "intensity");//创建点云颜色处理器,设置强度渲染
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud_in, color_intensity,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,name);
    viewer->initCameraParameters ();
    viewer->addText(name , 10, 10, 20, 1, 1, 1, name);
    while (!viewer->wasStopped ()) {
        viewer->spinOnce ();
     }
}
int main(int argc, char** argv) {
        FLAGS_log_dir = "./../log";
        FLAGS_minloglevel = google::INFO;
        google::InitGoogleLogging(argv[0]);
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;
        google::ParseCommandLineFlags(&argc, &argv, true);

        xmf::CloudPtr source(new xmf::PointCloudType), target(new xmf::PointCloudType);
        xmf::CloudPtr source_filtered(new xmf::PointCloudType), target_filtered(new xmf::PointCloudType);
        pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
        pcl::io::loadPCDFile(fLS::FLAGS_target, *target);
        //对source和target进行体素滤波
        pcl::VoxelGrid<xmf::PointType> sor_source;  
        sor_source.setInputCloud(source);  
        sor_source.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素的大小，这里是1cm  
        sor_source.filter(*source_filtered);  

        pcl::VoxelGrid<xmf::PointType> sor_target;  
        sor_target.setInputCloud(target);  
        sor_target.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素的大小，这里是1cm  
        sor_target.filter(*target_filtered);  

        //统计滤波，剔除离群点
        xmf::CloudPtr source_CloudFilterOut(new xmf::PointCloudType), target_CloudFilterOut(new xmf::PointCloudType);
        pcl::StatisticalOutlierRemoval<xmf::PointType> remove_discrete_points_source; //参数：统计滤波最近邻点个数，标准差阈值
        remove_discrete_points_source.setInputCloud (source_filtered);
        remove_discrete_points_source.setMeanK (20);//设置在进行统计滤波时考虑的临近点个数
        remove_discrete_points_source.setStddevMulThresh (1.0);//如果参数设置过小会导致边界点部分被剔除
        remove_discrete_points_source.filter (*source_CloudFilterOut);        
        if(1)
        {
                         //程序运行过程中临时显示点云
                        pcl::visualization::PCLVisualizer::Ptr viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_z1 (source_CloudFilterOut, "z");
                        viewer1->setBackgroundColor (0, 0, 0);
                        viewer1->addPointCloud<pcl::PointXYZI> (source_CloudFilterOut, color_z1,"target_points");
                        viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"target_points");
                        viewer1->initCameraParameters ();
                        while (!viewer1->wasStopped ()) {
                                viewer1->spinOnce ();
                        }
        }

        pcl::StatisticalOutlierRemoval<xmf::PointType> remove_discrete_points_target; //参数：统计滤波最近邻点个数，标准差阈值
        remove_discrete_points_target.setInputCloud (target_filtered);
        remove_discrete_points_target.setMeanK (20);//设置在进行统计滤波时考虑的临近点个数
        remove_discrete_points_target.setStddevMulThresh (1.0);//如果参数设置过小会导致边界点部分被剔除
        remove_discrete_points_target.filter (*target_CloudFilterOut);        
        
        //滤除50米以外且在固定视角范围内的点
        xmf::CloudPtr source_valid(new xmf::PointCloudType), target_valid(new xmf::PointCloudType);
        std::vector<xmf::PointType> source_temp,target_temp;
        std::mutex filtered_points_mutex;
        std::for_each(std::execution::par_unseq,target_CloudFilterOut->points.begin(),target_CloudFilterOut->points.end(),[&](xmf::PointType point){
                float angleZ = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
                float angleY = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
                if(sqrt(point.x*point.x+point.y*point.y+point.z*point.z)<20 && std::abs(angleY)<40){
                        // target_valid->points.push_back(point);//直接操作点云多线程会报错
                        target_temp.push_back(point);
                        std::lock_guard<std::mutex> guard(filtered_points_mutex);
                }
        });
        // std::for_each(std::execution::par_unseq,source_CloudFilterOut->points.begin(),source_CloudFilterOut->points.end(),[&](xmf::PointType point){
        //         float angleZ = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        //         float angleY = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
        //         if(sqrt(point.x*point.x+point.y*point.y+point.z*point.z)<20 && std::abs(angleY)<40){
        //                 // source_valid->points.push_back(point);//直接并行操作点云多线程会报错
        //                 source_temp.push_back(point);
        //                 std::lock_guard<std::mutex> guard(filtered_points_mutex);
        //         }
        // });
        source_valid->points.assign(source_temp.begin(),source_temp.end());
        source_valid->width =  source_temp.size();
        source_valid->height = 1;
        source_valid->is_dense = false;  

        // target_valid->points.assign(target_temp.begin(),target_temp.end());
        // target_valid->width =  target_temp.size();
        // target_valid->height = 1;
        // target_valid->is_dense = false;  


/*

        //开始配准：使用自己写的ndt算法
        bool success;
        xmf::Ndt3d::Options options;
        options.voxel_size_ = 0.5;
        options.remove_centroid_ = false;//是否使用两个点云的质心来设置初始位姿
        options.nearby_type_ = xmf::Ndt3d::NearbyType::CENTER;
        xmf::Ndt3d ndt(options);
        ndt.SetSource(source_valid);
        ndt.SetTarget(target_valid);
        
        double angX = 0,angY = 0,angZ = 0;
        SE3 pose(SO3::rotZ(angZ * DEG2RAD)*SO3::rotY (angY * DEG2RAD)*SO3::rotX (angX * DEG2RAD), Vec3d (0,0, 0) );
        xmf::GridSearchResult gr;
        gr.msPose = pose;
        if(ndt.AlignNdt(gr)){
                LOG(INFO) << "ndt align success, pose: " << gr.msResultPose.so3().unit_quaternion().coeffs().transpose() << ", "
                        << gr.msResultPose.translation().transpose();
                LOG(INFO)<<"matrix \n"<< gr.msResultPose.matrix();
        }
        else {
                LOG(ERROR) << "align failed.";
        }

        LOG(INFO)<<"matrix_init \n"<< gr.msPose.matrix();
        LOG(INFO)<<"matrix \n"<< gr.msResultPose.matrix();
        LOG(INFO)<<"Euler_init : "<< gr.msPose.angleX()*RAD2DEG<<", "<< gr.msPose.angleY()*RAD2DEG << ", "<< gr.msPose.angleZ()*RAD2DEG;
        LOG(INFO)<<"Euler : "<< gr.msResultPose.angleX()*RAD2DEG<<", "<< gr.msResultPose.angleY()*RAD2DEG << ", "<< gr.msResultPose.angleZ()*RAD2DEG;
        LOG(INFO)<<"align error \n"<< gr.dx;
        //保存转换之后的点云(体素滤波后)
        xmf::CloudPtr source_trans_filtered(new xmf::PointCloudType);
        pcl::transformPointCloud(*source_filtered, *source_trans_filtered, gr.msResultPose.matrix().cast<float>());
        xmf::SaveCloudToFile("/home/xmf/xmf_slam/slam_tools/data/result/source_trans_filtered.pcd", *source_trans_filtered);
        xmf::SaveCloudToFile("/home/xmf/xmf_slam/slam_tools/data/result/target_filtered.pcd", *target_filtered);
        xmf::CloudPtr source_trans_valid(new xmf::PointCloudType);
        pcl::transformPointCloud(*source_valid, *source_trans_valid, gr.msResultPose.matrix().cast<float>());
        xmf::SaveCloudToFile("/home/xmf/xmf_slam/slam_tools/data/result/source_valid_trans.pcd", *source_trans_valid);
        xmf::SaveCloudToFile("/home/xmf/xmf_slam/slam_tools/data/result/target_valid.pcd", *target_valid);
        */
        return 0;
}