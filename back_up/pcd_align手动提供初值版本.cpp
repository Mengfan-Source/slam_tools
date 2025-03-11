/**
 * @file pcd_align.cpp
 * @brief 通过自己写的ndt方法配准两个pcd点云，可用于两个激光雷达的标定
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2024-1023
 * @copyright Copyright (C) 2024 中国电科科技集团公司第二十一研究所
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
DEFINE_string(source, "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/scans110.pcd", "第1个点云路径");
DEFINE_string(target, "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/scans148.pcd", "第2个点云路径");//基准点云路径
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0/M_PI ;
// DEFINE_string(ground_truth_file, "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid360/kneeling_lady_pose.txt", "真值Pose");
int main(int argc, char** argv) {
        FLAGS_log_dir = "/home/cetc21/xmf/my_slam_ws/slam_test/build/log";
        FLAGS_minloglevel = google::ERROR;
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
        //平面分割
        pcl::SACSegmentation<pcl::PointXYZI> seg_source;
        pcl::PointIndices::Ptr inliers_source (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_source (new pcl::ModelCoefficients);
        seg_source.setOptimizeCoefficients (true);
        seg_source.setModelType (pcl::SACMODEL_PLANE);
        seg_source.setMethodType (pcl::SAC_RANSAC);
        seg_source.setMaxIterations (3000);
        seg_source.setDistanceThreshold (0.1);//设置点到模型的最大允许距离，用于决定一个点是否属于该模型。
        seg_source.setInputCloud (source_CloudFilterOut);
        seg_source.segment (*inliers_source, *coefficients_source);

        pcl::SACSegmentation<pcl::PointXYZI> seg_target;
        pcl::PointIndices::Ptr inliers_target (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_target (new pcl::ModelCoefficients);
        seg_target.setOptimizeCoefficients (true);
        seg_target.setModelType (pcl::SACMODEL_PLANE);
        seg_target.setMethodType (pcl::SAC_RANSAC);
        seg_target.setMaxIterations (3000);
        seg_target.setDistanceThreshold (0.1);//设置点到模型的最大允许距离，用于决定一个点是否属于该模型。
        seg_target.setInputCloud (target_CloudFilterOut);
        seg_target.segment (*inliers_target, *coefficients_target);
        //将分离出的平面点去除
        xmf::CloudPtr source_NoGroundPoint(new xmf::PointCloudType), target_NoGroundPoint(new xmf::PointCloudType);
        pcl::ExtractIndices<pcl::PointXYZI> extract_source;
        extract_source.setInputCloud (source_CloudFilterOut);
        extract_source.setIndices (inliers_source);
        extract_source.setNegative (true);
        extract_source.filter (*source_NoGroundPoint); 

        pcl::ExtractIndices<pcl::PointXYZI> extract_target;
        extract_target.setInputCloud (target_CloudFilterOut);
        extract_target.setIndices (inliers_target);
        extract_target.setNegative (true);
        extract_target.filter (*target_NoGroundPoint); 
        

        //使用pcl库的ndt算法
         /*
         pcl::NormalDistributionsTransform<xmf::PointType, xmf::PointType> ndt;
        ndt.setTransformationEpsilon (0.005);//设置配准终止条件的阈值，0.005
        ndt.setStepSize (0.01);//设置每次优化迭代的步长0.1
        ndt.setMaximumIterations (20);//设置最大迭代次数，防止算法陷入死循环10
        ndt.setInputSource (source_filtered);//设置源点云，即需要配准的点云 这里是当前帧点云经过0.1m分辨率体素滤波后的点云
        ndt.setInputTarget (target_filtered);
        ndt.setResolution (0.5 );
        //设置配准初值
        double grid_ang_rangeX = 91,grid_ang_rangeY = 1,grid_ang_rangeZ = 91, grid_ang_step = 20; 
        std::vector<xmf::GridSearchResult> search_poses;
        for(double angX = 90; angX < grid_ang_rangeX; angX += grid_ang_step)
        {
                for(double angY = 0; angY < grid_ang_rangeY; angY += grid_ang_step)
                {
                        for(double angZ = 90; angZ < grid_ang_rangeZ; angZ += grid_ang_step)
                        {
                                SE3 pose(SO3::rotZ(angZ * DEG2RAD)*SO3::rotY (angY * DEG2RAD)*SO3::rotX (angX * DEG2RAD), Vec3d (0,0, 0) );
                                xmf::GridSearchResult gr;
                                gr.msPose = pose;
                                search_poses.emplace_back(gr);
                        }
                }
        }
        //执行配准
        xmf::CloudPtr output(new xmf::PointCloudType);
        for(xmf::GridSearchResult & gr:search_poses){
               ndt.align(*output, gr.msPose.matrix ().cast<float> ());
                static int count = 0;
                count++;
                LOG(ERROR)<<"count:  "<< count <<" in total:"<<search_poses.size();
                gr.dx = ndt.getTransformationProbability ();
                gr.msResultPose = xmf::Mat4ToSE3 (ndt.getFinalTransformation ());
        }
        auto max_ele = std::max_element (search_poses.begin (), search_poses.end (),
                                        [](const auto& g1, const auto& g2) { return g1.dx < g2.dx; });
        LOG(ERROR)<<" max_ele_matrix_init \n"<<  max_ele->msPose.matrix();
        LOG(ERROR)<<" max_ele : "<<  max_ele->dx;
        LOG(ERROR)<<" max_ele_matrix \n"<<  max_ele->msResultPose.matrix();
        LOG(ERROR)<<" max_ele_Euler_init : "<<  max_ele->msPose.angleX()*RAD2DEG<<", "<<  max_ele->msPose.angleY()*RAD2DEG << ", "<<  max_ele->msPose.angleZ()*RAD2DEG;
        LOG(ERROR)<<" max_ele_Euler : "<<  max_ele->msResultPose.angleX()*RAD2DEG<<", "<<  max_ele->msResultPose.angleY()*RAD2DEG << ", "<<  max_ele->msResultPose.angleZ()*RAD2DEG;
        //保存转换之后的点云
        xmf::CloudPtr source_trans_filtered(new xmf::PointCloudType);
        pcl::transformPointCloud(*source_filtered, *source_trans_filtered,  max_ele->msResultPose.matrix().cast<float>());
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601017/result/source_trans_filtered.pcd", *source_trans_filtered);
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601017/result/target_filtered.pcd", *target_filtered);
        */

        //使用自己写的ndt算法
        bool success;
        xmf::Ndt3d::Options options;
        options.voxel_size_ = 0.5;
        options.remove_centroid_ = false;//是否使用两个点云的质心来设置初始位姿
        options.nearby_type_ = xmf::Ndt3d::NearbyType::CENTER;
        xmf::Ndt3d ndt(options);
        ndt.SetSource(source_filtered);
        ndt.SetTarget(target_filtered);
        // ndt.SetSource(source_NoGroundPoint);
        // ndt.SetTarget(target_NoGroundPoint);
        
        //设置配准初值
        double grid_ang_rangeX = 91,grid_ang_rangeY = 1,grid_ang_rangeZ = 91, grid_ang_step = 20; 
        std::vector<xmf::GridSearchResult> search_poses;
        for(double angX = 90; angX < grid_ang_rangeX; angX += grid_ang_step)
        {
                for(double angY = 0; angY < grid_ang_rangeY; angY += grid_ang_step)
                {
                        for(double angZ = 90; angZ < grid_ang_rangeZ; angZ += grid_ang_step)
                        {
                                SE3 pose(SO3::rotZ(angZ * DEG2RAD)*SO3::rotY (angY * DEG2RAD)*SO3::rotX (angX * DEG2RAD), Vec3d (0,0, 0) );
                                xmf::GridSearchResult gr;
                                gr.msPose = pose;
                                search_poses.emplace_back(gr);
                        }
                }
        }
        //执行配准
        for(xmf::GridSearchResult & gr:search_poses){
                if(ndt.AlignNdt(gr)){
                        LOG(INFO) << "ndt align success, pose: " << gr.msResultPose.so3().unit_quaternion().coeffs().transpose() << ", "
                        << gr.msResultPose.translation().transpose();
                        LOG(INFO)<<"matrix \n"<< gr.msResultPose.matrix();
                 }
                 else {
                        LOG(ERROR) << "align failed.";
                }
                static int count = 0;
                count++;
                LOG(ERROR)<<"count:  "<< count <<" in total:"<<search_poses.size();
        }
        auto min_dx = std::min_element(search_poses.begin(), search_poses.end(), [](const xmf::GridSearchResult& a, const xmf::GridSearchResult& b) {
                return a.dx < b.dx;
        });
        LOG(ERROR)<<"min_matrix_init \n"<< min_dx->msPose.matrix();
        LOG(ERROR)<<"min_dx : "<< min_dx->dx;
        LOG(ERROR)<<"min_matrix \n"<< min_dx->msResultPose.matrix();
        LOG(ERROR)<<"min_Euler_init : "<< min_dx->msPose.angleX()*RAD2DEG<<", "<< min_dx->msPose.angleY()*RAD2DEG << ", "<< min_dx->msPose.angleZ()*RAD2DEG;
        LOG(ERROR)<<"min_Euler : "<< min_dx->msResultPose.angleX()*RAD2DEG<<", "<< min_dx->msResultPose.angleY()*RAD2DEG << ", "<< min_dx->msResultPose.angleZ()*RAD2DEG;
        //保存转换之后的点云(体素滤波后)
        xmf::CloudPtr source_trans_filtered(new xmf::PointCloudType);
        pcl::transformPointCloud(*source_filtered, *source_trans_filtered, min_dx->msResultPose.matrix().cast<float>());
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/result/source_trans_filtered.pcd", *source_trans_filtered);
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/result/target_filtered.pcd", *target_filtered);
         //保存转换之后的点云(去除地面点后)
        xmf::CloudPtr source_trans_noGround(new xmf::PointCloudType);
        pcl::transformPointCloud(*source_NoGroundPoint, *source_trans_noGround, min_dx->msResultPose.matrix().cast<float>());
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/result/source_trans_noGround.pcd", *source_trans_noGround);
         xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/result/source_NoGroundPoint.pcd", *source_NoGroundPoint);
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/result/target_NoGroundPoint.pcd", *target_NoGroundPoint);


        /*SE3 pose(SO3::rotZ(90 * DEG2RAD)*SO3::rotX (90 * DEG2RAD), Vec3d (0,0, 0) );
        //进行单次NDT配准
        success = ndt.AlignNdt(pose);
        if (success) {
        LOG(INFO) << "ndt align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", "
                        << pose.translation().transpose();
        LOG(INFO)<<"matrix \n"<< pose.matrix();
        xmf::CloudPtr source_trans_filtered(new xmf::PointCloudType);
        pcl::transformPointCloud(*source_filtered, *source_trans_filtered, pose.matrix().cast<float>());
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601017/result/source_trans_filtered.pcd", *source_trans_filtered);
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601017/result/target_filtered.pcd", *target_filtered);
        } 
        else {
                LOG(ERROR) << "align failed.";
        }*/
        return 0;
}