/**
 * @file pcd_align.cpp
 * @brief 配准两个pcd点云，可用于两个激光雷达的标定
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2024-1023
 * @copyright Copyright (C) 2024 中国电科科技集团公司第二十一研究所
 */
#include <string>
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
#include <yaml-cpp/yaml.h>

DEFINE_string(config_yaml, "./../src/config/param_pcdalign.yaml", "配置文件");
// DEFINE_string(source, "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/scans110.pcd", "第1个点云路径");
// DEFINE_string(target, "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/scans148.pcd", "第2个点云路径");//基准点云路径
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0/M_PI ;
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
//如果矩阵接近旋转矩阵，但可能由于数值误差而略有偏差，使用 Rodrigues 公式 来修正。这种方法需要计算矩阵的对数（即旋转轴和角度），然后重新构造旋转矩阵
Eigen::Matrix3d My_ToRotationMatrix(const Eigen::Matrix3d& A) {
    // 使用 Rodrigues 公式
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis;
    double angle;
    Eigen::AngleAxisd aa;
    aa.fromRotationMatrix(A);
    axis = aa.axis();
    angle = aa.angle();
    // 重新构造旋转矩阵
    R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

    return R;
}
int main(int argc, char** argv) {
        FLAGS_log_dir = "./../log";
        FLAGS_minloglevel = google::ERROR; //ERROR,WARNING,INFO,DEBUG
        google::InitGoogleLogging(argv[0]);
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;
        google::ParseCommandLineFlags(&argc, &argv, true);
        //加载参数
        auto yaml = YAML::LoadFile(fLS::FLAGS_config_yaml);
        std::string source_path,target_path;
        bool is_show_temp = false; //过程中是否显示临时点云
        bool is_save_pcd = true; //是否保存点云
        //用于保存点云的路径
        std::string source_trans_filtered_path,target_filtered_path,source_trans_noGround_path,source_NoGroundPoint_path,target_NoGroundPoint_path;
        //滤波相关参数
        double voxelgrid_leaf_size,outlier_removal_stddev_mul_thresh,sac_distance_threshold;
        int outlier_removal_mean_k,sac_max_iterations;
        try {
                    source_path = yaml["source_path"].as<std::string>();
                    target_path = yaml["target_path"].as<std::string>();
                    is_show_temp = yaml["is_show_temp"].as<bool>();
                    //保存点云相关参数
                    is_save_pcd = yaml["save_pcd_result_params"]["is_save_pcd"].as<bool>();
                    source_trans_filtered_path = yaml["save_pcd_result_params"]["source_trans_filtered_path"].as<std::string>();
                    target_filtered_path = yaml["save_pcd_result_params"]["target_filtered_path"].as<std::string>();
                    source_trans_noGround_path = yaml["save_pcd_result_params"]["source_trans_noGround_path"].as<std::string>();
                    source_NoGroundPoint_path = yaml["save_pcd_result_params"]["source_NoGroundPoint_path"].as<std::string>();
                    target_NoGroundPoint_path = yaml["save_pcd_result_params"]["target_NoGroundPoint_path"].as<std::string>();
                    //滤波相关参数
                    voxelgrid_leaf_size = yaml["filter_params"]["voxelgrid_leaf_size"].as<double>();
                    outlier_removal_stddev_mul_thresh = yaml["filter_params"]["outlier_removal_stddev_mul_thresh"].as<double>();
                    outlier_removal_mean_k = yaml["filter_params"]["outlier_removal_mean_k"].as<int>();
                    sac_distance_threshold = yaml["filter_params"]["sac_distance_threshold"].as<double>();
                    sac_max_iterations = yaml["filter_params"]["sac_max_iterations"].as<int>();

                } catch (...) {
                    LOG(ERROR) << "failed to parse yaml";
                }
        xmf::CloudPtr source(new xmf::PointCloudType), target(new xmf::PointCloudType);
        xmf::CloudPtr source_filtered(new xmf::PointCloudType), target_filtered(new xmf::PointCloudType);
        // pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
        // pcl::io::loadPCDFile(fLS::FLAGS_target, *target);
        pcl::io::loadPCDFile(source_path, *source);
        pcl::io::loadPCDFile(target_path, *target);
    
        //对source和target进行体素滤波
        pcl::VoxelGrid<xmf::PointType> sor_source;  
        sor_source.setInputCloud(source);  
        sor_source.setLeafSize(voxelgrid_leaf_size, voxelgrid_leaf_size,voxelgrid_leaf_size); // 设置体素的大小，这里是1cm  
        sor_source.filter(*source_filtered);  

        pcl::VoxelGrid<xmf::PointType> sor_target;  
        sor_target.setInputCloud(target);  
        sor_target.setLeafSize(voxelgrid_leaf_size, voxelgrid_leaf_size, voxelgrid_leaf_size); // 设置体素的大小，这里是1cm  
        sor_target.filter(*target_filtered);  

        //统计滤波，剔除离群点
        xmf::CloudPtr source_CloudFilterOut(new xmf::PointCloudType), target_CloudFilterOut(new xmf::PointCloudType);
        pcl::StatisticalOutlierRemoval<xmf::PointType> remove_discrete_points_source; //参数：统计滤波最近邻点个数，标准差阈值
        remove_discrete_points_source.setInputCloud (source_filtered);
        remove_discrete_points_source.setMeanK (outlier_removal_mean_k);//设置在进行统计滤波时考虑的临近点个数
        remove_discrete_points_source.setStddevMulThresh (outlier_removal_stddev_mul_thresh);//如果参数设置过小会导致边界点部分被剔除
        remove_discrete_points_source.filter (*source_CloudFilterOut);        
        if(is_show_temp){
            visualize_point(source_CloudFilterOut,"remove_discrete_points_source");
        }
        pcl::StatisticalOutlierRemoval<xmf::PointType> remove_discrete_points_target; //参数：统计滤波最近邻点个数，标准差阈值
        remove_discrete_points_target.setInputCloud (target_filtered);
        remove_discrete_points_target.setMeanK (outlier_removal_mean_k);//设置在进行统计滤波时考虑的临近点个数
        remove_discrete_points_target.setStddevMulThresh (outlier_removal_stddev_mul_thresh);//如果参数设置过小会导致边界点部分被剔除
        remove_discrete_points_target.filter (*target_CloudFilterOut);        

        //平面分割
        pcl::SACSegmentation<pcl::PointXYZI> seg_source;
        pcl::PointIndices::Ptr inliers_source (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_source (new pcl::ModelCoefficients);
        seg_source.setOptimizeCoefficients (true);
        seg_source.setModelType (pcl::SACMODEL_PLANE);
        seg_source.setMethodType (pcl::SAC_RANSAC);
        seg_source.setMaxIterations (sac_max_iterations);
        seg_source.setDistanceThreshold (sac_distance_threshold);//设置点到模型的最大允许距离，用于决定一个点是否属于该模型。
        seg_source.setInputCloud (source_CloudFilterOut);
        seg_source.segment (*inliers_source, *coefficients_source);

        pcl::SACSegmentation<pcl::PointXYZI> seg_target;
        pcl::PointIndices::Ptr inliers_target (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_target (new pcl::ModelCoefficients);
        seg_target.setOptimizeCoefficients (true);
        seg_target.setModelType (pcl::SACMODEL_PLANE);
        seg_target.setMethodType (pcl::SAC_RANSAC);
        seg_target.setMaxIterations (sac_max_iterations);
        seg_target.setDistanceThreshold (sac_distance_threshold);//设置点到模型的最大允许距离，用于决定一个点是否属于该模型。
        seg_target.setInputCloud (target_CloudFilterOut);
        seg_target.segment (*inliers_target, *coefficients_target);
        //提取非平面点云
        xmf::CloudPtr source_NoGroundPoint(new xmf::PointCloudType), target_NoGroundPoint(new xmf::PointCloudType);
        pcl::ExtractIndices<pcl::PointXYZI> extract_source;
        extract_source.setInputCloud (source_CloudFilterOut);
        extract_source.setIndices (inliers_source);
        extract_source.setNegative (true);//提取非内点，非内点是平面点
        extract_source.filter (*source_NoGroundPoint); 
        pcl::ExtractIndices<pcl::PointXYZI> extract_target;
        extract_target.setInputCloud (target_CloudFilterOut);
        extract_target.setIndices (inliers_target);
        extract_target.setNegative (true);
        extract_target.filter (*target_NoGroundPoint); 
        //显示去除地面点后的点云
        if(is_show_temp){
            visualize_point(source_NoGroundPoint,"source_NoGroundPoint");
        }

        //提取出平面点云
        xmf::CloudPtr source_GroundPoint(new xmf::PointCloudType), target_GroundPoint(new xmf::PointCloudType);
        pcl::ExtractIndices<pcl::PointXYZI> groud_source;
        groud_source.setInputCloud (source_CloudFilterOut);
        groud_source.setIndices (inliers_source);
        groud_source.setNegative (false);//提取内点，内点是平面点
        groud_source.filter (*source_GroundPoint);
        pcl::ExtractIndices<pcl::PointXYZI> groud_target;
        groud_target.setInputCloud (target_CloudFilterOut);
        groud_target.setIndices (inliers_target);
        groud_target.setNegative (false);
        groud_target.filter (*target_GroundPoint);
        //显示提取出的平面点云
        if(is_show_temp){
            //显示提取出的平面点云（source）
            visualize_point(source_GroundPoint,"source_GroundPoint");
            //显示提取出的平面点云（target）
             visualize_point(target_GroundPoint,"target_GroundPoint");
        }
        //根据平面点的法向量和协方差矩阵最大的一个特征值对应的特征向量来，
        Vec3d n_source(coefficients_source->values[0],coefficients_source->values[1],coefficients_source->values[2]);//平面法向量
        Vec3d n_target(coefficients_target->values[0],coefficients_target->values[1],coefficients_target->values[2]);//平面法向量
        LOG(WARNING)<<"n_source: \n"<<n_source<<endl;
        LOG(WARNING)<<"n_target: \n"<<n_target<<endl;

        //规范法向量的方向：所计算出的法向量与非地面点云的质心方向夹角为锐角
        Vec4d center_S,center_T ;
        pcl::compute3DCentroid(*source_NoGroundPoint, center_S);
        pcl::compute3DCentroid(*target_NoGroundPoint, center_T);
        Vec3d center_NoGroudSource =center_S.head<3>();//计算非地面点云的质心
        Vec3d center_NoGroudTarget = center_T.head<3>();
        LOG(WARNING)<<"center_NoGroudSource: \n"<<center_S;
        LOG(WARNING)<<"center_NoGroudTarget: \n"<<center_T;
        double cos_theta_s = n_source.dot(center_NoGroudSource)/(n_source.norm()*center_NoGroudSource.norm());
        double cos_theta_t = n_target.dot(center_NoGroudTarget)/(n_target.norm()*center_NoGroudTarget.norm());
        double angle_rad_s = std::acos(cos_theta_s)*RAD2DEG;//计算平面法向量与质心方向的夹角，单位角度
        double angle_rad_t = std::acos(cos_theta_t)*RAD2DEG;
        LOG(WARNING)<<"angle_rad_s: "<<angle_rad_s<<endl;
        LOG(WARNING)<<"angle_rad_t: "<<angle_rad_t<<endl;
        if ((abs(angle_rad_s) - 90)*(abs(angle_rad_t) - 90)<=0) {
            n_source = -n_source;
        }
    
        Vec4d cen_GroundSource,cen_GroundTarget;
        Mat3d cov_GroundSource,cov_GroundTarget;
        pcl::computeMeanAndCovarianceMatrix(*source_GroundPoint, cov_GroundSource, cen_GroundSource);//计算平面点的均值和协方差矩阵
        pcl::computeMeanAndCovarianceMatrix(*target_GroundPoint, cov_GroundTarget, cen_GroundTarget);

        //计算协方差矩阵的特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_GroundSource(cov_GroundSource);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_GroundTarget(cov_GroundTarget);
        std::vector<xmf::GridSearchResult> search_poses;
        if (eigen_solver_GroundSource.info() == Eigen::Success && eigen_solver_GroundTarget.info() == Eigen::Success){
            //获取最大特征值对应的特征向量（这里不需要使用绝对值，因为协方差矩阵一定是半正定矩阵，半正定矩阵的特征值一定大于等于0）
            Vec3d max_VectorGroundSource = eigen_solver_GroundSource.eigenvectors().col(2);
            Vec3d max_VectorGroundTarget = eigen_solver_GroundTarget.eigenvectors().col(2);
            //输出矩阵的特征值
            LOG(WARNING)<<" eigenvalues_GroundSource: \n"<<eigen_solver_GroundSource.eigenvalues();
            LOG(WARNING)<<" eigenvalues_GroundTarget: \n"<<eigen_solver_GroundTarget.eigenvalues();

            //输出最大特征值对应的特征向量
            LOG(WARNING)<<" max_VectorGroundSource: \n"<<max_VectorGroundSource;
            LOG(WARNING)<<" max_VectorGroundTarget: \n"<<max_VectorGroundTarget;

            //最大特征值对应的特征向量或许会指向两个相反的方向，因此这里保存两个初值用于配准。
            //创建旋转矩阵
            for(int i = 0;i<2;i++){
                Vec3d source_v1 = max_VectorGroundSource;//平面点协方差矩阵的最大特征值对应的特征向量为x轴方向
                Vec3d source_v2 = source_v1.cross(n_source);
                Vec3d source_v3 = n_source;
                Mat3d rot_source = (Mat3d() << source_v1, source_v2, source_v3).finished();//列向量作为基向量

                Vec3d target_v1 = max_VectorGroundTarget;
                if(i%2){
                    target_v1 = -max_VectorGroundTarget;
                }
                Vec3d target_v2 = target_v1.cross(n_target);
                Vec3d target_v3 = n_target;
                Mat3d rot_target = (Mat3d() << target_v1, target_v2, target_v3).finished();
                //计算两个旋转矩阵的相对位置关系：Target = R * Source，得到的旋转矩阵可以将source坐标系下的点变换到target坐标系下
                Mat3d R = rot_target * rot_source.transpose();
                //所计算出的矩阵接近旋转矩阵，使用 Rodrigues 公式 来修正
                Mat3d R_init  = My_ToRotationMatrix(R);
                //输出旋转矩阵
                LOG(WARNING)<<" R: \n"<<R;
                LOG(WARNING)<<" R_init: \n"<<R_init;
                Eigen::Vector3d eulerAngle = R_init.eulerAngles(2, 1, 0);// 指定旋转顺序为ZYX 单位为弧度
                LOG(WARNING)<<" euler: \n"<<eulerAngle*RAD2DEG;
                //使用非地面点的质心计算平移量
                Vec3d t_init = center_NoGroudTarget - R_init * center_NoGroudSource;
                //设置配准初值
                SE3 pose(R_init, t_init);
                xmf::GridSearchResult gr;
                gr.msPose = pose;
                search_poses.emplace_back(gr);
            }

        }
        else{
           LOG(ERROR) << "Error: Eigen decomposition failed!" ;
            return 0;
        }
        //开始配准 使用自己写的ndt算法
        bool success;
        xmf::Ndt3d::Options options;
        options.voxel_size_ = 0.5;
        options.remove_centroid_ = false;//是否使用两个点云的质心来设置初始位姿
        options.nearby_type_ = xmf::Ndt3d::NearbyType::CENTER;
        xmf::Ndt3d ndt(options);
        ndt.SetSource(source_filtered);
        ndt.SetTarget(target_filtered);

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
                LOG(ERROR)<<"gr.dx:  " << gr.dx;
        }
        auto min_dx = std::min_element(search_poses.begin(), search_poses.end(), [](const xmf::GridSearchResult& a, const xmf::GridSearchResult& b) {
                return a.dx < b.dx;
        });
        LOG(ERROR)<< std::fixed << std::setprecision(5)<<"min_matrix_init \n"<< min_dx->msPose.matrix();
        LOG(ERROR)<< std::fixed << std::setprecision(5)<<"min_dx : \n "<< min_dx->dx;
        LOG(ERROR)<< std::fixed << std::setprecision(5)<<"min_matrix \n"<< min_dx->msResultPose.matrix();
        // LOG(ERROR)<<"min_Euler_init : "<< min_dx->msPose.angleX()*RAD2DEG<<", "<< min_dx->msPose.angleY()*RAD2DEG << ", "<< min_dx->msPose.angleZ()*RAD2DEG;
        // LOG(ERROR)<<"min_Euler : "<< min_dx->msResultPose.angleX()*RAD2DEG<<", "<< min_dx->msResultPose.angleY()*RAD2DEG << ", "<< min_dx->msResultPose.angleZ()*RAD2DEG;
        LOG(ERROR)<< std::fixed << std::setprecision(5)<<"min_Euler_init :  \n"<< min_dx->msPose.rotationMatrix().eulerAngles(2,1,0)*RAD2DEG;
        LOG(ERROR)<< std::fixed << std::setprecision(5)<<"min_Euler :  \n"<< min_dx->msResultPose.rotationMatrix().eulerAngles(2,1,0)*RAD2DEG;
        if(is_save_pcd){
            //保存转换之后的点云(体素滤波后)
            xmf::CloudPtr source_trans_filtered(new xmf::PointCloudType);
            pcl::transformPointCloud(*source_filtered, *source_trans_filtered, min_dx->msResultPose.matrix().cast<float>());
            xmf::SaveCloudToFile(source_trans_filtered_path, *source_trans_filtered);
            xmf::SaveCloudToFile(target_filtered_path, *target_filtered);
            //保存转换之后的点云(去除地面点后)
            xmf::CloudPtr source_trans_noGround(new xmf::PointCloudType);
            pcl::transformPointCloud(*source_NoGroundPoint, *source_trans_noGround, min_dx->msResultPose.matrix().cast<float>());
            xmf::SaveCloudToFile(source_trans_noGround_path, *source_trans_noGround);
            xmf::SaveCloudToFile(source_NoGroundPoint_path, *source_NoGroundPoint);
            xmf::SaveCloudToFile(target_NoGroundPoint_path, *target_NoGroundPoint);
        }

        /* 方案一：根据平面点的法向量计算旋转矩阵(缺少信息)
        //计算两个平面的相对位置关系
        Vec3d n_source(coefficients_source->values[0],coefficients_source->values[1],coefficients_source->values[2]);//平面法向量
        Vec3d n_target(coefficients_target->values[0],coefficients_target->values[1],coefficients_target->values[2]);//平面法向量
        LOG(INFO)<<"n_source: \n"<<n_source;
        LOG(INFO)<<"n_target: \n"<<n_target;
        //计算两平面法向量的夹角
        double cos_theta = n_source.dot(n_target)/(n_source.norm()*n_target.norm());
        LOG(INFO)<<"cos_theta: "<<cos_theta;
        double angle_rad = std::acos(cos_theta);//计算两平面法向量的夹角，单位为弧度
        LOG(INFO)<<"angle_rad: "<<angle_rad;
        
        Mat3d R ;
        R= Eigen::AngleAxisd(angle_rad, n_source.cross(n_target).normalized()).toRotationMatrix();//计算旋转矩阵
        Eigen::Vector3d eulerAngle = R.eulerAngles(2, 1, 0);// 指定旋转顺序为ZYX 单位为弧度
        // Vec3d t = -R * (coefficients_source->values[3] * n_source - coefficients_target->values[3] * n_source);//计算平移向量
        Vec3d t = Eigen::Vector3d::Zero();
        Mat4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        LOG(INFO)<<"align plane:\n "<<T;
        LOG(INFO)<<"align plane euler: \n"<<eulerAngle*RAD2DEG;
        // Vec3d t = source_center - R*target_center;//计算平移向量

        /*  //测试旋转矩阵
         xmf::CloudPtr source_GroundPoint_trans(new xmf::PointCloudType);
         pcl::transformPointCloud(*source_GroundPoint, *source_GroundPoint_trans, T.cast<float>());
         xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_tools/build/temp_data/source_GroundPoint.pcd", *source_GroundPoint);
         xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_tools/build/temp_data/source_GroundPoint_trans.pcd", *source_GroundPoint_trans);
         xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_tools/build/temp_data/target_GroundPoint.pcd", *target_GroundPoint);
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_tools/build/temp_data/source_NoGroundPoint.pcd", *source_NoGroundPoint);
        xmf::SaveCloudToFile("/home/cetc21/xmf/my_slam_ws/slam_tools/build/temp_data/target_NoGroundPoint.pcd", *target_NoGroundPoint);
         */
        return 0;
}