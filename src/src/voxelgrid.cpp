#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "point_cloud_utils.h"
#include <pcl/filters/passthrough.h>
DEFINE_string(source, "/home/xmf/xmf_slam/mapping_offline_test/data/mapdata/map_opti2.pcd", "点云路径");
DEFINE_string(source_filtered, "/home/xmf/xmf_slam/mapping_offline_test/data/mapdata/map_filtered.pcd", "点云路径");

int main(int argc, char** argv) {
    FLAGS_log_dir = "/home/xmf/xmf_slam/slam_tools/log";
    FLAGS_minloglevel = google::ERROR;
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    xmf::CloudPtr source(new xmf::PointCloudType);
    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
    //滤除Z轴
    xmf::CloudPtr source_filtered_z(new xmf::PointCloudType);
    pcl::PassThrough<xmf::PointType> passz;
    passz.setInputCloud(source);
    passz.setFilterFieldName("z");
    passz.setFilterLimits(-50, 50);
    passz.filter(*source_filtered_z);
    //滤除Y轴
    xmf::CloudPtr source_filtered_y(new xmf::PointCloudType);
    pcl::PassThrough<xmf::PointType> passy;
    passy.setInputCloud(source_filtered_z);
    passy.setFilterFieldName("y");
    passy.setFilterLimits(-500, 500);
    passy.filter(*source_filtered_y);
    //滤除X轴
    xmf::CloudPtr source_filtered_x(new xmf::PointCloudType);
    pcl::PassThrough<xmf::PointType> passx;
    passx.setInputCloud(source_filtered_y);
    passx.setFilterFieldName("x");
    passx.setFilterLimits(-500, 500);
    passx.filter(*source_filtered_x);

    //体素滤波
    //如果体素滤波0.2*0.2*0.2，最大x*y*z<=17179869.176立方米
    xmf::CloudPtr source_filtered(new xmf::PointCloudType);
    pcl::VoxelGrid<xmf::PointType> sor_source;  
    sor_source.setInputCloud(source_filtered_x);  
    sor_source.setLeafSize(0.2f, 0.2f, 0.2f); // 设置体素的大小，这里是1cm  
    sor_source.filter(*source_filtered);  
    xmf::SaveCloudToFile(fLS::FLAGS_source_filtered, *source_filtered);
    return 0;

}