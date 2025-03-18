#include <iostream>	
#include <cmath>       
#include <pcl/point_types.h>	    
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iomanip> 
namespace airy_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint16_t ring = 0;
      double timestamp = 0;
      float feature;
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
    (float, feature, feature)
)
ros::Subscriber sub_rslidar;
// ros::Publisher pub_rslidar;
void rsHandler(sensor_msgs::PointCloud2 pc_msg) {
    pcl::PointCloud<airy_ros::Point>::Ptr pc(new pcl::PointCloud<airy_ros::Point>());
    pcl::fromROSMsg(pc_msg, *pc);
    std::cout<<"point size: "<<pc->points.size()<<std::endl;
    for(int i = 0;i<pc->points.size();i++){
        std::cout<<"point intensity: "<<pc->points[i].intensity<<std::endl;
        std::cout<<"point ring: "<<pc->points[i].ring<<std::endl;
        // if(i ==0 || i==pc->points.size()-1)
        std::cout<<std::fixed << std::setprecision(8)<<"point timestamp: "<<pc->points[i].timestamp<<std::endl;
        std::cout<<std::fixed << std::setprecision(8)<<"point feature: "<<pc->points[i].feature<<std::endl;
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rs_lidar_test");
    ros::NodeHandle nh;
    sub_rslidar = nh.subscribe("/rslidar_points", 1, rsHandler);
    ROS_INFO("Listening to /rslidar_points ......");
    ros::spin();
    return 0;
}
