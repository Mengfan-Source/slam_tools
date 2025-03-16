cmake_minimum_required(VERSION 3.16.3)
project(xmf_slam_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -ggdb")

# set(livox_point /home/llz/WorkPlaces/catkin_ws/livox/livox_ws2/devel/include)
set(thirdpart ${CMAKE_CURRENT_SOURCE_DIR}/thirtpart)
set(includepart ${CMAKE_CURRENT_SOURCE_DIR}/src/include)
add_definitions(-DCURRENT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
#   livox_ros_driver2
  message_generation
  eigen_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(glog REQUIRED)
# include_directories(${GLOG_INCLUDE_DIRS})

include_directories(
        ${includepart}/
        ${thirdpart}/
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        # ${livox_point}/include
        ${catkin_INCLUDE_DIRS} 
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        )

add_executable(pcd_align  src/src/pcd_align.cpp src/src/ndt_3d.cc src/src/point_cloud_utils.cc)

target_link_libraries(pcd_align 
                                        ${catkin_LIBRARIES}  
                                        ${Boost_LIBRARIES}
                                        ${PCL_LIBRARIES}
                                        tbb
                                        glog 
                                        gflags#对于tbb、glog、gflags等库，只需要链接到可执行文件中，并不需要find_package和include_directories
                                        ${yaml-cpp_LIBRARIES}
                                        yaml-cpp
                                        )
add_executable(remove_ground  src/src/remove_ground_main.cpp src/src/remove_ground.cpp)   
target_link_libraries(remove_ground
                                         ${catkin_LIBRARIES}  
                                         ${Boost_LIBRARIES}
                                         ${PCL_LIBRARIES}
                                         tbb
                                         glog 
                                         gflags#对于tbb、glog、gflags等库，只需要链接到可执行文件中，并不需要find_package和include_directories
                                         ${OpenCV_LIBRARIES}
                                         )
add_executable(lidar_fusion  src/src/lidar_fusion.cpp)   
target_link_libraries(lidar_fusion
                                         ${catkin_LIBRARIES}  
                                         ${PCL_LIBRARIES}
                                         tbb
                                         glog 
                                         gflags#对于tbb、glog、gflags等库，只需要链接到可执行文件中，并不需要find_package和include_directories
                                         )

find_package(Boost REQUIRED COMPONENTS filesystem)
add_executable(merge_single_point  src/src/merge_single_point.cpp)   
target_link_libraries(merge_single_point ${Boost_FILESYSTEM_LIBRARY}
                                       ${PCL_LIBRARIES}
                                       
                                )
add_executable(voxelgrid  src/src/voxelgrid.cpp src/src/point_cloud_utils.cc) 
target_link_libraries(voxelgrid                                   
${PCL_LIBRARIES}
${catkin_LIBRARIES}  
${Boost_LIBRARIES}
glog
gflags
)  
find_package(Boost REQUIRED COMPONENTS filesystem)
add_executable(merge_single_point_11  src/src/merge_single_point_11.cpp)   
target_link_libraries(merge_single_point_11 ${Boost_FILESYSTEM_LIBRARY}
                                       ${PCL_LIBRARIES}
                                       glog
                                       gflags
                                )
add_executable(test_euler_angle  src/src/test_euler_angle.cpp)   
target_link_libraries(test_euler_angle 
                                    ${PCL_LIBRARIES}
                                   ${catkin_LIBRARIES}  
                                )
                                
add_executable(pcd_align_filtered  src/src/pcd_align_filtered.cpp src/src/ndt_3d.cc src/src/point_cloud_utils.cc)
target_link_libraries(pcd_align_filtered 
                                        ${catkin_LIBRARIES}  
                                        ${Boost_LIBRARIES}
                                        ${PCL_LIBRARIES}
                                        tbb
                                        glog 
                                        gflags#对于tbb、glog、gflags等库，只需要链接到可执行文件中，并不需要find_package和include_directories
                                        ${yaml-cpp_LIBRARIES}
                                        yaml-cpp
                                        )