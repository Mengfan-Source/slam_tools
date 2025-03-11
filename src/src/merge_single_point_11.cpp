#include<boost/filesystem.hpp>
#include<iostream>
#include<string>
#include<fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include "eigen_types.h"
#include "point_types.h"
#include <pcl/common/transforms.h>
#include<pcl/filters/voxel_grid.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

std::string pcd_file_path = "/home/xmf/xmf_project/cetc11/bag/pcd";
std::string pose_file_path="/home/xmf/xmf_project/cetc11/fast_lio_cetc/src/Log/pose_tum_with_pcd.txt";
std::string merged_pcd="/home/xmf/xmf_project/cetc11/fast_lio_cetc/src/Log/allpoints.pcd";  
std::string merged_pcd_filtered="/home/xmf/xmf_project/cetc11/fast_lio_cetc/src/Log/allpoints_filtered.pcd";    
 // 使用 Boost 扫描获取指定路径下所有以 "extension" 结尾的文件列表
void scanFilesUseBoost(const std::string& rootPath,std::vector<std::string>& container, std::string extension){
    //清空容器
    //container.clear();
    // 将根路径转换为 Boost 文件系统路径
    boost::filesystem::path fullpath(rootPath);
    //// 检查路径是否存在且为目录
    if(!boost::filesystem::exists(fullpath) || !boost::filesystem::is_directory(fullpath))
    {
        std::cerr<<"File path not exist!" << std::endl;
        return;
    }
    //递归遍历目录
    boost::filesystem::recursive_directory_iterator end_iter;
    for(boost::filesystem::recursive_directory_iterator iter(fullpath); iter!=end_iter; iter++)
    {
        try
        {
            //如果是目录
            if(boost::filesystem::is_directory( *iter ) ){
                std::cout<<*iter << "is dir" << std::endl;
                //scanFilesUseRecursive(iter->path().string(),container,extension); //if find file recursively
            }
            else
            {
                //如果是普通文件且扩展名匹配
                if(boost::filesystem::is_regular_file(*iter) && iter->path().extension() == extension)
                    container.push_back(iter->path().string());
                // std::cout << *iter << " is a file" << std::endl;
            }
        } catch ( const std::exception & ex ){
            std::cerr << ex.what() << std::endl;
            std::cout<<"try_error"<<std::endl;
            continue;
        }
    }
}
bool GreaterEqSort(std::string filePath1, std::string filePath2)
{
    int len1 = filePath1.length();
    int len2 = filePath2.length();
    //    std::cout<<"len1:"<<len1<<" path:"<<filePath1<<std::endl;
    //    std::cout<<"len2:"<<len2<<" path:"<<filePath2<<std::endl;
    if(len1 < len2)
    {
        return false;
    }
    else if(len1 > len2)//若第一个路径长度大于第二个路径长度，直接返回true
    {
        return true;
    }
    else//如果两个文件长度相等，按照字典树勋，若第一个路径更大，则返回true
    {
        int iter = 0;
        while(iter < len1)
        {
            if(filePath1.at(iter) < filePath2.at(iter))
            {
                return false;
            }
            else if(filePath1.at(iter) > filePath2.at(iter))
            {
                return true;
            }
            ++iter;
        }
    }
    return true;
}
bool LessSort(std::string filePath1, std::string filePath2)
{
    return (!GreaterEqSort(filePath1, filePath2));
}
void readtxt(const std::string& filepath , std::vector<std::vector<double> > &result , int skipth)
{
    std::ifstream ifs(filepath);
    std::cout<<"load the txtfile:"<<filepath<<std::endl;
    int linefirst = 0;
    while (!ifs.eof())
    {
        std::string line;
        double value;
        std::getline(ifs, line);
        if (linefirst < skipth)
        {
            linefirst++;
            continue;
        }
        std::stringstream word;
        word << line;
        std::vector<double> v;
        while (!word.eof())
        {
            word >> value;
            v.push_back(value);
        }
        if(v.size()>3)
        {
            result.push_back(v);
        }
    }
    ifs.close();
}
void gettime_from_filepath(const std::string& filepath , double &result)
{
        // 使用 stringstream 分割字符串
    std::istringstream ss(filepath);
    std::string segment;
    std::vector<std::string> segments;

    // 按 "/" 拆分字符串
    while (std::getline(ss, segment, '/')) {
        if (!segment.empty()) {
            segments.push_back(segment);
        }
    }

    // 获取最后一段
    if (!segments.empty()) {
        std::string last_segment = segments.back();

        // 在最后一段中提取数字部分
        size_t dot_pos = last_segment.find_last_of('.'); // 找到最后一个“.”的位置
        if (dot_pos != std::string::npos) {
            std::string timestamp_str = last_segment.substr(0, dot_pos);
            try {
                double timestamp = std::stod(timestamp_str);
                result = timestamp;
                // 输出整数时间戳
                // std::cout << "Timestamp as integer: " << timestamp << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Error converting string to integer: " << e.what() << std::endl;
            }
        }
    }
}
SE3 findpose_in_time(std::vector<std::vector<double>> &lio_poses, double time_s)
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

int main(int argc, char** argv){
        std::vector<std::string> pcd_file_list;
        scanFilesUseBoost(pcd_file_path,pcd_file_list,".pcd");//相机点云列表
        std::sort(pcd_file_list.begin(), pcd_file_list.end(), LessSort);
        for(int i =0;i<pcd_file_list.size();i++){
            std::cout<<pcd_file_list[i]<<std::endl;
        }
        std::vector<std::vector<double> > lio_poses;//LIO轨迹位姿列表
        readtxt(pose_file_path, lio_poses,0);
        for(int i = 0;i<lio_poses.size();i++){
            for(int j = 0;j<lio_poses[i].size();j++){
                std::cout<<lio_poses[i][j]<<", ";
            }
            std::cout<<std::endl;
        }
        xmf::CloudPtr merged_points(new xmf::PointCloudType);
        long long pcd_num = 0;


        //获取初始时刻位姿差值：
        double first_pcd_time = 0.0;//11所采集数据时间不同步，做时间同步
        gettime_from_filepath(pcd_file_list[0],first_pcd_time);
        double time_diff = fabs(lio_poses[0][0]-first_pcd_time);

        for(auto &file:pcd_file_list){
                //读取PCD文件
                xmf::CloudPtr points(new xmf::PointCloudType);
                xmf::CloudPtr points_imu(new xmf::PointCloudType);
                xmf::CloudPtr points_transformed(new xmf::PointCloudType);
                pcl::io::loadPCDFile(file, *points);//读取相机点云
                double curtime = 0.0;
                gettime_from_filepath(file,curtime);
                std::cout<< std::fixed << std::setprecision(15)<<"curtime: "<<curtime<<std::endl;
                curtime+=time_diff;//针对十一所的数据做位姿补偿
                std::cout<< std::fixed << std::setprecision(15)<<"curtime_remove_time_diff: "<<curtime<<std::endl;
                SE3 cur_pose = findpose_in_time(lio_poses,curtime);
                std::vector<double> match;
                if(!PoseInterp<std::vector<double>>(curtime,lio_poses,[](const std::vector<double> &s) { return s[0]; },[](const std::vector<double> &s) { 
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
                        cur_pose = findpose_in_time(lio_poses,curtime);
                    }

                std::cout<<"found_pose: "<<cur_pose.matrix().cast<double>()<<std::endl;
                pcl::transformPointCloud(*points,*points_transformed,cur_pose.matrix().cast<double>());
                *merged_points += *points_transformed;
                // *merged_points += *points;//拼接点云
                pcd_num++;
        }
        pcl::VoxelGrid<xmf::PointType> sor_source;  
        sor_source.setInputCloud(merged_points );  
        sor_source.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素的大小，这里是1cm  
        xmf::CloudPtr merged_points_filtered(new xmf::PointCloudType);
        sor_source.filter(*merged_points_filtered);  

        pcl::io::savePCDFileASCII(merged_pcd, *merged_points);
        pcl::io::savePCDFileASCII(merged_pcd_filtered, *merged_points_filtered);
        std::cout << "all PCD saved successfully. "<< std::endl;
        std::cout << "merged pcd num: " << pcd_num << std::endl;


        return 0;
}