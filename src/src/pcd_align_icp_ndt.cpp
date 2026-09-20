/**
 * @file pcd_align_icp_ndt.cpp
 * @brief 读取两个pcd点云，根据配置文件中给定的初值(平移+四元数)分别进行ICP与NDT配准，
 *        按照配准结果把两个原始点云拼接后输出，并打印配准初值与配准结果
 * @author MengfanXu
 * @version 1.1
 * @date 2026-0917
 * @copyright Copyright (C) 2026 中国电科科技集团公司第二十一研究所
 *
 * 说明：程序不使用gflags，所有参数(点云路径、配准初值、配准参数、输出路径)都从yaml配置文件读取，
 *       配置文件中配准初值的旋转部分用四元数给出，顺序为x y z w
 *       配置文件路径可通过命令行第一个参数指定，不指定时使用下面的默认路径
 */
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <yaml-cpp/yaml.h>

#include "eigen_types.h"
#include "ndt_3d.h"
#include "point_types.h"

namespace {

constexpr double kRAD2DEG = 180.0 / M_PI;
/// 默认配置文件路径，相对于build目录
const std::string kDefaultConfigPath = "./../src/config/param_pcdalign_icp_ndt.yaml";

/// 配准参数，全部来自yaml配置文件
struct Config {
        //输入点云
        std::string source_path;
        std::string target_path;

        //配准初值：平移单位m，旋转用四元数(单位四元数，顺序x y z w)
        double init_x = 0.0;
        double init_y = 0.0;
        double init_z = 0.0;
        double init_qx = 0.0;
        double init_qy = 0.0;
        double init_qz = 0.0;
        double init_qw = 1.0;

        //配准输入点云降采样，只影响配准，拼接时使用的仍然是原始点云
        double reg_voxel_leaf = 0.1;

        //ICP参数
        int icp_max_iteration = 50;
        double icp_max_correspondence_distance = 2.0;
        double icp_transformation_epsilon = 1e-8;
        double icp_euclidean_fitness_epsilon = 1e-6;

        //NDT参数
        double ndt_voxel_size = 0.5;
        int ndt_max_iteration = 30;
        bool ndt_use_nearby6 = true;
        double ndt_res_outlier_th = 20.0;
        int ndt_min_effective_pts = 10;

        //结果输出
        std::string final_result = "ndt";
        bool save_result = true;
        bool binary_output = true;
        std::string merged_pcd = "./result/merged_points.pcd";
        std::string source_transformed_pcd = "./result/source_transformed.pcd";
        std::string result_txt = "./result/registration_result.txt";
};

/// 读取yaml中某个键的值，键不存在或解析失败时使用默认值
template <typename T>
T ReadYaml(const YAML::Node& parent, const std::string& key, const T& default_value) {
    if (!parent) {
        return default_value;
    }
    const YAML::Node item = parent[key];
    if (!item) {
        LOG(WARNING) << "配置文件中缺少 " << key << "，使用默认值: " << default_value;
        return default_value;
    }
    try {
        return item.as<T>();
    } catch (const std::exception& ex) {
        LOG(ERROR) << "配置文件中 " << key << " 解析失败(" << ex.what() << ")，使用默认值: " << default_value;
        return default_value;
    }
}

/// 解析配置文件，文件不存在或缺少必要参数时抛异常
Config LoadConfig(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("配置文件不存在: " + path);
    }

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(path);
    } catch (const std::exception& ex) {
        throw std::runtime_error("配置文件解析失败: " + path + ", " + ex.what());
    }

    Config config;
    config.source_path = ReadYaml<std::string>(yaml, "source_path", "");
    config.target_path = ReadYaml<std::string>(yaml, "target_path", "");
    if (config.source_path.empty() || config.target_path.empty()) {
        throw std::runtime_error("配置文件 " + path + " 中必须指定 source_path 与 target_path");
    }

    //配准初值：旋转部分用四元数给出
    const YAML::Node init_pose = yaml["init_pose"];
    if (!init_pose) {
        LOG(WARNING) << "配置文件中缺少 init_pose，配准初值使用单位阵";
    }
    config.init_x = ReadYaml<double>(init_pose, "x", 0.0);
    config.init_y = ReadYaml<double>(init_pose, "y", 0.0);
    config.init_z = ReadYaml<double>(init_pose, "z", 0.0);
    config.init_qx = ReadYaml<double>(init_pose, "qx", 0.0);
    config.init_qy = ReadYaml<double>(init_pose, "qy", 0.0);
    config.init_qz = ReadYaml<double>(init_pose, "qz", 0.0);
    config.init_qw = ReadYaml<double>(init_pose, "qw", 1.0);

    //配准参数
    config.reg_voxel_leaf = ReadYaml<double>(yaml, "reg_voxel_leaf", config.reg_voxel_leaf);

    const YAML::Node icp = yaml["icp_params"];
    config.icp_max_iteration = ReadYaml<int>(icp, "max_iteration", config.icp_max_iteration);
    config.icp_max_correspondence_distance =
        ReadYaml<double>(icp, "max_correspondence_distance", config.icp_max_correspondence_distance);
    config.icp_transformation_epsilon =
        ReadYaml<double>(icp, "transformation_epsilon", config.icp_transformation_epsilon);
    config.icp_euclidean_fitness_epsilon =
        ReadYaml<double>(icp, "euclidean_fitness_epsilon", config.icp_euclidean_fitness_epsilon);

    const YAML::Node ndt = yaml["ndt_params"];
    config.ndt_voxel_size = ReadYaml<double>(ndt, "voxel_size", config.ndt_voxel_size);
    config.ndt_max_iteration = ReadYaml<int>(ndt, "max_iteration", config.ndt_max_iteration);
    config.ndt_use_nearby6 = ReadYaml<bool>(ndt, "use_nearby6", config.ndt_use_nearby6);
    config.ndt_res_outlier_th = ReadYaml<double>(ndt, "res_outlier_th", config.ndt_res_outlier_th);
    config.ndt_min_effective_pts = ReadYaml<int>(ndt, "min_effective_pts", config.ndt_min_effective_pts);

    const YAML::Node result = yaml["result_params"];
    config.final_result = ReadYaml<std::string>(result, "final_result", config.final_result);
    config.save_result = ReadYaml<bool>(result, "save_result", config.save_result);
    config.binary_output = ReadYaml<bool>(result, "binary_output", config.binary_output);
    config.merged_pcd = ReadYaml<std::string>(result, "merged_pcd", config.merged_pcd);
    config.source_transformed_pcd =
        ReadYaml<std::string>(result, "source_transformed_pcd", config.source_transformed_pcd);
    config.result_txt = ReadYaml<std::string>(result, "result_txt", config.result_txt);

    //参数检查
    const Quatd init_q(config.init_qw, config.init_qx, config.init_qy, config.init_qz);
    if (init_q.norm() < 1e-6) {
        throw std::runtime_error("配置文件 " + path + " 中 init_pose 的四元数(qx qy qz qw)不能为零四元数");
    }
    if (std::abs(init_q.norm() - 1.0) > 1e-3) {
        LOG(WARNING) << "init_pose 的四元数模长为 " << init_q.norm() << ", 使用时将自动归一化";
    }
    if (config.ndt_voxel_size <= 0.0) {
        throw std::runtime_error("配置文件 " + path + " 中 ndt_params/voxel_size 必须大于0");
    }
    if (config.ndt_max_iteration <= 0 || config.icp_max_iteration <= 0) {
        throw std::runtime_error("配置文件 " + path + " 中 ICP/NDT最大迭代次数必须大于0");
    }
    if (config.final_result != "icp" && config.final_result != "ndt") {
        throw std::runtime_error("配置文件 " + path + " 中 result_params/final_result 只能为 icp 或 ndt");
    }
    return config;
}

/// 打印配置信息
void PrintConfig(const std::string& path, const Config& config) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "==================== 配置信息 ====================" << std::endl;
    std::cout << "配置文件    : " << path << std::endl;
    std::cout << "source点云  : " << config.source_path << std::endl;
    std::cout << "target点云  : " << config.target_path << std::endl;
    std::cout << "配准初值    : 平移(m) " << config.init_x << " " << config.init_y << " " << config.init_z
              << ", 四元数(x y z w) " << config.init_qx << " " << config.init_qy << " " << config.init_qz << " "
              << config.init_qw << std::endl;
    std::cout << "配准降采样  : " << (config.reg_voxel_leaf > 0.0 ? std::to_string(config.reg_voxel_leaf) + " m"
                                                                : std::string("关闭"))
              << std::endl;
    std::cout << "ICP参数     : max_iteration " << config.icp_max_iteration << ", max_correspondence_distance "
              << config.icp_max_correspondence_distance << std::scientific << std::setprecision(1)
              << ", transformation_epsilon " << config.icp_transformation_epsilon
              << ", euclidean_fitness_epsilon " << config.icp_euclidean_fitness_epsilon << std::fixed
              << std::setprecision(6) << std::endl;
    std::cout << "NDT参数     : voxel_size " << config.ndt_voxel_size << ", max_iteration "
              << config.ndt_max_iteration << ", use_nearby6 " << (config.ndt_use_nearby6 ? "true" : "false")
              << ", res_outlier_th " << config.ndt_res_outlier_th << ", min_effective_pts "
              << config.ndt_min_effective_pts << std::endl;
    std::cout << "最终结果    : " << config.final_result << ", 保存结果 "
              << (config.save_result ? "true" : "false") << std::endl
              << std::endl;
}

/// 配准初值：平移由配置文件的x/y/z给出，旋转部分直接由四元数(x y z w)生成
SE3 InitialPoseFromQuaternion(double qx, double qy, double qz, double qw, double tx, double ty, double tz) {
    Quatd q(qw, qx, qy, qz);
    q.normalize();  // 四元数写入配置文件时可能有精度误差，这里统一归一化
    return SE3(q, Vec3d(tx, ty, tz));
}

/// 旋转矩阵转欧拉角(yaw pitch roll，单位度，ZYX顺序)
Vec3d RotationMatrixToYawPitchRollDeg(const Mat3d& R) {
    return R.eulerAngles(2, 1, 0) * kRAD2DEG;
}

/// 旋转矩阵转欧拉角(roll pitch yaw，单位度)，仅用于打印参考
Vec3d RotationMatrixToRollPitchYawDeg(const Mat3d& R) {
    const Vec3d yaw_pitch_roll = RotationMatrixToYawPitchRollDeg(R);
    return Vec3d(yaw_pitch_roll[2], yaw_pitch_roll[1], yaw_pitch_roll[0]);
}

/// 打印位姿：矩阵、平移、欧拉角与四元数
void PrintPose(const std::string& tag, const SE3& pose) {
    const Vec3d yaw_pitch_roll = RotationMatrixToYawPitchRollDeg(pose.rotationMatrix());
    const Vec3d roll_pitch_yaw = RotationMatrixToRollPitchYawDeg(pose.rotationMatrix());
    const Quatd q = pose.unit_quaternion();

    std::cout << "---------- " << tag << " ----------" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "T = " << std::endl << pose.matrix() << std::endl;
    std::cout << "t                      (x y z, m) = " << pose.translation().transpose() << std::endl;
    std::cout << "euler RPY      (roll pitch yaw, deg) = " << roll_pitch_yaw.transpose()
              << "   <-- 仅作参考，配置文件中的旋转用四元数给出" << std::endl;
    std::cout << "euler ZYX      (yaw pitch roll, deg) = " << yaw_pitch_roll.transpose() << std::endl;
    std::cout << "quaternion                 (x y z w) = " << q.coeffs().transpose()
              << "   <-- 与配置文件中的qx/qy/qz/qw顺序一致" << std::endl
              << std::endl;
}

/// 体素滤波，leaf_size<=0时返回原点云的拷贝
xmf::CloudPtr DownSample(const xmf::CloudPtr& cloud, double leaf_size) {
    xmf::CloudPtr out(new xmf::PointCloudType);
    if (leaf_size <= 0.0) {
        *out = *cloud;
        return out;
    }

    pcl::VoxelGrid<xmf::PointType> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size),
                      static_cast<float>(leaf_size));
    voxel.filter(*out);
    return out;
}

/// 把src按位姿T变换后，统计其到tgt的平均最近邻距离(m)，用于横向比较不同配准结果
double MeanNearestNeighborDistance(const xmf::CloudPtr& src, const xmf::CloudPtr& tgt, const Mat4d& T) {
    if (src->empty() || tgt->empty()) {
        return -1.0;
    }

    xmf::CloudPtr src_trans(new xmf::PointCloudType);
    pcl::transformPointCloud(*src, *src_trans, T.cast<float>());

    pcl::KdTreeFLANN<xmf::PointType> kdtree;
    kdtree.setInputCloud(tgt);

    std::vector<int> index(1);
    std::vector<float> sq_dist(1);
    double sum_dist = 0.0;
    size_t valid_num = 0;
    for (const auto& pt : src_trans->points) {
        if (kdtree.nearestKSearch(pt, 1, index, sq_dist) > 0) {
            sum_dist += std::sqrt(sq_dist[0]);
            valid_num++;
        }
    }
    return valid_num == 0 ? -1.0 : sum_dist / valid_num;
}

/// 保存点云，binary为true时保存二进制格式
void SaveCloud(const std::string& path, xmf::PointCloudType& cloud, bool binary) {
    cloud.height = 1;
    cloud.width = cloud.size();
    cloud.is_dense = false;
    if (binary) {
        pcl::io::savePCDFileBinary(path, cloud);
    } else {
        pcl::io::savePCDFileASCII(path, cloud);
    }
}

/// 创建文件所在目录
void EnsureParentDirectory(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.has_parent_path() && !std::filesystem::exists(p.parent_path())) {
        std::filesystem::create_directories(p.parent_path());
    }
}

/// 点云是否加载成功
bool LoadCloud(const std::string& path, const std::string& name, xmf::CloudPtr& cloud) {
    if (!std::filesystem::exists(path)) {
        LOG(ERROR) << name << "点云文件不存在: " << path;
        return false;
    }
    if (pcl::io::loadPCDFile(path, *cloud) < 0) {
        LOG(ERROR) << name << "点云文件读取失败: " << path;
        return false;
    }
    if (cloud->empty()) {
        LOG(ERROR) << name << "点云为空: " << path;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    // 日志同时输出到stderr，便于直接查看运行过程(不依赖log目录是否存在)
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);

    // 配置文件路径：命令行第一个参数可选指定，不指定时使用默认路径
    const std::string config_path = (argc > 1) ? std::string(argv[1]) : kDefaultConfigPath;

    Config config;
    try {
        config = LoadConfig(config_path);
    } catch (const std::exception& ex) {
        LOG(ERROR) << ex.what();
        std::cout << "用法: " << argv[0] << " [配置文件路径]" << std::endl
                  << "默认配置文件: " << kDefaultConfigPath << std::endl
                  << "配置文件示例: " << std::endl
                  << "  source_path: ./scans148.pcd" << std::endl
                  << "  target_path: ./scans110.pcd" << std::endl
                  << "  init_pose:            #配准初值，平移单位m，旋转用四元数(单位四元数)" << std::endl
                  << "    x: 0.0" << std::endl
                  << "    y: 0.0" << std::endl
                  << "    z: 0.0" << std::endl
                  << "    qx: 0.0             #四元数x" << std::endl
                  << "    qy: 0.0             #四元数y" << std::endl
                  << "    qz: 0.0             #四元数z" << std::endl
                  << "    qw: 1.0             #四元数w" << std::endl;
        return 1;
    }
    PrintConfig(config_path, config);

    // 加载两个原始点云，拼接使用的就是这两个原始点云
    xmf::CloudPtr source_raw(new xmf::PointCloudType);
    xmf::CloudPtr target_raw(new xmf::PointCloudType);
    if (!LoadCloud(config.source_path, "source", source_raw) ||
        !LoadCloud(config.target_path, "target", target_raw)) {
        return 1;
    }
    std::cout << "source点数: " << source_raw->size() << ", target点数: " << target_raw->size() << std::endl
              << std::endl;

    // 配准初值：平移由配置文件的x/y/z给定，旋转部分直接由四元数生成
    const SE3 init_pose = InitialPoseFromQuaternion(config.init_qx, config.init_qy, config.init_qz, config.init_qw,
                                                   config.init_x, config.init_y, config.init_z);
    const Mat4d T_init = init_pose.matrix();
    PrintPose("配准初值 T_init", init_pose);

    // 配准使用降采样后的点云
    xmf::CloudPtr source_reg = DownSample(source_raw, config.reg_voxel_leaf);
    xmf::CloudPtr target_reg = DownSample(target_raw, config.reg_voxel_leaf);
    std::cout << "配准输入点数: source = " << source_reg->size() << ", target = " << target_reg->size() << std::endl
              << std::endl;

    const double init_error = MeanNearestNeighborDistance(source_reg, target_reg, T_init);

    // ---------------- ICP配准 ----------------
    Mat4d T_icp = T_init;
    double icp_fitness = -1.0;
    bool icp_success = false;
    double icp_cost_ms = 0.0;
    {
        pcl::IterativeClosestPoint<xmf::PointType, xmf::PointType> icp;
        icp.setMaximumIterations(config.icp_max_iteration);
        if (config.icp_max_correspondence_distance > 0.0) {
            icp.setMaxCorrespondenceDistance(config.icp_max_correspondence_distance);
        }
        icp.setTransformationEpsilon(config.icp_transformation_epsilon);
        icp.setEuclideanFitnessEpsilon(config.icp_euclidean_fitness_epsilon);
        icp.setInputSource(source_reg);
        icp.setInputTarget(target_reg);

        xmf::CloudPtr source_aligned(new xmf::PointCloudType);
        auto t1 = std::chrono::high_resolution_clock::now();
        icp.align(*source_aligned, T_init.cast<float>());
        auto t2 = std::chrono::high_resolution_clock::now();

        icp_cost_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        icp_success = icp.hasConverged();
        if (icp_success) {
            T_icp = icp.getFinalTransformation().cast<double>();
            icp_fitness = icp.getFitnessScore();
        } else {
            LOG(ERROR) << "ICP未收敛, 结果不可用";
        }
    }

    // ---------------- NDT配准 ----------------
    Mat4d T_ndt = T_init;
    double ndt_mean_res = -1.0;
    bool ndt_success = false;
    double ndt_cost_ms = 0.0;
    {
        xmf::Ndt3d::Options options;
        options.voxel_size_ = config.ndt_voxel_size;
        options.max_iteration_ = config.ndt_max_iteration;
        options.nearby_type_ =
            config.ndt_use_nearby6 ? xmf::Ndt3d::NearbyType::NEARBY6 : xmf::Ndt3d::NearbyType::CENTER;
        options.res_outlier_th_ = config.ndt_res_outlier_th;
        options.min_effective_pts_ = config.ndt_min_effective_pts;
        options.remove_centroid_ = false;  // 使用配置文件给定的初值，不用质心作为初值

        xmf::Ndt3d ndt(options);
        ndt.SetSource(source_reg);
        ndt.SetTarget(target_reg);

        xmf::GridSearchResult gr;
        gr.msPose = init_pose;  // 欧拉角构造的初值直接作为NDT初值，不经过四元数转换
        auto t1 = std::chrono::high_resolution_clock::now();
        ndt_success = ndt.AlignNdt(gr);
        auto t2 = std::chrono::high_resolution_clock::now();

        ndt_cost_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        if (ndt_success) {
            T_ndt = gr.msResultPose.matrix();
            ndt_mean_res = gr.dx;
        } else {
            LOG(ERROR) << "NDT未收敛, 结果不可用";
        }
    }

    const double icp_error = icp_success ? MeanNearestNeighborDistance(source_reg, target_reg, T_icp) : -1.0;
    const double ndt_error = ndt_success ? MeanNearestNeighborDistance(source_reg, target_reg, T_ndt) : -1.0;

    // ---------------- 打印配准结果 ----------------
    std::cout << "==================== 配准结果 ====================" << std::endl;
    if (icp_success) {
        PrintPose("ICP配准结果 T_icp", xmf::Mat4ToSE3(T_icp));
    }
    if (ndt_success) {
        PrintPose("NDT配准结果 T_ndt", xmf::Mat4ToSE3(T_ndt));
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "结果对比:" << std::endl;
    std::cout << std::setw(10) << "方法" << std::setw(18) << "平均最近邻距离(m)" << std::setw(18) << "耗时(ms)"
              << std::setw(16) << "算法代价" << std::endl;
    std::cout << std::setw(10) << "初值" << std::setw(18) << init_error << std::setw(18) << "-" << std::setw(16)
              << "-" << std::endl;
    std::cout << std::setw(10) << "ICP" << std::setw(18) << icp_error << std::setw(18) << icp_cost_ms
              << std::setw(16) << (icp_success ? std::to_string(icp_fitness) : std::string("未收敛")) << std::endl;
    std::cout << std::setw(10) << "NDT" << std::setw(18) << ndt_error << std::setw(18) << ndt_cost_ms
              << std::setw(16) << (ndt_success ? std::to_string(ndt_mean_res) : std::string("未收敛")) << std::endl
              << std::endl;
    std::cout << "说明: ICP的算法代价为fitness score(对应点均方距离), NDT的算法代价为每轮的平均残差(马氏距离), "
                 "两者不可直接比较, 横向比较请使用平均最近邻距离"
              << std::endl
              << std::endl;

    // ---------------- 选择最终结果并拼接原始点云 ----------------
    std::string used_method = config.final_result;
    bool used_success = (used_method == "icp") ? icp_success : ndt_success;
    if (!used_success) {
        const std::string backup = (used_method == "icp") ? "ndt" : "icp";
        const bool backup_success = (backup == "icp") ? icp_success : ndt_success;
        if (backup_success) {
            LOG(WARNING) << used_method << "未收敛, 自动改用" << backup << "的结果进行拼接";
            used_method = backup;
            used_success = true;
        }
    }
    if (!used_success) {
        LOG(ERROR) << "ICP与NDT均未收敛, 不进行点云拼接";
        return 1;
    }

    const Mat4d T_final = (used_method == "icp") ? T_icp : T_ndt;
    std::cout << "==================== 最终结果 ====================" << std::endl;
    std::cout << "采用 " << used_method << " 的配准结果拼接原始点云" << std::endl;
    PrintPose("最终变换 T_final (" + used_method + ")", xmf::Mat4ToSE3(T_final));

    // 把最终结果以四元数形式给出，方便直接写回配置文件的init_pose
    {
        const Vec3d t_final = T_final.block<3, 1>(0, 3);
        Quatd q_final(T_final.block<3, 3>(0, 0));
        q_final.normalize();
        std::cout << std::fixed << std::setprecision(9) << "可直接写回配置文件 init_pose 的初值(四元数):"
                  << std::endl
                  << "  x: " << t_final[0] << std::endl
                  << "  y: " << t_final[1] << std::endl
                  << "  z: " << t_final[2] << std::endl
                  << "  qx: " << q_final.x() << std::endl
                  << "  qy: " << q_final.y() << std::endl
                  << "  qz: " << q_final.z() << std::endl
                  << "  qw: " << q_final.w() << std::endl
                  << std::endl;
    }

    // 用最终变换处理原始source点云，并与原始target点云拼接
    xmf::CloudPtr source_transformed(new xmf::PointCloudType);
    pcl::transformPointCloud(*source_raw, *source_transformed, T_final.cast<float>());
    xmf::CloudPtr merged(new xmf::PointCloudType);
    *merged = *target_raw;
    *merged += *source_transformed;
    merged->height = 1;
    merged->width = merged->size();
    merged->is_dense = false;
    std::cout << "拼接后点云点数: " << merged->size() << " (target " << target_raw->size() << " + source "
              << source_transformed->size() << ")" << std::endl
              << std::endl;

    // ---------------- 保存结果 ----------------
    if (config.save_result) {
        try {
            EnsureParentDirectory(config.merged_pcd);
            EnsureParentDirectory(config.source_transformed_pcd);
            EnsureParentDirectory(config.result_txt);

            SaveCloud(config.merged_pcd, *merged, config.binary_output);
            SaveCloud(config.source_transformed_pcd, *source_transformed, config.binary_output);

            std::ofstream fout(config.result_txt);
            if (fout.is_open()) {
                fout << std::fixed << std::setprecision(9);
                fout << "# pcd_align_icp_ndt 配准结果" << std::endl;
                fout << "config: " << config_path << std::endl;
                fout << "source: " << config.source_path << ", points: " << source_raw->size() << std::endl;
                fout << "target: " << config.target_path << ", points: " << target_raw->size() << std::endl;
                fout << "reg_voxel_leaf: " << config.reg_voxel_leaf << std::endl;
                fout << "used_method: " << used_method << std::endl;
                fout << "init_error(nn): " << init_error << ", icp_error(nn): " << icp_error
                     << ", ndt_error(nn): " << ndt_error << std::endl;
                fout << std::endl << "# T_init" << std::endl << T_init << std::endl;
                if (icp_success) {
                    fout << "# T_icp, fitness_score: " << icp_fitness << std::endl << T_icp << std::endl;
                } else {
                    fout << "# T_icp: not converged" << std::endl;
                }
                if (ndt_success) {
                    fout << "# T_ndt, mean_res: " << ndt_mean_res << std::endl << T_ndt << std::endl;
                } else {
                    fout << "# T_ndt: not converged" << std::endl;
                }
                fout << "# T_final (" << used_method << ")" << std::endl << T_final << std::endl;
                fout.close();
                std::cout << "配准结果文本已保存: " << config.result_txt << std::endl;
            } else {
                LOG(ERROR) << "无法写入配准结果文本: " << config.result_txt;
            }
            std::cout << "拼接后的原始点云已保存: " << config.merged_pcd << std::endl;
            std::cout << "变换后的source点云已保存: " << config.source_transformed_pcd << std::endl;
        } catch (const std::exception& ex) {
            LOG(ERROR) << "保存结果失败: " << ex.what();
            return 1;
        }
    }

    return 0;
}
