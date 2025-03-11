#include<iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0/M_PI ;

int main(){
    double x_angle = 30.0;  
    double y_angle = 40.0; 
    double z_angle = 70.0;  
    //通过欧拉角构造旋转矩阵，ZYX顺序（RPY顺序）
    Eigen::Matrix3d R;
    R = (Eigen::AngleAxisd(z_angle * DEG2RAD, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(y_angle * DEG2RAD, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(x_angle * DEG2RAD, Eigen::Vector3d::UnitX())).toRotationMatrix();
    std::cout << "matrix R = \n" << R << std::endl;
    //通过旋转矩阵求欧拉角 ZYX顺序（RPY顺序）
    Eigen::Vector3d rpy = R.eulerAngles(2,1,0);//zyx顺序，即roll pitch yaw顺序
    std::cout << "rpy from matrix = \n" << rpy * RAD2DEG << std::endl; //roll pitch yaw
    //通过旋转矩阵构造Sophus::SO3d
    Sophus::SO3d so3(R);
    //通过Sophus::SO3d求旋转矩阵
    Eigen::Matrix3d R_fromso3 = so3.matrix();
    std::cout << "R_fromso3 = \n" << R_fromso3 << std::endl;
    //通过Sophus::SO3d求欧拉角 ZYX顺序（RPY顺序）
    Eigen::Vector3d rpy_fromSO3 = so3.matrix().eulerAngles(2,1,0);
    std::cout << "rpy_fromso3 = \n" << rpy_fromSO3 * RAD2DEG << std::endl; //roll pitch yaw
     
    Eigen::Vector3d t(1.0, 2.0, 3.0);
    //从旋转矩阵和平移向量构造Sophus::SE3d
    Sophus::SE3d pose(R, t);
    //这种方式获取的欧拉角是错误的
    std::cout<<"dirrect Euler : "<< pose.angleX()*RAD2DEG<<", "<< pose.angleY()*RAD2DEG << ", "<< pose.angleZ()*RAD2DEG<<std::endl ;
  
   //从Sophus::SE3d中获取平移向量
    Eigen::Vector3d t_fromso3 = pose.translation();
    std::cout << "t_fromso3 = \n" << t_fromso3 << std::endl;

    //从Sophus::SE3d中获取旋转矩阵和欧拉角
    Eigen::Vector3d rpy_fromSE3 = pose.rotationMatrix().eulerAngles(2,1,0);//指定zyx顺序
    std::cout << "rpy_fromSE3 = \n" << rpy_fromSE3 * RAD2DEG << std::endl; //roll pitch yaw
    Eigen::Matrix3d R_fromSE3 = pose.rotationMatrix();
    std::cout << "R_fromSE3 = \n" << R_fromSE3 << std::endl;

    //从Sophus::SE3d中获取齐次坐标矩阵
    Eigen::Matrix4d T = pose.matrix();
    std::cout << "T = \n" << T << std::endl;

    //从Sophus::SE3d中获取平移向量
    Eigen::Vector3d t_fromSE3 = pose.translation();
    std::cout << "t_fromSE3 = \n" << t_fromSE3 << std::endl;


    //创建一个四元数 顺序wxyz
    Eigen::Quaterniond q_1(0.0029166745953261852,0.7073081731796265,-0.7068824768066406,0.004880243446677923);
    std::cout << "q_1 = \n" << q_1.coeffs() << std::endl;//输出顺序xyzw，与存储顺序相反
    //将四元数转换为ZYX欧拉角
    Eigen::Vector3d rpy_q1 = q_1.toRotationMatrix().eulerAngles(2,1,0);//zyx顺序，即roll pitch yaw顺序
    std::cout << "rpy from quaterniond = \n" << rpy_q1 * RAD2DEG << std::endl; //roll pitch yaw
    //将四元数转换为旋转矩阵
    Eigen::Matrix3d R_q1 = q_1.toRotationMatrix();
    std::cout << "R from quaterniond = \n" << R_q1 << std::endl;

    return 0;
}






































