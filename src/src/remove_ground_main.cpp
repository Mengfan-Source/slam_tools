/**
 * @file remove_ground.h
 * @brief 去除点云地图中的地面点云类
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2024-10-17
 * @copyright Copyright (C) 2024 中国电科科技集团公司第二十一研究所
 */

#include "load_paramter.h"
#include "remove_ground.h"
int main(int argc, char** argv){
        xmf::LoadParamter load_paramter("/home/cetc21/xmf/my_slam_ws/slam_test/src/config/param.yaml");
        load_paramter.LoadParamterFromYaml();
        xmf::RemoveGroundCloud rgc(load_paramter.mstruRemoveParamter);
        rgc.Remove();
        return 0;
}
  