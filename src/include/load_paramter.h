/**
 * @file load_paramter.h
 * @brief 参数加载服务类
 * @author Mengfan Xu
 * @version 1.0
 * @date 2024-10-18
 * @copyright Copyright (C) 2024 中国电科科技集团公司第二十一研究所
 */
#pragma once
#include <string>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "remove_ground.h"

namespace xmf
{
        /**
         * @brief 加载地面分割、参数服务类
         */
        class LoadParamter
        {
        public:
                LoadParamter(const std::string &param_path)
                {
                        this->msParamPath = param_path;
                }
                void LoadParamterFromYaml()
                {
                         cv::FileStorage fs(this->msParamPath, cv::FileStorage::READ);
                        this->mstruRemoveParamter.msPCDPath = (std::string)fs["pcd_path"];
                        this->mstruRemoveParamter.mfVoxelFilterSize = (float)fs["voxel_filter_size"];
                        this->mstruRemoveParamter.mnStatisticFilterKmeans = (int)fs["statictis_filter_kmeans"];
                        this->mstruRemoveParamter.mfStatisticFilterThre = (float)fs["statictis_filter_thred"];
                        this->mstruRemoveParamter.mnRansacIters = (int)fs["ransac_iters"];
                        this->mstruRemoveParamter.mfRansacDistanceThre = (float)fs["ransac_distance_thre"];
                        this->mstruRemoveParamter.mbVoxelFlag = (int)fs["vexel_flag"];
                        this->mstruRemoveParamter.mbStatisticFlag = (int)fs["statistic_flag"];
                        this->mstruRemoveParamter.mfMaxHight = (float)fs["pass_filter_maxHight"];
                        this->mstruRemoveParamter.mfMinHight = (float)fs["pass_filter_minHight"];
                        this->mstruRemoveParamter.mnKmeansFactor = (int)fs["kmeans_factor"];
                        this->mstruRemoveParamter.mnStaticThredFactor = (int)fs["thred_factor"];
                        this->mstruRemoveParamter.mbGroundCloudViewerFlag = (int)fs["pcl_viewer"];
                        this->mstruRemoveParamter.mbGroundCloudViewerSize = (int)fs["cloud_size_viewer"];
                        fs.release();
                }
                ~LoadParamter() {}

        public:
                std::string msParamPath;
                RemoveGroundParamter mstruRemoveParamter;
        };
}