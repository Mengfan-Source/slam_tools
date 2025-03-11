/**
 * @file remove_ground.h
 * @brief 去除点云地图中的地面点云类
 * @author MengfanXu (15262133937@163.com)
 * @version 1.0
 * @date 2024-10-17
 * @copyright Copyright (C) 2024 中国电科科技集团公司第二十一研究所
 */

#include "remove_ground.h"
namespace xmf {
RemoveGroundCloud::RemoveGroundCloud (const RemoveGroundParamter& param) {
        this->mstruParam = param;
        mpCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        mpNoGroundCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        mpCloudFilteredIn = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        mpCloudFiltered = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        mpCloudFilterOut = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        mpNoGroundPoints = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
}

void RemoveGroundCloud::Remove () {
        if (pcl::io::loadPCDFile<pcl::PointXYZI> (this->mstruParam.msPCDPath, *mpCloud) == -1)
        {
                printf ("%s", "In PCD Path, Failed to open the input cloud");
        }
        //对Z轴数据进行筛选
        pcl::PassThrough<pcl::PointXYZI> passFilter;
        passFilter.setInputCloud (mpCloud);
        passFilter.setFilterFieldName ("z");
        passFilter.setFilterLimits (this->mstruParam.mfMinHight, this->mstruParam.mfMaxHight);
        passFilter.setFilterLimitsNegative (false);
        passFilter.filter (*mpCloudFilteredIn);
        //如果进行点云体素滤波降采样标志位和统计滤波标志位都为真，先进行体素滤波降采样在进行统计滤波
        if (this->mstruParam.mbVoxelFlag && this->mstruParam.mbStatisticFlag) {
                //体素滤波降采样
                pcl::VoxelGrid<pcl::PointXYZI> sor;
                sor.setInputCloud (mpCloudFilteredIn);
                sor.setLeafSize (this->mstruParam.mfVoxelFilterSize, this->mstruParam.mfVoxelFilterSize, this->mstruParam.mfVoxelFilterSize);
                sor.filter (*mpCloudFiltered);
                if(0)
                {
                         //程序运行过程中临时显示点云
                        pcl::visualization::PCLVisualizer::Ptr viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_z1 (mpCloudFiltered, "z");
                        viewer1->setBackgroundColor (0, 0, 0);
                        viewer1->addPointCloud<pcl::PointXYZI> (mpCloudFiltered, color_z1,"target_points");
                        viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, this->mstruParam.mbGroundCloudViewerSize,"target_points");
                        viewer1->initCameraParameters ();
                        while (!viewer1->wasStopped ()) {
                                viewer1->spinOnce ();
                        }
                }
               
                //统计滤波
                pcl::StatisticalOutlierRemoval<pcl::PointXYZI> remove_discrete_points; //参数：统计滤波最近邻点个数，标准差阈值
                remove_discrete_points.setInputCloud (mpCloudFiltered);
                remove_discrete_points.setMeanK (this->mstruParam.mnStatisticFilterKmeans);//设置在进行统计滤波时考虑的临近点个数
                remove_discrete_points.setStddevMulThresh (this->mstruParam.mfStatisticFilterThre);//如果参数设置过小会导致边界点部分被剔除
                remove_discrete_points.filter (*mpCloudFilterOut);
                //程序运行过程中临时显示点云
                pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_z (mpCloudFilterOut, "z");
                viewer->setBackgroundColor (0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI> (mpCloudFilterOut, color_z,"target_points");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, this->mstruParam.mbGroundCloudViewerSize,"target_points");
                viewer->initCameraParameters ();
                while (!viewer->wasStopped ()) {
                        viewer->spinOnce ();
                }
        }
        else if (this->mstruParam.mbStatisticFlag) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZI> remove_discrete_points;
                remove_discrete_points.setInputCloud (mpCloudFilteredIn);
                remove_discrete_points.setMeanK (this->mstruParam.mnStatisticFilterKmeans);
                remove_discrete_points.setStddevMulThresh (this->mstruParam.mfStatisticFilterThre);
                remove_discrete_points.filter (*mpCloudFilterOut);
        }
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (this->mstruParam.mnRansacIters);
        seg.setDistanceThreshold (this->mstruParam.mfRansacDistanceThre);//设置点到模型的最大允许距离，用于决定一个点是否属于该模型。
        seg.setInputCloud (mpCloudFilterOut);
        seg.segment (*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud (mpCloudFilterOut);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*mpNoGroundPoints);
        if(1){
                pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_z (mpNoGroundPoints, "z");
                viewer->setBackgroundColor (0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZI> (mpNoGroundPoints, color_z,"target_points");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, this->mstruParam.mbGroundCloudViewerSize,"target_points");
                viewer->initCameraParameters ();
                while (!viewer->wasStopped ()) {
                        viewer->spinOnce ();
                }
        }
        //再次对离散点进行统计滤波
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> remove_outpoints;
        // remove_outpoints.setInputCloud (mpNoGroundPoints);
        // remove_outpoints.setMeanK (int (this->mstruParam.mnStatisticFilterKmeans / this->mstruParam.mnKmeansFactor));
        // remove_outpoints.setStddevMulThresh (this->mstruParam.mfStatisticFilterThre / this->mstruParam.mnStaticThredFactor);
        // remove_outpoints.filter (*mpNoGroundCloud);

        // if (this->mstruParam.mbGroundCloudViewerFlag) {
        //         pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        //         pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_z (mpNoGroundCloud, "z");
        //         viewer->setBackgroundColor (0, 0, 0);
        //         viewer->addPointCloud<pcl::PointXYZI> (mpNoGroundCloud, color_z,"target_points");
        //         viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, this->mstruParam.mbGroundCloudViewerSize,"target_points");
        //         viewer->initCameraParameters ();
        //         while (!viewer->wasStopped ()) {
        //                 viewer->spinOnce ();
        //         }
        // }
}
} 