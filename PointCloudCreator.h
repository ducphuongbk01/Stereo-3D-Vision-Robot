#ifndef POINTCLOUDCREATOR_H
#define POINTCLOUDCREATOR_H

#include <iostream>
#include <algorithm> 
#include <string>
#include <vector>

// Intel Realsense Headers
#include <librealsense2/rs.hpp>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/opencv.hpp>

#include "ObjectDetection.h"
#include "convert.h"

#pragma once

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_pointer;
typedef std::vector<cv::Mat> objPCMatInfo;
typedef std::vector<std::vector<cv::Mat>> cloud_container_Mat;
typedef std::vector<std::tuple<cloud_pointer, cv::Mat>> cloud_container_PCL;

//---------------------------------------> Background Filter <---------------------------------------------

class BackgroundFilter
{
    private:
        int sizeKernelBlur;
        int threshValue;
        int sizeStructElementClosing;
        int iterationClosing;

        float thresh_diff;
        cv::Mat ref_frame;

        void closingMorphImgProc(cv::Mat &image, cv::Mat struct_element, int iter_dilate, int iter_erode);

    public:

        BackgroundFilter();
        BackgroundFilter(std::string paramPath);
        BackgroundFilter(int sizeKernelBlur, int threshValue, int sizeStructElementClosing, int iterationClosing);
        ~BackgroundFilter();

        bool loadParam(std::string paramPath);
        void updateRefFrame(const cv::Mat &frame);
        void startFilter(const cv::Mat &cur_frame, cv::Mat &mask);
};

//---------------------------------------> PointCloud Creator <---------------------------------------------

class PointCloudCreator
{
    protected:
        double minRange, maxRange;

        void frameFilter(rs2::frameset &frames);
        void pcFilter(const cloud_pointer cloud_in, cloud_pointer cloud_out);

    public:
        PointCloudCreator();
        PointCloudCreator(double minRange, double maxRange);
        virtual ~PointCloudCreator() = 0;
        
        void setDepthRange(double minRange, double maxRange);
        void load_PCDFile(cloud_pointer pointCloud, const std::string pc_path);
        void save_PCDFile(const cloud_pointer pointCloud, const std::string pc_path);
        void show(viewer_pointer cloudViewer, const cloud_pointer pointCloud, const std::string pcID);
        void clearPCID(viewer_pointer cloudViewer, const std::string pcID);
        void close(viewer_pointer cloudViewer);
        virtual void startPCProcess(cloud_pointer &pointCloud, rs2::frameset &frames) = 0;
        virtual void pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_Mat &pc_Object) = 0;
        virtual void pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_PCL &pc_Object) = 0;
        void pointCloudFilter(cloud_container_Mat &pc_Object_In, cloud_container_Mat &pc_Object_Out, uint32_t thresh_area, double thresh_conf);
        void pointCloudFilter(cloud_container_PCL &pc_Object_In, cloud_container_PCL &pc_Object_Out, uint32_t thresh_area, double thresh_conf);
        void objectFilter(const std::vector<Detection> &output_yolo, std::vector<Detection> &output_filter, const cv::Mat &mask, 
                        uint32_t thresh_area, double thresh_conf, bool chose_couple_best);    
};

class RealsensePC : public PointCloudCreator
{
    private:
        std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
        cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color);

    public:
        rs2::pointcloud pc;
        rs2::points points;

        RealsensePC();
        RealsensePC(double minRange, double maxRange);
        ~RealsensePC();

        void startPCProcess(cloud_pointer &pointCloud, rs2::frameset &frames);
        void pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_Mat &pc_Object);
        void pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_PCL &pc_Object);
};

class AlgorithmPC : public PointCloudCreator
{
    public:
        rs2_intrinsics intrinsic;

        AlgorithmPC();
        AlgorithmPC(double minRange, double maxRange);
        ~AlgorithmPC();

        void startPCProcess(cloud_pointer &pointCloud, rs2::frameset &frames);
        void pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_Mat &pc_Object);
        void pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_PCL &pc_Object);
};

#endif //POINTCLOUDCREATOR_H
