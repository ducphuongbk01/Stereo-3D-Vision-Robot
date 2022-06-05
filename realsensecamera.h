#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H

#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>

#include <QThread>
#include <QImage>
#include <QMetaType>

#include "ObjectDetection.h"
#include "PointCloudCreator.h"
#include "convert.h"

typedef std::vector<cv::Mat> objPCMatInfo;

Q_DECLARE_METATYPE(objPCMatInfo)

class RealsenseCamera : public QThread
{
    Q_OBJECT

    private:
        BackgroundFilter *bgFilter;
        ObjectDetection *objDetector;
        AlgorithmPC *pcCreator;

        rs2::config cfg;
        rs2::pipeline pipe;
        int width, height;
        std::string config_path;
        bool isUpdate = false;
        bool isSavePC = false;
        bool inProcess = false;
        bool start_bb = true;

        void warm_up_camera();
        bool init();

    public:
        bool isRunning = false;

        cv::Mat pcCup_Mat, pcBottle_Mat;

        RealsenseCamera(int width, int height);
        RealsenseCamera(int width, int height, std::string config_path);
        ~RealsenseCamera();

        void loadCameraConfigPath(std::string config_path);

        bool initialize_bgFilter(std::string paramPath);
        void initialize_pcCreator(double minRange, double maxRange);
        void initialize_objDetector(std::string model_path, std::string class_list_path, int num_class, bool is_cuda,
                                    float score_threshold, float nms_threshold,float confidence_threshold, float classes_score_threshold);

        void updateRefFrame();
        void savePC();
        bool check_device_connected();
        void run();
        void stop();
        void startProcess();
        void stopProcess();
        void startBB();
        void stopBB();

    signals:
        void colorFrameReady(QImage color_frame);
        void refFrameReady(QImage ref_frame);
        void detectReady(QImage detectframe);
        void maskReady(QImage mask);
        void pcReady();
        void singlePCReady();
};

#endif // REALSENSECAMERA_H
