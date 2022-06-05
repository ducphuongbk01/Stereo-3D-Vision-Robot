#ifndef EVALUATEPOSE_H
#define EVALUATEPOSE_H

#include "convert.h"

#include <QThread>
#include <QMetaType>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/core/vpEigenConversion.h>

#include <eigen3/Eigen/Core>

class EvaluatePose : public QThread
{
    Q_OBJECT

private:
    bool trigger = false;
    bool thread_stop = false;
    vpHomogeneousMatrix pose;
    cv::Matx44d pose_Mat, tag2obj;
    vpRealSense2 camera;
    std::string cameraParameterFile;
    std::string savePath;

public:
    EvaluatePose(std::string cameraParameterFile, std::string savePath, cv::Matx44d tag2obj);
    ~EvaluatePose();
    void run();
    void stop();
    void startTrigger();
};

#endif // EVALUATEPOSE_H
