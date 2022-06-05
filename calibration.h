#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "convert.h"
#include "udp.h"
#include "controllib.h"

#include <QThread>
#include <QImage>
#include <QSlider>
#include <QMetaType>
#include <QString>
#include <QPlainTextEdit>

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

#include "udp.h"



Q_DECLARE_METATYPE(Eigen::Affine3f)

//--------------------------------> Color Calibration <------------------------------------

class ColorCalibration : public QThread
{
    Q_OBJECT

    private:
        rs2::config cfg;
        rs2::pipeline pipe;
        std::string cameraConfigPath;
        bool isRunning = false;
        bool inProgress = false;
        QPlainTextEdit *QPLainText;

        QSlider *slider_sizeKernelBlur;
        QSlider *slider_threshValue;
        QSlider *slider_sizeStructElementClosing;
        QSlider *slider_iterationClosing;

        void warm_up_Camera();
        bool checkDeviceConnected();
        void init();
        void closingMorphImgProc(cv::Mat &image, cv::Mat struct_element, int iter_dilate, int iter_erode);


    public:
        int width, height;
        int sizeKernelBlur;
        int threshValue;
        int sizeStructElementClosing;
        int iterationClosing;
        cv::Mat refFrame;

        ColorCalibration(int width, int height, std::string cameraConfigPath, QSlider *slider_sizeKernelBlur, QSlider *slider_threshValue,
                                                                            QSlider *slider_sizeStructElementClosing, QSlider *slider_iterationClosing); //, QPlainTextEdit *pte
        ~ColorCalibration();
        void run();
        void save(std::string cameraConfigDir);
        void stop();

    signals:
        void frameReady(QImage filter, QImage mask);
        void refFrameReady(QImage refImage);

};

//--------------------------------> Hand Eye Calibration <------------------------------------

class HandEyeCalibration : public QThread
{
    Q_OBJECT

    private:
        bool trigger = false;
        std::vector<vpHomogeneousMatrix> cMo;
        std::vector<vpHomogeneousMatrix> eMw;
        vpHomogeneousMatrix wMc,oMe;
        bool thread_stop = false;
        vpRealSense2 camera;
        std::string cameraParameterFile;
        int32_t pos[6];

    public:
        udp *motomanRobot;
        bool isReceivePositionFromRobot = false;
        cv::Matx44d w2c_transmit, o2e_transmit;


        HandEyeCalibration(std::string cameraParameterFile, udp *motomanRobot);
        ~HandEyeCalibration();
        void run();
        void stop();
        void startTrigger(int32_t *pulse);
        void receivedPosition(int32_t* pos);
        void caculatePose();
        void test();

    signals:
        // A signal sent by our class to notify that there are frames that need to be processed
        void framesReady(QImage frameRGB, QImage frameDepth);
        void finishCalibrate(Eigen::Affine3f aswer);
};

#endif // CALIBRATION_H
