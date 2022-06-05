#include "calibration.h"
#include "controllib.h"
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>
#include <QHostAddress>
#include <QDebug>
#include <QPlainTextEdit>

//--------------------------------> Color Calibration <------------------------------------

void ColorCalibration::warm_up_Camera()
{
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        frames = this->pipe.wait_for_frames();
    }
};

bool ColorCalibration::checkDeviceConnected()
{
    bool tmp;
    rs2::context ctx;

    auto devices = ctx.query_devices();
    size_t device_count = devices.size();
    if (!device_count) tmp = false;
    else tmp = true;

    return tmp;
};

void ColorCalibration::init()
{
    this->cfg.enable_stream(RS2_STREAM_COLOR, this->width, this->height, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile selection = this->pipe.start(this->cfg);
    rs2::device dev = selection.get_device();
    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
    std::ifstream file(this->cameraConfigPath);
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    advanced_mode_dev.load_json(str);

    this->warm_up_Camera();

    this->isRunning = true;
};

void ColorCalibration::closingMorphImgProc(cv::Mat &image, cv::Mat struct_element, int iter_dilate, int iter_erode)
{
    cv::dilate(image, image, struct_element, cv::Point(-1, -1), iter_dilate);
    cv::erode(image, image, struct_element, cv::Point(-1, -1), iter_erode);
};

ColorCalibration::ColorCalibration(int width, int height, std::string cameraConfigPath, QSlider *slider_sizeKernelBlur, QSlider *slider_threshValue,
                                        QSlider *slider_sizeStructElementClosing, QSlider *slider_iterationClosing)
{
    this->width = width;
    this->height = height;
    this->cameraConfigPath = cameraConfigPath;
    this->slider_sizeKernelBlur = slider_sizeKernelBlur;
    this->slider_threshValue = slider_threshValue;
    this->slider_sizeStructElementClosing = slider_sizeStructElementClosing;
    this->slider_iterationClosing = slider_iterationClosing;

    QPLainText = new QPlainTextEdit;
};

ColorCalibration::~ColorCalibration()
{

};

void ColorCalibration::run()
{
    if(!this->checkDeviceConnected())
    {
        std::cout << "No device connected" << std::endl;
//        ControlLib::AppOutput("No device connected", QPLainText);
        return;
    }

    this->init();

    int sizeKernelBlur_tmp;
    int threshValue_tmp;
    int sizeStructElementClosing_tmp;
    int iterationClosing_tmp;

    rs2::frameset frames;
    rs2::frame color_frame;
    cv::Mat ref_frame_tmp, gray_frame_tmp, gray_ref_frame_tmp, filter_frame_tmp, cv_color_tmp;
    cv::Mat diff_frame;
    cv::Mat thresh_frame;

    QImage refImage, threshFrame, threshFilter;

    std::cout << "Warm up camera..." << std::endl;

//    ControlLib::AppOutput("Warm up camera...", QPLainText);

    for(int i =0; i < 30; ++i)
    {
        frames = this->pipe.wait_for_frames();
    };

    frames = this->pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    ref_frame_tmp = Convert::frame_to_mat(color_frame).clone();
    cv::cvtColor(ref_frame_tmp, ref_frame_tmp, cv::COLOR_BGR2RGB);

    refImage = Convert::matToQImage(ref_frame_tmp);
    Q_EMIT refFrameReady(refImage);

    std::cout << "Calibration starting..." << std::endl;
    while(this->isRunning)
    {
        this->inProgress = true;

        sizeKernelBlur_tmp = 2*(int)this->slider_sizeKernelBlur->value()+1;
        threshValue_tmp = (int)this->slider_threshValue->value();
        sizeStructElementClosing_tmp = 2*(int)this->slider_sizeStructElementClosing->value()+1;
        iterationClosing_tmp = (int)this->slider_iterationClosing->value();

        frames = this->pipe.wait_for_frames();
        color_frame = frames.get_color_frame();

        cv::Mat cv_color(cv::Size(this->width, this->height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        cv::cvtColor(ref_frame_tmp, gray_ref_frame_tmp, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_color, gray_frame_tmp, cv::COLOR_BGR2GRAY);

        cv::GaussianBlur(gray_ref_frame_tmp, gray_ref_frame_tmp, cv::Size(sizeKernelBlur_tmp, sizeKernelBlur_tmp), 0);
        cv::GaussianBlur(gray_frame_tmp, gray_frame_tmp, cv::Size(sizeKernelBlur_tmp, sizeKernelBlur_tmp), 0);

        cv::absdiff(gray_ref_frame_tmp, gray_frame_tmp, diff_frame);

        cv::threshold(diff_frame, thresh_frame, threshValue_tmp, 255.0, cv::THRESH_BINARY);

        cv::Mat struct_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(sizeStructElementClosing_tmp, sizeStructElementClosing_tmp));
        this->closingMorphImgProc(thresh_frame, struct_element, iterationClosing_tmp, iterationClosing_tmp);

        cv::cvtColor(thresh_frame, thresh_frame, cv::COLOR_GRAY2RGB);
        cv::cvtColor(cv_color, cv_color, cv::COLOR_BGR2RGB);

        cv::Mat thresh_filter = cv_color & thresh_frame;


        threshFrame = Convert::matToQImage(thresh_frame);
        threshFilter = Convert::matToQImage(thresh_filter);
        Q_EMIT frameReady(threshFilter, threshFrame);
        this->inProgress = false;
    };
};

void ColorCalibration::save(std::string cameraConfigDir)
{
    this->sizeKernelBlur = 2*(int)this->slider_sizeKernelBlur->value() + 1;
    this->threshValue  = (int)this->slider_threshValue->value();
    this->sizeStructElementClosing = 2*(int)this->slider_sizeStructElementClosing->value() + 1;
    this->iterationClosing = (int)this->slider_iterationClosing->value();

    std::string save_path = cameraConfigDir + "/Color_Calib.xml";
    cv::FileStorage fs(save_path.c_str(), cv::FileStorage::WRITE);
    fs << "SizeKernelBlur" << this->sizeKernelBlur;
    fs << "ThreshValue" << this->threshValue;
    fs << "SizeStructElementClosing" << this->sizeStructElementClosing;
    fs << "IterationClosing" << this->iterationClosing;
    fs.release();
};

void ColorCalibration::stop()
{
    if(this->isRunning)
    {
        this->isRunning = false;
        this->pipe.stop();
    }
    else
    {
        std::cout << "Camera was already stopped." << std::endl;
    }

};

//--------------------------------> Hand Eye Calibration <------------------------------------

HandEyeCalibration::HandEyeCalibration(std::string cameraParameterFile, udp *motomanRobot)
{
    this->cameraParameterFile = cameraParameterFile;
    this->motomanRobot = motomanRobot;
};

HandEyeCalibration::~HandEyeCalibration()
{

};

void HandEyeCalibration::run()
{
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

    double tagSize = 0.063;
    float quad_decimate = 1.0;
    int nThreads = 1;
    bool display_tag = false;
    int color_id = -1;
    unsigned int thickness = 2;
    bool align_frame = false;
    bool display_off = false;

    try
    {
        std::cout << "Use Realsense 2 grabber" << std::endl;

        rs2::config config;
        unsigned int width = 640, height = 480;
//        config.disable_stream(RS2_STREAM_DEPTH);
//        config.disable_stream(RS2_STREAM_INFRARED);
        config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_RGBA8, 30);
        config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Z16, 30);

        vpImage<unsigned char> I;
        vpImage<vpRGBa> I_color(height, width);
        vpImage<uint16_t> I_depth_raw(height, width);
        vpImage<vpRGBa> I_depth;

        cv::Mat image_color, image_depth;
        QImage img_color, img_depth;

        rs2::context ctx;
        auto list = ctx.query_devices();

        if(list.size() > 0)
        {
            this->camera.open(config);
            auto prof = this->camera.getPipeline().get_active_profile();
            rs2::device dev = prof.get_device();
            auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
            std::ifstream file(this->cameraParameterFile);
            std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            advanced_mode_dev.load_json(str);

            const float depth_scale = this->camera.getDepthScale();
            rs2::align align_to_color = RS2_STREAM_COLOR;
            this->camera.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                        nullptr, nullptr, &align_to_color);

            std::cout << "Read camera parameters from Realsense device" << std::endl;
            vpCameraParameters cam;
            cam = this->camera.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);

            std::cout << cam << std::endl;
            std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
            std::cout << "tagFamily: " << tagFamily << std::endl;
            std::cout << "nThreads : " << nThreads << std::endl;
            std::cout << "Z aligned: " << align_frame << std::endl;

            vpImage<vpRGBa> I_color2 = I_color;
            vpImage<float> depthMap;
            vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

            vpDisplay *d1 = NULL;
            vpDisplay *d2 = NULL;
            vpDisplay *d3 = NULL;
            if (!display_off)
            {
                #ifdef VISP_HAVE_X11
                        d1 = new vpDisplayX(I_color, 100, 30, "Pose from Homography");
                        d2 = new vpDisplayX(I_color2, I_color.getWidth()+120, 30, "Pose from RGBD fusion");
                        d3 = new vpDisplayX(I_depth, 100, I_color.getHeight()+70, "Depth");
                #elif defined(VISP_HAVE_GDI)
                        d1 = new vpDisplayGDI(I_color, 100, 30, "Pose from Homography");
                        d2 = new vpDisplayGDI(I_color2, I_color.getWidth()+120, 30, "Pose from RGBD fusion");
                        d3 = new vpDisplayGDI(I_depth, 100, I_color.getHeight()+70, "Depth");
                #elif defined(VISP_HAVE_OPENCV)
                        d1 = new vpDisplayOpenCV(I_color, 100, 30, "Pose from Homography");
                        d2 = new vpDisplayOpenCV(I_color2, I_color.getWidth()+120, 30, "Pose from RGBD fusion");
                        d3 = new vpDisplayOpenCV(I_depth, 100, I_color.getHeight()+70, "Depth");
                #endif
            }

            vpDetectorAprilTag detector(tagFamily);

            detector.setAprilTagQuadDecimate(quad_decimate);
            detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
            detector.setAprilTagNbThreads(nThreads);
            detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
            detector.setZAlignedWithCameraAxis(align_frame);

            while (!this->thread_stop)
            {
                this->camera.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                                     nullptr, nullptr, &align_to_color);
                QImage tmp;
                tmp = Convert::vispToQImage(I_color);
                Q_EMIT framesReady(tmp,tmp);

                vpImageConvert::convert(I_color, I);
                I_color2 = I_color;
                vpImageConvert::convert(I_color, I);
                vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

                vpDisplay::display(I_color);
                vpDisplay::display(I_color2);
                vpDisplay::display(I_depth);

                depthMap.resize(I_depth_raw.getHeight(), I_depth_raw.getWidth());
                for (unsigned int i = 0; i < I_depth_raw.getHeight(); i++)
                {
                    for (unsigned int j = 0; j < I_depth_raw.getWidth(); j++)
                    {
                        if (I_depth_raw[i][j])
                        {
                            float Z = I_depth_raw[i][j] * depth_scale;
                            depthMap[i][j] = Z;
                        }
                        else
                        {
                            depthMap[i][j] = 0;
                        }
                    }
                }

                vpDisplay::display(I_color);
                vpDisplay::display(I_color2);
                vpDisplay::display(I_depth);


                //std::cout << "Finding..." << std::endl;

                std::vector<vpHomogeneousMatrix> cMo_vec;
                detector.detect(I, tagSize, cam, cMo_vec);

                // Display camera pose for each tag
                for (size_t i = 0; i < cMo_vec.size(); i++)
                {
                    vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
                }

                std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
                std::vector<int> tags_id = detector.getTagsId();
                std::map<int, double> tags_size;
                tags_size[-1] = tagSize; // Default tag size
                std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);

                for (size_t i = 0; i < tags_corners.size(); i++)
                {
                    vpHomogeneousMatrix cMo_tmp;
                    double confidence_index;
                    if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo_tmp, &confidence_index))
                    {
                        if (confidence_index > 0.5)
                        {
                            vpDisplay::displayFrame(I_color2, cMo_tmp, cam, tagSize/2, vpColor::none, 3);

                            if(this->trigger)
                            {
                                int32_t pos_2[6];
                                if(this->motomanRobot->GetPulsePos(pos_2))
                                {
                                    receivedPosition(pos_2);
                                    this->cMo.push_back(cMo_tmp);
                                    std::cout << "Added cMo_tmp pose "<< std::endl<< cMo_tmp << std::endl;
                                    this->isReceivePositionFromRobot = false;
                                }
                                else
                                {
                                    std::cout << "Cannot connect to controller, please check again! " << std::endl;
                                }
                                this->trigger = false;
                            }
                        }
                        else
                        {
                            std::cout << "Fail" << std::endl;
                        }

                    }
                }
                vpDisplay::flush(I_color);
                vpDisplay::flush(I_color2);
                vpDisplay::flush(I_depth);

//                vpImageConvert::convert(I_color, image_color);
//                vpImageConvert::convert(I_depth, image_depth);
//                cv::cvtColor(image_color, image_color, cv::COLOR_BGR2RGB);
//                cv::cvtColor(image_depth, image_depth, cv::COLOR_BGR2RGB);
//                img_color = Convert::matToQImage(image_color);
//                img_depth = Convert::matToQImage(image_depth);
//                Q_EMIT framesReady(img_color, img_depth);
            }
        }
        else
        {
            std::cout << "Cannot connect with camera, simulate." << std::endl;
            while(!this->thread_stop)
            {
                if(this->trigger)
                {
                    this->trigger = false;
                    this->receivedPosition(this->pos);
                }

            }
        }

    }
    catch (const vpException &e)
    {
        std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
};

void HandEyeCalibration::stop()
{
    this->thread_stop = true;
};

void HandEyeCalibration::startTrigger(int32_t *pulse)
{
    memcpy(this->pos, pulse, 6);
    this->trigger = true;
}

void HandEyeCalibration::receivedPosition(int32_t* pos)
{
    float position[6];
    std::vector<int32_t> pulse(6);
    std::vector<double> joint(6);
    std::memcpy(pulse.data(),pos,6*4);
    std::cout << "pulse " << pulse[5];
    Convert::pulse2Joint(pulse,joint);
    cv::Matx44d pose;
    Convert::forwardKinematic(joint,pose);
    std::cout << "wMe pose " << std::endl << pose << std::endl;
    pose = pose.inv();


    vpHomogeneousMatrix pose_vp;
    Convert::Matx2ViSP(pose_vp,pose);

    this->eMw.push_back(pose_vp);

    this->isReceivePositionFromRobot = true;
};

void HandEyeCalibration::caculatePose()
{
    if(this->eMw.size()>=3 && this->cMo.size()>=3)
    {
        if(vpHandEyeCalibration::calibrate(this->cMo, this->eMw, this->wMc) == 0)
        {
            std::cout << "***VISP***" << std::endl;
            std::cout << this->wMc << std::endl;
            Eigen::Affine3f eigen_pose_cam2base;
            Eigen::Matrix4d matrix_pose_cam2base;
            vp::visp2eigen(this->wMc, matrix_pose_cam2base);
            eigen_pose_cam2base.matrix() = matrix_pose_cam2base.cast<float>();

            Q_EMIT finishCalibrate(eigen_pose_cam2base);

            Convert::ViSP2Matx(this->wMc, this->w2c_transmit);
        }
        else
        {
            std::cout << "Fail World to Camera Calibration." << std::endl;
        }

        if(vpHandEyeCalibration::calibrate(this->eMw, this->cMo, this->oMe)==0)
        {
            std::cout << " Result oMe: " << std::endl << this->oMe << std::endl;
            Convert::ViSP2Matx(this->oMe, this->o2e_transmit);
        }
        else
        {
            std::cout << " Fail Object to End-effector Calibration." << std::endl;
        }
    }
    else
    {
        std::cout << "Please get more pose." << std::endl;
    }
};

void HandEyeCalibration::test()
{
    std::vector<vpHomogeneousMatrix> a;
    vpHomogeneousMatrix wMc_tmp, wMc_true(1.2, -1.3, 1.4, -M_PI_4, M_PI_4, -M_PI_4),
                              oMe_true(0.2, 0.3, 0.4, -M_PI/9, M_PI_4/2, -M_PI_4/3);

    std::cout << "wMc_true: " << std::endl << wMc_true << std::endl;
    std::cout << "oMe_true: " << std::endl << oMe_true << std::endl;

    vpHomogeneousMatrix base2gripper(1, -1, 1, -M_PI_4/2, M_PI_4/5, -M_PI_4/7);
    this->eMw.push_back(base2gripper);
    this->cMo.push_back(wMc_true.inverse()*base2gripper.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper1(0, -1, 1, M_PI_4/3, M_PI_4/7, M_PI_4/2);
    this->eMw.push_back(base2gripper1);
    this->cMo.push_back(wMc_true.inverse()*base2gripper1.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper2(0, 1.2, -3, M_PI_2/3, -M_PI_4/7, -M_PI_2/5);
    this->eMw.push_back(base2gripper2);
    this->cMo.push_back(wMc_true.inverse()*base2gripper2.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper3(-0.9, -1.2, -0.1, M_PI_2/9, M_PI_4/15, M_PI_2/25);
    this->eMw.push_back(base2gripper3);
    this->cMo.push_back(wMc_true.inverse()*base2gripper3.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper4(1.9, 0.2, 5.1, M_PI_2/1, -M_PI_4/4, -M_PI_2/3);
    this->eMw.push_back(base2gripper4);
    this->cMo.push_back(wMc_true.inverse()*base2gripper4.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper5(2, 1.12, 0.1, -M_PI_2/4, -M_PI_4/5, -M_PI_2/25);
    this->eMw.push_back(base2gripper5);
    this->cMo.push_back(wMc_true.inverse()*base2gripper5.inverse()*oMe_true.inverse());

    if(vpHandEyeCalibration::calibrate(this->cMo, this->eMw, this->wMc)==0)
    {
        std::cout << " Result wMc: " << std::endl << this->wMc << std::endl;
    }
    else
    {
        std::cout << " Fail World to Camera Calibration Test." << std::endl;
    }
};
