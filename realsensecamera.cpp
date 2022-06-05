#include "realsensecamera.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>
#include <QDebug>
#include <iomanip>

void RealsenseCamera::warm_up_camera()
{
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        frames = pipe.wait_for_frames();
    }
};

bool RealsenseCamera::init()
{
    try
    {
        if(this->check_device_connected())
        {
            this->cfg.enable_stream(RS2_STREAM_COLOR, this->width, this->height, RS2_FORMAT_RGB8, 30);
            this->cfg.enable_stream(RS2_STREAM_INFRARED, this->width, this->height, RS2_FORMAT_Y8, 30);
            this->cfg.enable_stream(RS2_STREAM_DEPTH, this->width, this->height, RS2_FORMAT_Z16, 30);

            rs2::pipeline_profile selection = this->pipe.start(cfg);
            rs2::device dev = selection.get_device();

            auto depth_sensor = dev.first<rs2::depth_sensor>();
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f);

            auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
            std::ifstream file(this->config_path);
            std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            advanced_mode_dev.load_json(str);

//            this->warm_up_camera();
            this->isRunning = true;
            return true;
        }
        else
        {
            this->isRunning = false;
            throw "Cannot connect to camera.";
        }
    }
    catch(const char* msg)
    {
        std::cerr << msg << std::endl;
        return false;
    }
};

RealsenseCamera::RealsenseCamera(int width, int height)
{
    this->width = width;
    this->height = height;
};

RealsenseCamera::RealsenseCamera(int width, int height, std::string config_path)
{
    this->width = width;
    this->height = height;
    this->config_path = config_path;
};

RealsenseCamera::~RealsenseCamera()
{

};

void RealsenseCamera::loadCameraConfigPath(std::string config_path)
{
    this->config_path = config_path;
};

bool RealsenseCamera::initialize_bgFilter(std::string paramPath)
{
    this->bgFilter = new BackgroundFilter(paramPath);
};

void RealsenseCamera::initialize_pcCreator(double minRange, double maxRange)
{
    this->pcCreator = new AlgorithmPC(minRange, maxRange);
};

void RealsenseCamera::initialize_objDetector(std::string model_path, std::string class_list_path, int num_class, bool is_cuda,
                            float score_threshold, float nms_threshold,float confidence_threshold, float classes_score_threshold)
{
    this->objDetector = new ObjectDetection(model_path, class_list_path, this->width, this->width, num_class, is_cuda,
                                            score_threshold, nms_threshold, confidence_threshold, classes_score_threshold);
};

bool RealsenseCamera::check_device_connected()
{
    bool tmp;
    rs2::context ctx;

    auto devices = ctx.query_devices();
    size_t device_count = devices.size();
    if (!device_count) tmp = false;
    else tmp = true;

    return tmp;
};

void RealsenseCamera::run()
{
    std::cout << "Configuration camera..." << std::endl;

    bool ret = this->init();
    if (!ret)
    {
        std::cout << "No device connected!" << std::endl;
        return;
    }

    std::cout << "Warm up camera..." << std::endl;
    this->warm_up_camera();

    std::cout << "Running..." << std::endl;

    rs2::frameset frames;
    rs2::frame color_frame, depth_frame, ir_frame;
    cv::Mat cv_color, cv_depth, cv_ir;
    cv::Mat cv_detect, cv_mask, cv_mask_RGB;

    QImage color_img, detect_img, mask_img;

    std::vector<Detection> outCheck, outYolo, outFilter;

    cloud_container_Mat pcCollection;

    std::cout << "Update reference frame for background filter..." << std::endl;

    do
    {
        frames = this->pipe.wait_for_frames();
        color_frame = frames.get_color_frame();
        cv_color = Convert::frame_to_mat(color_frame).clone();
        cv::cvtColor(cv_color, cv_color, cv::COLOR_BGR2RGB);

        this->objDetector->detect(cv_color, outCheck);
        cv_detect = this->objDetector->draw_bbox(cv_color, outCheck);
        detect_img = Convert::matToQImage(cv_detect);
        Q_EMIT detectReady(detect_img);
        if(outCheck.size()==0)
        {
            this->bgFilter->updateRefFrame(cv_color);
            color_img = Convert::matToQImage(cv_color);
            Q_EMIT refFrameReady(color_img);
        }
        else
        {
            std::cout << "Please put cup or bottle or can out of workspace..." << std::endl;
        }
    } while (outCheck.size()!=0);

    int64 start = 0;
    double fps = 0;
    int i = 20;

    bool chooseCouple = false;

    while(isRunning)
    {
        start = cv::getTickCount();

        if(!this->check_device_connected())
        {
            std::cout << "Camera was unpluged!" << std::endl;
            return;
        }

        frames = this->pipe.wait_for_frames();

        if(frames.get_data_size()!=614400) continue;

        color_frame = frames.get_color_frame();

        cv_color = Convert::frame_to_mat(color_frame);

        cv::cvtColor(cv_color, cv_color, cv::COLOR_BGR2RGB);
        color_img = Convert::matToQImage(cv_color);
        Q_EMIT colorFrameReady(color_img);

        if(this->start_bb)
        {
            outYolo.clear();
            outFilter.clear();
            pcCollection.clear();

            this->objDetector->detect(cv_color, outYolo);

            if(this->isUpdate)
            {
                if(outYolo.size()==0)
                {
                    this->bgFilter->updateRefFrame(cv_color);
                    this->isUpdate = false;
                }
                else
                {
                    std::cout << "Please put cup or bottle or can out of workspace..." << std::endl;
                }
                continue;
            }

            this->bgFilter->startFilter(cv_color, cv_mask);
            cv::cvtColor(cv_mask, cv_mask_RGB, cv::COLOR_GRAY2BGR);
            cv_mask_RGB = cv_mask_RGB & cv_color;
            mask_img = Convert::matToQImage(cv_mask_RGB);
            Q_EMIT maskReady(mask_img);

            this->pcCreator->objectFilter(outYolo, outFilter, cv_mask, 1000, 0.9, chooseCouple);

            cv_detect = this->objDetector->draw_bbox(cv_color, outFilter);

            if(this->inProcess)
            {
                qDebug() << "In Process";

                this->pcCreator->pcObjectCollection(frames, cv_mask, outFilter, pcCollection);
                std::cout << "Number of point cloud: " << pcCollection.size() << std::endl;

                if(pcCollection.size() > 0 && this->isSavePC)
                {
                    Convert::saveMatFile(pcCollection.at(0).at(0), "/home/phuongdoan/Code/ThesisGUI/PC_Collections/mypc" + std::to_string(i++) + ".yml");
                    std::cout << "Saved PC successfull" << std::endl;
                    this->isSavePC = false;
                }

                if(chooseCouple)
                {
                    if(outFilter.size()<2)
                    {
                        std::cout << "Cannot recognize exactly bottle and cup!" << std::endl;
                        continue;
                    }
                    this->pcCup_Mat = pcCollection.at(0).at(0);
                    this->pcBottle_Mat = pcCollection.at(1).at(0);
                    Q_EMIT pcReady();
                    ControlLib::delay_ms(50);
                }
                else
                {
                    if(outFilter.size()>0)
                    {
                        this->pcCup_Mat = pcCollection.at(0).at(0);
                        Convert::saveMatFile(this->pcCup_Mat, "/home/phuongdoan/Code/ThesisGUI/PC_Collections/mypc" + std::to_string(i++) + ".yml");
                        Q_EMIT singlePCReady();
                        qDebug() << "Signal Ready";
                        ControlLib::delay_ms(50);
                    }
                }
            }
        }

        else
        {
            cv_detect = cv_color.clone();
        }

        fps = cv::getTickFrequency()/(cv::getTickCount() - start);

        cv::putText(cv_detect, "FPS: " + std::to_string(fps), cv::Point(10,30),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,255),2,false);
        cv::Mat img_detect = cv_detect.clone();
        detect_img = Convert::matToQImage(img_detect);
        Q_EMIT detectReady(detect_img);
    }
}

void RealsenseCamera::updateRefFrame()
{
    this->isUpdate = true;
};

void RealsenseCamera::savePC()
{
    this->isSavePC = true;
};

void RealsenseCamera::stop()
{
    if(this->isRunning)
    {
        this->isRunning = false;
        ControlLib::delay_ms(100);
        this->pipe.stop();
    }
};

void RealsenseCamera::startProcess()
{
    this->inProcess=true;
}

void RealsenseCamera::stopProcess()
{
    this->inProcess=false;
}

void RealsenseCamera::startBB()
{
    this->start_bb = true;
}

void RealsenseCamera::stopBB()
{
    this->start_bb = false;
}
