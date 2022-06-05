#include "evaluatepose.h"
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

EvaluatePose::EvaluatePose(std::string cameraParameterFile, std::string savePath, cv::Matx44d tag2obj)
{
    this->cameraParameterFile = cameraParameterFile;
    this->savePath = savePath;
    this->tag2obj = tag2obj;
}

EvaluatePose::~EvaluatePose()
{

}

void EvaluatePose::run()
{
    this->thread_stop = false;

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

    double tagSize = 0.055;
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

            int idx=20;

            while (!this->thread_stop)
            {
                this->camera.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                                     nullptr, nullptr, &align_to_color);
                QImage tmp;
                tmp = Convert::vispToQImage(I_color);

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
                    double confidence_index;
                    if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], this->pose, &confidence_index))
                    {
                        if (confidence_index > 0.5)
                        {
                            vpDisplay::displayFrame(I_color2, this->pose, cam, tagSize/2, vpColor::none, 3);


                            cv::Matx44d matPose;
                            Convert::ViSP2Matx(this->pose, matPose);
                            this->pose_Mat = matPose*this->tag2obj;

                            if(this->trigger)
                            {
                                Convert::saveMatFile(this->pose_Mat, this->savePath + "/Pose_" + std::to_string(idx) + ".yml");
                                std::cout << "Added " + std::to_string(idx++) + " pose:"<< std::endl<< this->pose_Mat << std::endl;

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
            }
        }
        else
        {
            std::cout << "Cannot connect with camera, simulate." << std::endl;
        }

    }
    catch (const vpException &e)
    {
        std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
}

void EvaluatePose::stop()
{
    this->thread_stop=true;
}

void EvaluatePose::startTrigger()
{
    this->trigger = true;
}

