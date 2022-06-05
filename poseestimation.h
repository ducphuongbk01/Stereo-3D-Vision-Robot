#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/ppf_match_3d.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/t_hash_int.hpp>

#include <QThread>
#include <QMetaType>

#include <iostream>
#include <fstream>

#include "convert.h"

class PoseEstimation : public QThread
{
    Q_OBJECT
    private:
        std::string modelPath;
        cv::ppf_match_3d::PPF3DDetector *detector;

        double ppf_relative_sampling_step = 0.025;
        double ppf_relative_distance_step = 0.05;
        double ppf_num_angles = 30;

        double ppf_relative_scene_sample_step = 1.0/20.0;
        double ppf_relative_scene_distance = 0.05;

        int normal_num_neighbors = 7;
        bool normal_flip_viewpoint = true;
        cv::Vec3f normal_viewpoint = cv::Vec3f(0,0,0);

        int icp_iterations = 100;
        float icp_tolerence = 0.005;
        float icp_rejection_scale = 2.5;
        int icp_num_level = 5;
        int icp_sample_type = cv::ppf_match_3d::ICP::ICP_SAMPLING_TYPE_UNIFORM;
        int icp_num_max_corr = 1;

        bool isComplete = false;

    public:
        PoseEstimation();
        PoseEstimation(std::string modelPath,
                       double ppf_relative_sampling_step, double ppf_relative_distance_step, double ppf_num_angles,
                       double ppf_relative_scene_sample_step, double ppf_relative_scene_distance, int normal_num_neighbors,
                       int icp_iterations, float icp_tolerence, float icp_rejection_scale, int icp_num_level);
        ~PoseEstimation();

        void run();
        bool startTrain(const std::string modelPath,
                        const double ppf_relative_sampling_step, const double ppf_relative_distance_step, const double ppf_num_angles);
        bool estimatePose(const cv::Mat &pcEstimate, cv::ppf_match_3d::Pose3DPtr &result, cv::Mat &pcResult);
        bool caculateUsedPulsePose(const cv::ppf_match_3d::Pose3DPtr &result,
                                                                const cv::Matx44d c2b, const cv::Matx44d g2e,
                                                                std::vector<std::vector<int32_t>> &pulses);
};

#endif // POSEESTIMATION_H
