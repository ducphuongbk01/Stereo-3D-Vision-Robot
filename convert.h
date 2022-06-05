#ifndef CONVERT_H
#define CONVERT_H

#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpEigenConversion.h>
#include <QImage>

#define CALIB_RADIANS 0
#define CALIB_DEGREES 1

namespace Convert
{
    void mat2Pcl(const cv::Mat& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, cv::Scalar color);
    bool saveMatFile(const cv::Matx44d &mat, std::string filename);
    bool loadMatFile(cv::Matx44d &mat, std::string filename);
    bool saveMatFile(const cv::Mat&mat, std::string filename);
    bool loadMatFile(cv::Mat &mat, std::string filename);
    cv::Mat frame_to_mat(const rs2::frame& f);
    cv::Mat depth_frame_to_meters( const rs2::depth_frame & f);

    cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta);
    bool isRotationMatrix(cv::Mat &R);
    cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
    void Rotate_and_Transmit_2_Pose(const cv::Mat& R, const cv::Mat& t, cv::Mat& Pose);

    bool pulse2Joint(const std::vector<int32_t> &pulse,std::vector<double> &joint);
    bool joint2Pulse(const std::vector<double> &joint,std::vector<int32_t> &pulse);
    bool jointLimit(std::vector<double> joints);
    bool JointWorkspace(std::vector<float> joints);
    bool forwardKinematic(const std::vector<double> &joint, cv::Matx44d &pose);
    bool myforwardKinematic(const std::vector<double> &joint, cv::Matx44d &pose);
    bool inverseKinematic(const cv::Matx44d &pose, std::vector<double> &joint);

    int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_out);
    int ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_out);
    int ViSP2Matx(const vpHomogeneousMatrix& visp_in, cv::Matx44d& mat_out);
    int Matx2ViSP( vpHomogeneousMatrix& visp_out,const cv::Matx44d& mat_in);

    void mat2eigen(const cv::Matx44d &pose, Eigen::Affine3f &eigen_pose);
    void visp2eigen(const vpHomogeneousMatrix& visp, Eigen::Affine3f &eigen_pose );

    QImage vispToQImage(const vpImage<vpRGBa> &f);
    cv::Mat qImageToMat(QImage img);
    QImage matToQImage(cv::Mat &mat);
};

#endif // CONVERT_H
