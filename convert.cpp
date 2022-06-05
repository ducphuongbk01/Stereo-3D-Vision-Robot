#include "convert.h"

void Convert::mat2Pcl(const cv::Mat& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, cv::Scalar color)
{
    pcl->points.resize(input.rows);
    for (int i=0;i<input.rows;i++)
    {
        pcl->points.at(i).x = input.at<float>(i,0);
        pcl->points.at(i).y = input.at<float>(i,1);
        pcl->points.at(i).z = input.at<float>(i,2);
        pcl->points.at(i).r = color[0];
        pcl->points.at(i).g = color[1];
        pcl->points.at(i).b = color[2];
    }
};


bool Convert::saveMatFile(const cv::Matx44d &mat, std::string filename)
{
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
    fs << "calib" << mat;
    fs.release();
    return true;
};


bool Convert::loadMatFile(cv::Matx44d &mat, std::string filename)
{
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    fs["calib"] >> mat;
    fs.release();
    return true;
};


bool Convert::saveMatFile(const cv::Mat&mat, std::string filename)
{
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
    fs << "calib" << mat;
    fs.release();
    return true;
};


bool Convert::loadMatFile(cv::Mat &mat, std::string filename)
{
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    fs["calib"] >> mat;
    fs.release();
    return true;
};


cv::Mat Convert::frame_to_mat(const rs2::frame& f)
{
    auto vf = f.as<rs2::video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == rs2_format::RS2_FORMAT_BGR8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == rs2_format::RS2_FORMAT_RGB8)
    {
        auto r_rgb = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat r_bgr;
        cv::cvtColor(r_rgb, r_bgr, cv::COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == rs2_format::RS2_FORMAT_Z16)
    {
        return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == rs2_format::RS2_FORMAT_Y8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == rs2_format::RS2_FORMAT_DISPARITY32)
    {
        return cv::Mat(cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else
    {
        throw std::runtime_error("Frame format is not supported yet!");
    }
};


cv::Mat Convert::depth_frame_to_meters( const rs2::depth_frame & f)
{
    cv::Mat dm = frame_to_mat(f);
    dm.convertTo( dm, CV_64F );
    dm = dm * f.get_units();
    return dm;
};



cv::Mat Convert::eulerAnglesToRotationMatrix(cv::Vec3f &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<float>(3,3) <<
               1,       0,               0,
               0,       cos(theta[0]),   sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<float>(3,3) <<
               cos(theta[1]),      0,      sin(theta[1]),
               0,                  1,      0,
               -sin(theta[1]),     0,      cos(theta[1])
               );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<float>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,                     0,              1
               );


    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;
    //Mat R = R_x * R_y * R_z;
    return R;
};


bool Convert::isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
            cv::transpose(R, Rt);
            cv::Mat shouldBeIdentity = Rt * R;
            cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

            return  cv::norm(I, shouldBeIdentity) < 1e-6;
};


cv::Vec3f Convert::rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(Convert::isRotationMatrix(R));

            float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

            bool singular = sy < 1e-6; // If

            float x, y, z;
            if (!singular)
            {
                x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
                y = atan2(-R.at<float>(2,0), sy);
                z = atan2(R.at<float>(1,0), R.at<float>(0,0));
            }
            else
            {
                x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
                y = atan2(-R.at<float>(2,0), sy);
                z = 0;
            }
            return cv::Vec3f(x, y, z);
};


void Convert::Rotate_and_Transmit_2_Pose(const cv::Mat& R, const cv::Mat& t, cv::Mat& Pose)
{
    cv::Matx34f P;
          cv::hconcat(R, t, P);
          cv::vconcat(P, cv::Matx14f(0, 0, 0, 1), Pose);
};



bool Convert::pulse2Joint(const std::vector<int32_t> &pulse,std::vector<double> &joint)
{
    joint.resize(6);
    if(pulse.size()<6) return false;
    joint[0] = pulse[0]/(34816/30.0)*M_PI/180;
    joint[1] = -pulse[1]/(102400/90.0)*M_PI/180;
    joint[2] = pulse[2]/(51200/90.0)*M_PI/180;
    joint[3] =  -pulse[3]/(10204/30.0)*M_PI/180;
    joint[4] =  pulse[4]/(10204/30.0)*M_PI/180;
    joint[5] =  -pulse[5]/(10204/30.0)*M_PI/180;
    return true;
};


bool Convert::joint2Pulse(const std::vector<double> &joint,std::vector<int32_t> &pulse)
{
    pulse.resize(6);
    if(joint.size()<6) return false;
    pulse[0] = round(joint[0]*(34816/30.0)/(M_PI/180));
    pulse[1] = round(joint[1]*(102400/90.0)/(M_PI/180));
    pulse[2] = round(joint[2]*(51200/90.0)/(M_PI/180));
    pulse[3] = round(joint[3]*(10204/30.0)/(M_PI/180));
    pulse[4] = round(joint[4]*(10204/30.0)/(M_PI/180));
    pulse[5] = round(joint[5]*(10204/30.0)/(M_PI/180));
    return true;
};

bool Convert::jointLimit(std::vector<double> joints)
{
    std::vector<double> joint_limit_up{170*M_PI/180,    90*M_PI/180,    90*M_PI/180,    140*M_PI/180,   210*M_PI/180,   360*M_PI/180}
                        ,joint_limit_down{-170*M_PI/180,    -85*M_PI/180,   -50*M_PI/180,   -140*M_PI/180,  -30*M_PI/180,   -360*M_PI/180};
    for (size_t i=0;i<6;i++)
    {
        if(joints.at(i)>joint_limit_up.at(i))
            return false;
        if(joints.at(i)<joint_limit_down.at(i))
            return false;
    }
    return true;
};


bool Convert::JointWorkspace(std::vector<float> joints)
{
    std::vector<double> joint_limit_up{170,    90,    90,    140,   210,   360}
                        ,joint_limit_down{-170,    -85,   -50,   -140,  -30,   -360};
    for (size_t i=0;i<6;i++)
    {
        if(joints.at(i)>joint_limit_up.at(i))
            return false;
        if(joints.at(i)<joint_limit_down.at(i))
            return false;
    }
    return true;
};

bool Convert::forwardKinematic(const std::vector<double> &joint, cv::Matx44d &pose)
{
    if(joint.size() < 6) return false;
    double a1 = 0.02;
    double a2 = 0.165;
    double d1 = 0.103;
    double d4 = 0.165;
    double d6 = 0.04;

    double t1 = joint[0];
    double t2 = joint[1]+M_PI/2;
    double t3 = joint[2];
    double t4 = joint[3];
    double t5 = joint[4]-M_PI/2;
    double t6 = joint[5];

    double r11 = sin(t6)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + cos(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)));
    double r12 = cos(t6)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)));
    double r13 = sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2));
    double px = a1*cos(t1) + d4*sin(t2 + t3)*cos(t1) + a2*cos(t1)*cos(t2) + d6*sin(t2 + t3)*cos(t1)*cos(t5) + d6*sin(t1)*sin(t4)*sin(t5) + d6*cos(t1)*cos(t2)*cos(t3)*cos(t4)*sin(t5) - d6*cos(t1)*cos(t4)*sin(t2)*sin(t3)*sin(t5);

    double r21 = -sin(t6)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) - cos(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)));
    double r22 = sin(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))) - cos(t6)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)));
    double r23 = cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)));
    double py = a1*sin(t1) + d4*sin(t2 + t3)*sin(t1) + a2*cos(t2)*sin(t1) + d6*sin(t2 + t3)*cos(t5)*sin(t1) - d6*cos(t1)*sin(t4)*sin(t5) + d6*cos(t2)*cos(t3)*cos(t4)*sin(t1)*sin(t5) - d6*cos(t4)*sin(t1)*sin(t2)*sin(t3)*sin(t5);

    double r31 = cos(t6)*(cos(t2 + t3)*sin(t5) + sin(t2 + t3)*cos(t4)*cos(t5)) - sin(t2 + t3)*sin(t4)*sin(t6);
    double r32 = -sin(t6)*(cos(t2 + t3)*sin(t5) + sin(t2 + t3)*cos(t4)*cos(t5)) - sin(t2 + t3)*cos(t6)*sin(t4);
    double r33 = sin(t2 + t3)*cos(t4)*sin(t5) - cos(t2 + t3)*cos(t5);
    double pz = d1 - d4*cos(t2 + t3) + a2*sin(t2) + (d6*sin(t2 + t3)*sin(t4 + t5))/2 - d6*cos(t2 + t3)*cos(t5) - (d6*sin(t4 - t5)*sin(t2 + t3))/2;

    cv::Matx44d result {r11, r12, r13, px,
                        r21, r22, r23, py,
                        r31, r32, r33, pz,
                        0  , 0  , 0  , 1};                                                                                                                                                        0,
    pose = result;
    return  true;
};

bool Convert::myforwardKinematic(const std::vector<double> &joint, cv::Matx44d &pose)
{
    if(joint.size() < 6) return false;
    double a1 = 0.02;
    double a2 = 0.165;
    double d1 = 0.103;
    double d4 = 0.165;
    double d6 = 0.04;

    double t1 = joint[0];
    double t2 = joint[1];
    double t3 = joint[2];
    double t4 = joint[3];
    double t5 = joint[4];
    double t6 = joint[5];

    cv::Matx44d result{   sin(t6)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + cos(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2))), cos(t6)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2))), sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)), a1*cos(t1) + d4*sin(t2 + t3)*cos(t1) + a2*cos(t1)*cos(t2) + d6*sin(t2 + t3)*cos(t1)*cos(t5) + d6*sin(t1)*sin(t4)*sin(t5) + d6*cos(t1)*cos(t2)*cos(t3)*cos(t4)*sin(t5) - d6*cos(t1)*cos(t4)*sin(t2)*sin(t3)*sin(t5),
     - sin(t6)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) - cos(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))), sin(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))) - cos(t6)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))), cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))), a1*sin(t1) + d4*sin(t2 + t3)*sin(t1) + a2*cos(t2)*sin(t1) + d6*sin(t2 + t3)*cos(t5)*sin(t1) - d6*cos(t1)*sin(t4)*sin(t5) + d6*cos(t2)*cos(t3)*cos(t4)*sin(t1)*sin(t5) - d6*cos(t4)*sin(t1)*sin(t2)*sin(t3)*sin(t5),
                                                                                                                                                                    cos(t6)*(cos(t2 + t3)*sin(t5) + sin(t2 + t3)*cos(t4)*cos(t5)) - sin(t2 + t3)*sin(t4)*sin(t6),                                                                                                                                                            - sin(t6)*(cos(t2 + t3)*sin(t5) + sin(t2 + t3)*cos(t4)*cos(t5)) - sin(t2 + t3)*cos(t6)*sin(t4),                                                                                                   sin(t2 + t3)*cos(t4)*sin(t5) - cos(t2 + t3)*cos(t5),                                                                                  d1 - d4*cos(t2 + t3) + a2*sin(t2) + (d6*sin(t2 + t3)*sin(t4 + t5))/2 - d6*cos(t2 + t3)*cos(t5) - (d6*sin(t4 - t5)*sin(t2 + t3))/2, 1};
                                                                                                                                                                                                                                                               0,                                                                                                                                                                                                                                                         0,                                                                                                                                                     0,
    pose = result;
    return  true;
};


bool Convert::inverseKinematic(const cv::Matx44d &pose, std::vector<double> &joint)
{
    double xc,yc,zc;
    xc = pose(0,3)-0.04*pose(0,2);
    yc = pose(1,3)-0.04*pose(1,2);
    zc = pose(2,3)-0.04*pose(2,2);

    double c3,s3;
    double c2,s2;

    double theta1[6], theta2[6], theta3[6], theta4[6];

    theta1[0]=atan2(yc,xc);
    theta2[0]=atan2(yc,xc);
    theta3[0]=atan2(yc,xc);
    theta4[0]=atan2(yc,xc);

    double tmp1 = sqrt(xc*xc+yc*yc);
    double tmp2 = zc-0.103;
    c3 = ((tmp1-0.02)*(tmp1-0.02) + tmp2*tmp2 - 0.165*0.165 - 0.165*0.165)/(2*0.165*0.165);
    double tmp3 = 0.165+0.165*c3;

    bool a1 = false;
    bool a2 = false;

    if(c3>=-1&&c3<=1)
    {
        // s3 < 0
        s3=-sqrt(1-c3*c3);

        theta3[2]=atan2(s3,c3);
        theta4[2]=atan2(s3,c3);

        c2=((tmp1-0.02)*tmp3+tmp2*0.165*s3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        s2=((-tmp1+0.02)*0.165*s3+tmp2*tmp3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
        {
            double tmp = atan2(s2, c2);
            if(-(tmp-M_PI/2) > 0 && -(tmp-M_PI/2) < M_PI/2)
            {
                theta3[1] = tmp;
                theta4[1] = tmp;
                a2=true;
            }
        }

        c2=((-tmp1-0.02)*tmp3+tmp2*0.165*s3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        s2=((tmp1+0.02)*0.165*s3+tmp2*tmp3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
        {
            double tmp = atan2(s2, c2);
            if(-(tmp-M_PI/2) > 0 && -(tmp-M_PI/2) < M_PI/2)
            {
                theta3[1] = tmp;
                theta4[1] = tmp;
                a2=true;
            }
        }

        // s3 > 0
        s3=sqrt(1-c3*c3);

        theta1[2]=atan2(s3,c3);
        theta2[2]=atan2(s3,c3);

        c2=((tmp1-0.02)*tmp3+tmp2*0.165*s3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        s2=((-tmp1+0.02)*0.165*s3+tmp2*tmp3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
        {
            double tmp = atan2(s2, c2);
            if(-(tmp-M_PI/2) > 0 && -(tmp-M_PI/2) < M_PI/2)
            {
                theta1[1] = tmp;
                theta2[1] = tmp;
                a1=true;
            }
        }

        c2=((-tmp1-0.02)*tmp3+tmp2*0.165*s3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        s2=((tmp1+0.02)*0.165*s3+tmp2*tmp3)/(0.165*0.165+0.165*0.165+2*0.165*0.165*c3);
        if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
        {
            double tmp = atan2(s2, c2);
            if(-(tmp-M_PI/2) > 0 && -(tmp-M_PI/2) < M_PI/2)
            {
                theta1[1] = tmp;
                theta2[1] = tmp;
                a1=true;
            }
        }
    }
    else
    {
        std::cout << "Cannot find IK" << std::endl;
        return false;
    }

    cv::Matx33d R_,R{pose(0,0),pose(0,1),pose(0,2),
              pose(1,0),pose(1,1),pose(1,2),
              pose(2,0),pose(2,1),pose(2,2)
              }, R_tmp;
    double s23, c23,c1,s1;
    //Joints caculate
    std::vector<double> joints1, joints2;
    joints1.resize(6);
    joints2.resize(6);
    std::vector<std::vector<double>> joints_contain;

    if(a1)
    {
        // ++ & +-
        s23 = sin(theta1[1] + theta1[2]);
        c23 = cos(theta1[1] + theta1[2]);
        c1 = cos(theta1[0]);
        s1 = sin(theta1[0]);
        R_(0,0)=-s23*c1;
        R_(0,1)=s1;
        R_(0,2)=c23*c1;

        R_(1,0)=-s23*s1;
        R_(1,1)=-c1;
        R_(1,2)=c23*s1;

        R_(2,0)=c23;
        R_(2,1)=0;
        R_(2,2)=s23;

        R_tmp = R_.t()*R;
        theta1[3] = atan2(R_tmp(1,2),R_tmp(0,2));
        theta1[4] = atan2(sqrt(R_tmp(0,2)*R_tmp(0,2)+R_tmp(1,2)*R_tmp(1,2)),R_tmp(2,2));
        theta1[5] = atan2(R_tmp(2,1),-R_tmp(2,0));

        theta2[3] = atan2(-R_tmp(1,2),-R_tmp(0,2));
        theta2[4] = atan2(-sqrt(R_tmp(0,2)*R_tmp(0,2)+R_tmp(1,2)*R_tmp(1,2)),R_tmp(2,2));
        theta2[5] = atan2(-R_tmp(2,1),+R_tmp(2,0));

        joints1.at(0)=theta1[0];
        joints1.at(1)=-(theta1[1]-M_PI/2);
        joints1.at(2) = (theta1[2]+M_PI/2);
        joints1.at(3)=-theta1[3];
        joints1.at(4)=(theta1[4]+M_PI/2);
        joints1.at(5) = -theta1[5];

        if(jointLimit(joints1))
        {
            joints_contain.push_back(joints1);
//            std::cout << "Joint 1: " << std::endl;
//            for (int i=0; i<6; i++)
//            {
//                std::cout << joints1.at(i)*180.0/M_PI << " " ;
//            }
//            std::cout << std::endl;

        }

        joints1.at(0)=theta2[0];
        joints1.at(1)=-(theta2[1]-M_PI/2);
        joints1.at(2) = (theta2[2]+M_PI/2);
        joints1.at(3)=-theta2[3];
        joints1.at(4)=(theta2[4]+M_PI/2);
        joints1.at(5) = -theta2[5];

        if(jointLimit(joints1))
        {
            joints_contain.push_back(joints1);
//            std::cout << "Joint 2: " << std::endl;
//            for (int i=0; i<6; i++)
//            {
//                std::cout << joints1.at(i)*180.0/M_PI << " " ;
//            }
//            std::cout << std::endl;
        }
    }

    if(a2)
    {
        // -+ & --
        s23 = sin(theta3[1] + theta3[2]);
        c23 = cos(theta3[1] + theta3[2]);
        c1 = cos(theta3[0]);
        s1 = sin(theta3[0]);
        R_(0,0)=-s23*c1;
        R_(0,1)=s1;
        R_(0,2)=c23*c1;

        R_(1,0)=-s23*s1;
        R_(1,1)=-c1;
        R_(1,2)=c23*s1;

        R_(2,0)=c23;
        R_(2,1)=0;
        R_(2,2)=s23;

        R_tmp = R_.t()*R;
        theta3[3] = atan2(R_tmp(1,2),R_tmp(0,2));
        theta3[4] = atan2(sqrt(R_tmp(0,2)*R_tmp(0,2)+R_tmp(1,2)*R_tmp(1,2)),R_tmp(2,2));
        theta3[5] = atan2(+R_tmp(2,1),-R_tmp(2,0));

        theta4[3] = atan2(-R_tmp(1,2),-R_tmp(0,2));
        theta4[4] = atan2(-sqrt(R_tmp(0,2)*R_tmp(0,2)+R_tmp(1,2)*R_tmp(1,2)),R_tmp(2,2));
        theta4[5] = atan2(-R_tmp(2,1),+R_tmp(2,0));

        joints1.at(0)=theta3[0];
        joints1.at(1)=-(theta3[1]-M_PI/2);
        joints1.at(2) = (theta3[2]+M_PI/2);
        joints1.at(3)=-theta3[3];
        joints1.at(4)=(theta3[4]+M_PI/2);
        joints1.at(5) = -theta3[5];

        if(jointLimit(joints1))
        {
            joints_contain.push_back(joints1);
//            std::cout << "Joint 3: " << std::endl;
//            for (int i=0; i<6; i++)
//            {
//                std::cout << joints1.at(i)*180.0/M_PI << " " ;
//            }
//            std::cout << std::endl;
        }

        joints1.at(0)=theta4[0];
        joints1.at(1)=-(theta4[1]-M_PI/2);
        joints1.at(2) = (theta4[2]+M_PI/2);
        joints1.at(3)=-theta4[3];
        joints1.at(4)=(theta4[4]+M_PI/2);
        joints1.at(5) = -theta4[5];

        if(jointLimit(joints1))
        {
            joints_contain.push_back(joints1);
//            std::cout << "Joint 4: " << std::endl;
//            for (int i=0; i<6; i++)
//            {
//                std::cout << joints1.at(i)*180.0/M_PI << " " ;
//            }
//            std::cout << std::endl;
        }
    }


    if(joints_contain.size()>0)
    {
        double diff = 0;
        double diff_tmp = 0;
        for(int i = 0; i<joints_contain.size(); i++)
        {
            if(i==0)
            {
                diff = sqrt(joints_contain.at(i).at(0)*joints_contain.at(i).at(0)+
                            joints_contain.at(i).at(1)*joints_contain.at(i).at(1)+
                            joints_contain.at(i).at(2)*joints_contain.at(i).at(2)+
                            5*joints_contain.at(i).at(3)*joints_contain.at(i).at(3)+
                            3*(joints_contain.at(i).at(4)-M_PI/2)*(joints_contain.at(i).at(4)-M_PI/2)+
                            joints_contain.at(i).at(5)*joints_contain.at(i).at(5));
                joints2.at(0)=joints_contain.at(i).at(0);
                joints2.at(1)=joints_contain.at(i).at(1);
                joints2.at(2)=joints_contain.at(i).at(2);
                joints2.at(3)=joints_contain.at(i).at(3);
                joints2.at(4)=joints_contain.at(i).at(4);
                joints2.at(5)=joints_contain.at(i).at(5);
            }
            else
            {
                diff_tmp = sqrt(joints_contain.at(i).at(0)*joints_contain.at(i).at(0)+
                                joints_contain.at(i).at(1)*joints_contain.at(i).at(1)+
                                joints_contain.at(i).at(2)*joints_contain.at(i).at(2)+
                                5*joints_contain.at(i).at(3)*joints_contain.at(i).at(3)+
                                3*(joints_contain.at(i).at(4)-M_PI/2)*(joints_contain.at(i).at(4)-M_PI/2)+
                                joints_contain.at(i).at(5)*joints_contain.at(i).at(5));
                if(diff > diff_tmp)
                {
                    diff = diff_tmp;
                    joints2.at(0)=joints_contain.at(i).at(0);
                    joints2.at(1)=joints_contain.at(i).at(1);
                    joints2.at(2)=joints_contain.at(i).at(2);
                    joints2.at(3)=joints_contain.at(i).at(3);
                    joints2.at(4)=joints_contain.at(i).at(4);
                    joints2.at(5)=joints_contain.at(i).at(5);
                }
            }
        }

        joint.resize(6);
        joint.at(0)=joints2.at(0);
        joint.at(1)=joints2.at(1);
        joint.at(2)=joints2.at(2);
        joint.at(3)=joints2.at(3);
        joint.at(4)=joints2.at(4);
        joint.at(5)=joints2.at(5);

        std::cout << "Joint : " << std::endl;
        for (int i=0; i<6; i++)
        {
            std::cout << joint.at(i)*180.0/M_PI << " " ;
        }
        std::cout << std::endl;

        return true;
    }
    else
    {
        std::cout << "No pose" << std::endl;
        return false;
    }
}

int Convert::Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_out)
{
    int ret = 0;

           //if (mat_in.type() != CV_32FC1 || mat_in.type() != CV_64FC1)
            if (mat_in.type() != CV_32FC1)
            {
                std::cout << "[HandEyeCalib] Mat input is not double floating-point number!" << std::endl;
                ret = 1;
                return ret;
            }

           for (int i=0; i<mat_in.rows; i++)
           {
                for (int j=0; j<mat_in.cols; j++)
                {
                    visp_out[i][j] = mat_in.ptr<float>(i)[j];  // new memory is created and data is copied in this line
                }
           }

    return ret;
};

int Convert::ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_out)
{
    int ret = 0;

            mat_out = cv::Mat::zeros(visp_in.getRows(),visp_in.getCols(),CV_32FC1);

            for (int i=0; i < mat_out.rows; i++)
            {
                for (int j=0; j < mat_out.cols; j++)
                {
                    mat_out.ptr<float>(i)[j] = visp_in[i][j];  // new memory is created and data is copied in this line
                }
            }

    return ret;
};

int Convert::ViSP2Matx(const vpHomogeneousMatrix& visp_in, cv::Matx44d& mat_out)
{
    cv::Mat mat,matd;
    ViSP2Mat(visp_in,mat);
    mat.convertTo(matd,CV_64F);
    cv::Matx44d temp(matd);
    mat_out = temp;
    return 0;
};

int Convert::Matx2ViSP( vpHomogeneousMatrix& visp_out,const cv::Matx44d& mat_in)
{
    int ret = 0;
    for (int i=0; i<mat_in.rows; i++)
    {
        for (int j=0; j<mat_in.cols; j++)
        {
            visp_out[i][j] = mat_in(i,j);  // new memory is created and data is copied in this line
        }
    }
    return ret;
};

void Convert::mat2eigen(const cv::Matx44d &pose, Eigen::Affine3f &eigen_pose)
{
    vpHomogeneousMatrix temp;
    cv::Mat dmat(pose);
    cv::Mat mat;
    dmat.convertTo(mat,CV_32F);
    Mat2ViSP(mat,temp);
    visp2eigen(temp,eigen_pose);
};

void Convert::visp2eigen(const vpHomogeneousMatrix& visp, Eigen::Affine3f &eigen_pose)
{
    Eigen::Matrix4d matrix_pose;
    vp::visp2eigen(visp,matrix_pose);
    eigen_pose.matrix() = matrix_pose.cast<float>();
};

QImage Convert::vispToQImage(const vpImage<vpRGBa> &f)
{
    auto r = QImage(reinterpret_cast<unsigned char *>(f.bitmap), f.getWidth(), f.getHeight(), f.getWidth()*4, QImage::Format_RGBA8888);
    return r;
};

cv::Mat Convert::qImageToMat(QImage img)
{
    QImage tmp = img.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(tmp.height(),tmp.width(),CV_8UC3,tmp.bits());
    return mat;
};

QImage Convert::matToQImage(cv::Mat &mat)
{
    return QImage((const unsigned char*)mat.data, mat.cols, mat.rows, QImage::Format_RGB888);
};
