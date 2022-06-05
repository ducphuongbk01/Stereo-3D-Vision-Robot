#include "poseestimation.h"
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <QDebug>

std::string getTrainFolder(const std::string& fname)
{
     size_t pos = fname.find_last_of("\\.");
     return (std::string::npos == pos) ? "" : fname.substr(0, pos);
}

//--------------------------------------> Pose Estimation <-------------------------------------------

PoseEstimation::PoseEstimation()
{

}


PoseEstimation::PoseEstimation(std::string modelPath,
                               double ppf_relative_sampling_step, double ppf_relative_distance_step, double ppf_num_angles,
                               double ppf_relative_scene_sample_step, double ppf_relative_scene_distance, int normal_num_neighbors,
                               int icp_iterations, float icp_tolerence, float icp_rejection_scale, int icp_num_level)
{
    this->modelPath = modelPath;
    this->ppf_relative_sampling_step = ppf_relative_sampling_step;
    this->ppf_relative_distance_step = ppf_relative_distance_step;
    this->ppf_num_angles = ppf_num_angles;
    this->ppf_relative_scene_sample_step = ppf_relative_scene_sample_step;
    this->ppf_relative_scene_distance = ppf_relative_scene_distance;
    this->normal_num_neighbors = normal_num_neighbors;
    this->icp_iterations = icp_iterations;
    this->icp_tolerence = icp_tolerence;
    this->icp_rejection_scale = icp_rejection_scale;
    this->icp_num_level = icp_num_level;
}

PoseEstimation::~PoseEstimation()
{
    this->detector->~PPF3DDetector();
}


bool PoseEstimation::estimatePose(const cv::Mat &pcEstimate, cv::ppf_match_3d::Pose3DPtr &result, cv::Mat &pcResult)
{
    if(this->isComplete)
    {
        std::cout << "Start matching ...";
        cv::Mat pc_normal;
        int64 tick1, tick2;
        tick1 = cv::getTickCount();
        cv::ppf_match_3d::computeNormalsPC3d(pcEstimate, pc_normal, this->normal_num_neighbors, this->normal_flip_viewpoint, this->normal_viewpoint);
        tick2 = cv::getTickCount();
        std::cout << std::endl << "Normal Estimation Time " << (tick2-tick1)/cv::getTickFrequency() << " s" << std::endl;
        tick1 = cv::getTickCount();
        std::vector<cv::ppf_match_3d::Pose3DPtr> results;
        this->detector->match(pc_normal, results, this->ppf_relative_scene_sample_step, this->ppf_relative_scene_distance);
        tick2 = cv::getTickCount();
        std::cout << std::endl << "PPF Elapsed Time " << (tick2-tick1)/cv::getTickFrequency() << " s" << std::endl;

        //check results size from match call above
        size_t results_size = results.size();
        //cout << "Number of matching poses: " << results_size;
        if (results_size == 0)
        {
            std::cout << std::endl << "No matching poses found. Exiting." << std::endl;
            return false;
        }

        // Get only first N results - but adjust to results size if num of results are less than that specified by N
        size_t N = 2;
        if (results_size < N)
        {
            std::cout << std::endl << "Reducing matching poses to be reported (as specified in code): "<< N
                        << " to the number of matches found: " << results_size << std::endl;
            N = results_size;
        }
        std::vector<cv::ppf_match_3d::Pose3DPtr> resultsSub(results.begin(),results.begin()+N);

        // Create an instance of ICP
        cv::ppf_match_3d::ICP icp(this->icp_iterations, this->icp_tolerence, this->icp_rejection_scale, this->icp_num_level, this->icp_sample_type, this->icp_num_max_corr);
        int64 t1 = cv::getTickCount();

        // Register for all selected poses
        //cout << endl << "Performing ICP on " << N << " poses..." << endl;
        icp.registerModelToScene(this->detector->model, pc_normal, resultsSub);
        int64 t2 = cv::getTickCount();

        std::cout << std::endl << "ICP Elapsed Time " << (t2-t1)/cv::getTickFrequency() << " s" << std::endl;
        //cout << endl << "There are " << resultsSub.size() << " poses" << endl;
        pcResult = cv::ppf_match_3d::transformPCPose(this->detector->model, resultsSub.at(0)->pose);

        std::cout << "Pose: " << resultsSub.at(0)->pose << std::endl;
        std::cout << "NumVotes: " << resultsSub.at(0)->numVotes << std::endl;
        std::cout << "Residual: " << resultsSub.at(0)->residual << std::endl;

        if(resultsSub.at(0)->residual>1||resultsSub.at(0)->numVotes<1000)
        {
            std::cout << "Cannot caculate good pose!" << std::endl;
            return false;
        }

        result = resultsSub.at(0);
        return true;
    }
    else
    {
        std::cout << "Please train before estimate!" << std::endl;
    }

}

bool PoseEstimation::caculateUsedPulsePose(const cv::ppf_match_3d::Pose3DPtr &result,
                                           const cv::Matx44d c2b, const cv::Matx44d g2e,
                                           std::vector<std::vector<int32_t>> &pulses)
{
    cv:: Matx44d oMg, rot, tMb_ready, tMb_pick;
    double alpha = 0;
    bool ret = false;

    std::vector<double> jointsReady_tmp,jointsReady, jointsPick, jointsCurPos;
    std::vector<int32_t> pulseReady,pulsePick,pulseCurPos;
    double diff, diff_tmp;
    bool k = false;

    for(int i = 0; i < 36; i++)
    {
        alpha = i*5*M_PI/180;
        rot = {1, 0, 0, 0,
              0, cos(alpha), cos(M_PI_2+alpha), 0,
              0, cos(M_PI_2-alpha), cos(alpha), 0,
              0, 0, 0, 1};
//        oMg = c2b*result->pose*rot*cv::Matx44d{1, 0, 0, 0.10,
//                                               0, 1, 0, 0,
//                                               0, 0, 1, -0.145,
//                                               0, 0, 0, 1};

        oMg = c2b*result->pose*rot*cv::Matx44d{1, 0, 0, 0.03,
                                               0, 1, 0, 0,
                                               0, 0, 1, -0.16,
                                               0, 0, 0, 1};
        ret = Convert::inverseKinematic(oMg, jointsReady_tmp);
        if(ret)
        {
            if(!k)
            {
//                diff = sqrt(2*jointsReady_tmp.at(3)*jointsReady_tmp.at(3)+
//                            jointsReady_tmp.at(5)*jointsReady_tmp.at(5));

                diff = sqrt(jointsReady_tmp.at(0)*jointsReady_tmp.at(0)+
                            jointsReady_tmp.at(1)*jointsReady_tmp.at(1)+
                            jointsReady_tmp.at(2)*jointsReady_tmp.at(2)+
                            5*jointsReady_tmp.at(3)*jointsReady_tmp.at(3)+
                            3*(jointsReady_tmp.at(4)-M_PI/2)*(jointsReady_tmp.at(4)-M_PI/2)+
                            jointsReady_tmp.at(5)*jointsReady_tmp.at(5));

                tMb_ready = {oMg.col(0).row(0).val[0], oMg.col(1).row(0).val[0], oMg.col(2).row(0).val[0], oMg.col(3).row(0).val[0],
                            oMg.col(0).row(1).val[0], oMg.col(1).row(1).val[0], oMg.col(2).row(1).val[0], oMg.col(3).row(1).val[0],
                            oMg.col(0).row(2).val[0], oMg.col(1).row(2).val[0], oMg.col(2).row(2).val[0], oMg.col(3).row(2).val[0],
                            oMg.col(0).row(3).val[0], oMg.col(1).row(3).val[0], oMg.col(2).row(3).val[0], oMg.col(3).row(3).val[0]};
                jointsReady = jointsReady_tmp;
                k=true;
            }
            else
            {
//                diff_tmp = sqrt(2*jointsReady_tmp.at(3)*jointsReady_tmp.at(3)+
//                                jointsReady_tmp.at(5)*jointsReady_tmp.at(5));

                diff_tmp = sqrt(jointsReady_tmp.at(0)*jointsReady_tmp.at(0)+
                            jointsReady_tmp.at(1)*jointsReady_tmp.at(1)+
                            jointsReady_tmp.at(2)*jointsReady_tmp.at(2)+
                            5*jointsReady_tmp.at(3)*jointsReady_tmp.at(3)+
                            3*(jointsReady_tmp.at(4)-M_PI/2)*(jointsReady_tmp.at(4)-M_PI/2)+
                            jointsReady_tmp.at(5)*jointsReady_tmp.at(5));


                if(diff_tmp < diff)
                {
                    diff = diff_tmp;
                    tMb_ready = {oMg.col(0).row(0).val[0], oMg.col(1).row(0).val[0], oMg.col(2).row(0).val[0], oMg.col(3).row(0).val[0],
                                oMg.col(0).row(1).val[0], oMg.col(1).row(1).val[0], oMg.col(2).row(1).val[0], oMg.col(3).row(1).val[0],
                                oMg.col(0).row(2).val[0], oMg.col(1).row(2).val[0], oMg.col(2).row(2).val[0], oMg.col(3).row(2).val[0],
                                oMg.col(0).row(3).val[0], oMg.col(1).row(3).val[0], oMg.col(2).row(3).val[0], oMg.col(3).row(3).val[0]};
                    jointsReady = jointsReady_tmp;
                }
            }
        }
    }

   if(jointsReady.size()==6)
   {
       std::cout << "Ready: " << tMb_ready << std::endl;
       std::cout << "ready joints: ";
       for(int i=0;i<jointsReady.size();i++)
       {
           std::cout << jointsReady[i]*(180/M_PI) << " ";
       }
       std::cout << std::endl;
       Convert::joint2Pulse(jointsReady,pulseReady);
       std::cout << "ready pulse: ";
       for(int i=0;i<pulseReady.size();i++)
       {
           std::cout << pulseReady[i] << " ";
       }
       std::cout << std::endl;
   }
   else
   {
       std::cout << "Cannot move to approach position!" << std::endl;
       return false;
   }


//   tMb_pick = tMb_ready*cv::Matx44d{1, 0, 0, -0.09,
//                                    0, 1, 0, 0,
//                                    0, 0, 1, 0,
//                                    0, 0, 0, 1};

   tMb_pick = tMb_ready*cv::Matx44d{1, 0, 0, -0.03,
                                    0, 1, 0, 0,
                                    0, 0, 1, 0.02,
                                    0, 0, 0, 1};

   if(!Convert::inverseKinematic(tMb_pick,jointsPick))
   {
       std::cout << "Cannot move to pick position!" << std::endl;
       return false;
   }
   std::cout << "Pick: " << tMb_pick << std::endl;
   std::cout << "Pick joints: ";
   for(int i=0;i<jointsPick.size();i++)
   {
       std::cout << jointsPick[i]*(180/M_PI)<< " ";
   }
   std::cout << std::endl;
   Convert::joint2Pulse(jointsPick,pulsePick);
   std::cout << "pick pulse: ";
   for(int i=0;i<pulsePick.size();i++)
   {
       std::cout << pulsePick[i] << " ";
   }
   std::cout << std::endl;

   if(pulseReady.size()==6 && pulsePick.size()==6)
   {
       pulseReady[5] +=15000;
       pulsePick[5] +=15000;
       pulses.push_back(pulseReady);
       pulses.push_back(pulsePick);
       return true;
   }
}

void PoseEstimation::run()
{
    bool ret = this->startTrain(this->modelPath,
                                this->ppf_relative_sampling_step, this->ppf_relative_distance_step, this->ppf_num_angles);
    if(!ret)
    {
        std::cout << "Train thread not complete." << std::endl;
    }
}

bool PoseEstimation::startTrain(std::string modelPath,
                                double ppf_relative_sampling_step, double ppf_relative_distance_step, double ppf_num_angles)
{
    this->modelPath = modelPath;
    this->ppf_relative_sampling_step = ppf_relative_sampling_step;
    this->ppf_relative_distance_step = ppf_relative_distance_step;
    this->ppf_num_angles = ppf_num_angles;

    if(!this->modelPath.empty())
    {
        int64 tick1, tick2;
        cv::Mat model = cv::ppf_match_3d::loadPLYSimple(this->modelPath.c_str(), 1);
        this->detector = new cv::ppf_match_3d::PPF3DDetector(this->ppf_relative_sampling_step, this->ppf_relative_distance_step, this->ppf_num_angles);

        std::string dir = getTrainFolder(this->modelPath);
        if(!this->detector->readFile(dir))
        {
            std::cout << "Training..." << std::endl;
            tick1 = cv::getTickCount();
            this->detector->trainModel(model);
            tick2 = cv::getTickCount();
            std::cout << "Training complete in "
                << (double)(tick2 - tick1) / cv::getTickFrequency()
                << " sec" << std::endl;
            // Serialize the model
            std::cout << "Serializing..." << std::endl;
            tick1 = cv::getTickCount();
            this->detector->writeFile(dir);
            tick2 = cv::getTickCount();

            std::cout << "Serialization complete in " << (double)(tick2 - tick1) / cv::getTickFrequency() << " sec" << std::endl;
        }
        else
        {
            std::cout << "Found detector file: Skipping training phase" << std::endl;
        }
        this->isComplete = true;
        return true;
    }
    else
    {
        std::cout << "Please load model path before." << std::endl;
        return false;
    }
}
