#include "PointCloudCreator.h"
#include <QDebug>

//---------------------------------------> Background Filter <---------------------------------------------

void BackgroundFilter::closingMorphImgProc(cv::Mat &image, cv::Mat struct_element, int iter_dilate, int iter_erode)
{
    cv::dilate(image, image, struct_element, cv::Point(-1, -1), iter_dilate);
    cv::erode(image, image, struct_element, cv::Point(-1, -1), iter_erode);
};

BackgroundFilter::BackgroundFilter()
{
    this->sizeKernelBlur = 1;
    this->threshValue = 0;
    this->sizeStructElementClosing = 0;
    this->iterationClosing = 1;
    this->thresh_diff = 7e-5;
};

BackgroundFilter::BackgroundFilter(std::string paramPath)
{
    if(!this->loadParam(paramPath))
    {
        std::cout << "Cannot load param." << std::endl;
        
        this->sizeKernelBlur = 1;
        this->threshValue = 0;
        this->sizeStructElementClosing = 0;
        this->iterationClosing = 1;
        this->thresh_diff = 7e-5;
    }
};

BackgroundFilter::BackgroundFilter(int sizeKernelBlur, int threshValue, int sizeStructElementClosing, int iterationClosing)
{
    this->sizeKernelBlur = sizeKernelBlur;
    this->threshValue = threshValue;
    this->sizeStructElementClosing = sizeStructElementClosing;
    this->iterationClosing = iterationClosing;
    this->thresh_diff = 7e-5;
};

BackgroundFilter::~BackgroundFilter()
{

};

bool BackgroundFilter::loadParam(std::string paramPath)
{
    std::ifstream paramFile(paramPath);
    if(!paramFile.good()) return false;

    cv::FileStorage fn(paramPath.c_str(), cv::FileStorage::READ);

    fn["SizeKernelBlur"] >> this->sizeKernelBlur;
    fn["ThreshValue"] >> this->threshValue;
    fn["SizeStructElementClosing"] >> this->sizeStructElementClosing;
    fn["IterationClosing"] >> this->iterationClosing;
    this->thresh_diff = 7e-5;
    
    return true;
};

void BackgroundFilter::updateRefFrame(const cv::Mat &frame)
{
    this->ref_frame = frame.clone();
};

void BackgroundFilter::startFilter(const cv::Mat &cur_frame, cv::Mat &mask)
{
    cv::Mat gray_frame, gray_ref_frame, diff_frame;

    cv::cvtColor(cur_frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::cvtColor(this->ref_frame, gray_ref_frame, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray_frame, gray_frame, cv::Size(this->sizeKernelBlur, this->sizeKernelBlur), 0);
    cv::GaussianBlur(gray_ref_frame, gray_ref_frame, cv::Size(this->sizeKernelBlur, this->sizeKernelBlur), 0);

    cv::absdiff(gray_ref_frame, gray_frame, diff_frame);

    cv::Mat thresh_frame;
    cv::threshold(diff_frame, thresh_frame, this->threshValue, 255.0, cv::THRESH_BINARY);

    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->sizeStructElementClosing, this->sizeStructElementClosing));
    this->closingMorphImgProc(thresh_frame, structuringElement, this->iterationClosing, this->iterationClosing);

    mask = thresh_frame.clone();
};

//---------------------------------------> PointCloud Creator <---------------------------------------------
//-------------------------------------------------------------------------------------------------------------
PointCloudCreator::PointCloudCreator()
{
    this->minRange = 0.0;
    this->maxRange = 3.0;
};

//-------------------------------------------------------------------------------------------------------------
PointCloudCreator::PointCloudCreator(double minRange, double maxRange)
{
    this->minRange = minRange;
    this->maxRange = maxRange;
};

//-------------------------------------------------------------------------------------------------------------
PointCloudCreator::~PointCloudCreator()
{

};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::frameFilter(rs2::frameset &frames)
{
    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::decimation_filter dec_filter(2);  // Decimation    - reduces depth frame density
    rs2::sequence_id_filter seq_filter(2);
    rs2::threshold_filter thres_filter(this->minRange, this->maxRange);
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noice
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    frames = dec_filter.process(frames);
    frames = seq_filter.process(frames);
    frames = thres_filter.process(frames);
    frames = depth_to_disparity.process(frames);
    frames = spat_filter.process(frames);
    frames = temp_filter.process(frames);
    frames = disparity_to_depth.process(frames);
    frames = align_to_color.process(frames);
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::pcFilter(const cloud_pointer cloud_in, cloud_pointer cloud_out)
{
    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
    Cloud_Filter.setInputCloud (cloud_in);           // Input generated cloud to filter
    Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
    Cloud_Filter.setFilterLimits (this->minRange, this->maxRange);      // Set accepted interval values
    Cloud_Filter.filter (*cloud_out);              // Filtered Cloud Outputted
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::setDepthRange(double minRange, double maxRange)
{
    this->minRange = minRange;
    this->maxRange = maxRange;
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::load_PCDFile(cloud_pointer pointCloud, const std::string pc_path)
{
    std::cout << "Loading PCD Point Cloud File... " << std::endl;
    pcl::io::loadPCDFile (pc_path, *pointCloud);
    std::cout << pc_path << " successfully loaded. " << std::endl; 
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::save_PCDFile(const cloud_pointer pointCloud, const std::string pc_path)
{
    std::cout << "Generating PCD Point Cloud File... " << std::endl;
    pcl::io::savePCDFileASCII(pc_path, *pointCloud);
    std::cout << pc_path << " successfully generated. " << std::endl; 
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::show(viewer_pointer cloudViewer, const cloud_pointer pointCloud, const std::string pcID)
{
    // Set background of viewer to black
    cloudViewer->setBackgroundColor (0, 0, 0); 
    // Add generated point cloud and identify with string "Cloud"
    cloudViewer->addPointCloud<pcl::PointXYZRGB>(pointCloud, pcID);
    // Default size for rendered points
    cloudViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pcID);
    // Viewer Properties
    cloudViewer->initCameraParameters();  // Camera Parameters for ease of viewing

    std::cout << std::endl;
    std::cout << "Press [Q] in viewer to continue. " << std::endl;
    
    cloudViewer->spin(); // Allow user to rotate point cloud and view it

    // Note: No method to close PC visualizer, pressing Q to continue software flow only solution. 
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::clearPCID(viewer_pointer cloudViewer, const std::string pcID)
{
    cloudViewer->removePointCloud(pcID);
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::close(viewer_pointer cloudViewer)
{
    cloudViewer->close();
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::pointCloudFilter(cloud_container_Mat &pc_Object_In, cloud_container_Mat &pc_Object_Out, uint32_t thresh_area, double thresh_conf)
{
    for(int i = 0; i < pc_Object_In.size(); ++i)
    {
        if(pc_Object_In.at(i).at(0).cols < thresh_area) continue;
        if(pc_Object_In.at(i).at(0).at<float>(1, 0) < thresh_conf) continue;
        pc_Object_Out.push_back(pc_Object_In.at(i));
    }
    std::cout << "Number of elements before filter: " << pc_Object_In.size() << " and after filter: " << pc_Object_Out.size() << std::endl;
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::pointCloudFilter(cloud_container_PCL &pc_Object_In, cloud_container_PCL &pc_Object_Out, uint32_t thresh_area, double thresh_conf)
{
    for(int i = 0; i < pc_Object_In.size(); ++i)
    {
        if(std::get<0>(pc_Object_In.at(i))->points.size() < thresh_area) continue;
        if(std::get<1>(pc_Object_In.at(i)).at<float>(1, 0) < thresh_conf) continue;
        pc_Object_Out.push_back(pc_Object_In.at(i));
    }
    std::cout << "Number of elements before filter: " << pc_Object_In.size() << " and after filter: " << pc_Object_Out.size() << std::endl;
};

//-------------------------------------------------------------------------------------------------------------
void PointCloudCreator::objectFilter(const std::vector<Detection> &output_yolo, std::vector<Detection> &output_filter, const cv::Mat &mask, 
                                    uint32_t thresh_area, double thresh_conf, bool choose_couple_best)
{
    cv::Mat mask_object;

    if(choose_couple_best)
    {
        Detection bottleBest, cupBest;

        bottleBest.confidence = 0.0;
        bottleBest.class_id = 1;

        cupBest.confidence = 0.0;
        cupBest.class_id = 0;

        int sum = 0;

        for(int i = 0; i < output_yolo.size(); ++i)
        {
            if(output_yolo.at(i).confidence < thresh_conf)
            {
                continue;
            }

            int x_tl = output_yolo.at(i).box.x;
            int y_tl = output_yolo.at(i).box.y;
            int x_br = output_yolo.at(i).box.x + output_yolo.at(i).box.width;
            int y_br = output_yolo.at(i).box.y + output_yolo.at(i).box.height;

            if(x_tl < 0 || y_tl < 0 || x_br > mask.cols || y_br > mask.rows) continue;

            mask_object = mask( cv::Range(y_tl, y_br), cv::Range(x_tl, x_br));

            if(cv::countNonZero(mask_object) < thresh_area)
            {
                continue;
            }

            if(output_yolo.at(i).class_id == bottleBest.class_id)
            {
                if(output_yolo.at(i).confidence > bottleBest.confidence) bottleBest = output_yolo.at(i);
            }
            else if(output_yolo.at(i).class_id == cupBest.class_id)
            {
                if(output_yolo.at(i).confidence > cupBest.confidence) cupBest = output_yolo.at(i);
            }
        }
        qDebug() << 1;

        if(bottleBest.confidence != 0.0 && cupBest.confidence != 0.0)
        {
            qDebug() << 2;
            output_filter.push_back(cupBest);
            qDebug() << 3;
            output_filter.push_back(bottleBest);
            qDebug() << 4;
        }
    }
    else
    {
        for(int i = 0; i < output_yolo.size(); ++i)
        {
            if(output_yolo.at(i).confidence < thresh_conf) continue;

            int x_tl = output_yolo.at(i).box.x;
            int y_tl = output_yolo.at(i).box.y;
            int x_br = output_yolo.at(i).box.x + output_yolo.at(i).box.width;
            int y_br = output_yolo.at(i).box.y + output_yolo.at(i).box.height;

            if(x_tl < 0 || y_tl < 0 || x_br > mask.cols || y_br > mask.rows) continue;

            mask_object = mask( cv::Range(y_tl, y_br), cv::Range(x_tl, x_br));

            if(cv::countNonZero(mask_object) < thresh_area) continue;

            if(output_yolo.at(i).class_id!=0) continue;

            output_filter.push_back(output_yolo.at(i));
        }

//        Detection tmp;
//        for(int i=0; i<output_filter.size()-1; i++)
//        {
//            for(int j=1; j<output_filter.size(); j++)
//            {
//                if(output_filter.at(i).confidence < output_filter.at(j).confidence)
//                {
//                    tmp = output_filter.at(i);
//                    output_filter.at(i) = output_filter.at(j);
//                    output_filter.at(j) = tmp;
//                }
//            }
//        }
    }
    //std::cout << "Number of elements before filter: " << output_yolo.size() << " and after filter: " << output_filter.size() << std::endl;
};

//-----------------------------------------> Realsense PointCloud Creator <-----------------------------------------------

//-------------------------------------------------------------------------------------------------------------
std::tuple<int, int, int> RealsensePC::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
};

//-------------------------------------------------------------------------------------------------------------
cloud_pointer RealsensePC::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color)
{
    // Object Declaration (Point Cloud)
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (RGB due to Camera Model)
        cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<0>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<2>

    }
    
   return cloud; // PCL RGB Point Cloud generated
};

//-------------------------------------------------------------------------------------------------------------
RealsensePC::RealsensePC() : PointCloudCreator()
{

};

//-------------------------------------------------------------------------------------------------------------
RealsensePC::RealsensePC(double minRange, double maxRange) : PointCloudCreator(minRange, maxRange)
{

};

//-------------------------------------------------------------------------------------------------------------
RealsensePC::~RealsensePC()
{

};

//-------------------------------------------------------------------------------------------------------------
void RealsensePC::startPCProcess(cloud_pointer &pointCloud, rs2::frameset &frames)
{
    this->frameFilter(frames);

    rs2::frame color_frame, depth_frame;

    depth_frame = frames.get_depth_frame();
    color_frame = frames.get_color_frame();

    // Map Color texture to each point
    this->pc.map_to(color_frame);

    // Generate Point Cloud
    this->points = this->pc.calculate(depth_frame);

    // Convert generated Point Cloud to PCL Formatting
    cloud_pointer cloud = this->PCL_Conversion(this->points, color_frame);
    
    //========================================
    // Filter PointCloud (PassThrough Method)
    //========================================
    this->pcFilter(cloud, pointCloud);
};

//-------------------------------------------------------------------------------------------------------------
void RealsensePC::pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_Mat &pc_Object)
{
    this->frameFilter(frames);

    rs2::frame depth_frame, color_frame;

    depth_frame = frames.get_depth_frame();
    color_frame = frames.get_color_frame();

    this->pc.map_to(color_frame);
    this->points = this->pc.calculate(depth_frame);

    auto Texture_Coord = this->points.get_texture_coordinates();
    auto Vertex = this->points.get_vertices();

    pc_Object.resize(output.size());

    int width = color_frame.as<rs2::video_frame>().get_width();
    int height = color_frame.as<rs2::video_frame>().get_height();

    int x_value, y_value;

    for(int i = 0; i < output.size(); ++i)
    {
        cv::Mat mask_object = mask( cv::Range(output.at(i).box.y, (output.at(i).box.y + output.at(i).box.height)), 
                                    cv::Range(output.at(i).box.x, (output.at(i).box.x + output.at(i).box.width)));
        pc_Object.at(i).resize(2);
        pc_Object.at(i).at(0).create(cv::countNonZero(mask_object), 3, CV_32FC1);
        pc_Object.at(i).at(1).create(2, 1, CV_32FC1);

        int sum_points = 0;
        for(int j = 0; j < this->points.size(); ++j)
        {
            if(Vertex[j].z < this->minRange) continue;

            x_value = std::min(std::max(int(Texture_Coord[j].u * width + .5f), 0), width - 1);
            y_value = std::min(std::max(int(Texture_Coord[j].v * height + .5f), 0), height - 1);

            if(mask.at<uchar>(y_value, x_value) == 0) continue;
            if((x_value < output.at(i).box.x) || (x_value >= (output.at(i).box.x + output.at(i).box.width))) continue;
            if((y_value < output.at(i).box.y) || (y_value >= (output.at(i).box.y + output.at(i).box.height))) continue;

            pc_Object.at(i).at(0).at<float>(sum_points, 0) = Vertex[j].x;
            pc_Object.at(i).at(0).at<float>(sum_points, 1) = Vertex[j].y;
            pc_Object.at(i).at(0).at<float>(sum_points, 2) = Vertex[j].z;

            sum_points++;
        }

        pc_Object.at(i).at(1).at<int>(0, 0) = output.at(i).class_id;
        pc_Object.at(i).at(1).at<float>(1, 0) = output.at(i).confidence;        
    }
};

//-------------------------------------------------------------------------------------------------------------
void RealsensePC::pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_PCL &pc_Object)
{
    this->frameFilter(frames);

    rs2::frame depth_frame, color_frame;

    depth_frame = frames.get_depth_frame();
    color_frame = frames.get_color_frame();

    this->pc.map_to(color_frame);
    this->points = this->pc.calculate(depth_frame);

    auto Texture_Coord = this->points.get_texture_coordinates();
    auto Vertex = this->points.get_vertices();

    pc_Object.resize(output.size());

    int width = color_frame.as<rs2::video_frame>().get_width();
    int height = color_frame.as<rs2::video_frame>().get_height();

    int x_value, y_value;

    cv::Mat mask_tmp;
    for(int i = 0; i < output.size(); ++i)
    {
        std::get<0>(pc_Object.at(i)).reset(new point_cloud);
        std::get<1>(pc_Object.at(i)).create(2, 1, CV_32FC1);
        std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

        mask_tmp = mask(cv::Range(output.at(i).box.y, (output.at(i).box.y + output.at(i).box.height)), 
                        cv::Range(output.at(i).box.x, (output.at(i).box.x + output.at(i).box.width)));

        std::get<0>(pc_Object.at(i))->points.resize(cv::countNonZero(mask_tmp));

        int sum_points = 0;
        for(int j = 0; j < this->points.size(); ++j)
        {
            if(Vertex[j].z < this->minRange) continue;

            x_value = std::min(std::max(int(Texture_Coord[j].u * width + .5f), 0), width - 1);
            y_value = std::min(std::max(int(Texture_Coord[j].v * height + .5f), 0), height - 1);

            if(mask.at<uchar>(y_value, x_value) == 0) continue;
            if((x_value < output.at(i).box.x) || (x_value >= (output.at(i).box.x + output.at(i).box.width))) continue;
            if((y_value < output.at(i).box.y) || (y_value >= (output.at(i).box.y + output.at(i).box.height))) continue;

            std::get<0>(pc_Object.at(i))->points[sum_points].x = Vertex[j].x;
            std::get<0>(pc_Object.at(i))->points[sum_points].y = Vertex[j].y;
            std::get<0>(pc_Object.at(i))->points[sum_points].z = Vertex[j].z;

            RGB_Color = this->RGB_Texture(color_frame, Texture_Coord[j]);
            std::get<0>(pc_Object.at(i))->points[sum_points].r = std::get<2>(RGB_Color); 
            std::get<0>(pc_Object.at(i))->points[sum_points].g = std::get<1>(RGB_Color); 
            std::get<0>(pc_Object.at(i))->points[sum_points].b = std::get<0>(RGB_Color); 

            sum_points++;
        }

        std::get<0>(pc_Object.at(i))->resize(sum_points);
        this->pcFilter(std::get<0>(pc_Object.at(i)), std::get<0>(pc_Object.at(i)));
        std::get<1>(pc_Object.at(i)).at<int>(0, 0) = output.at(i).class_id;
        std::get<1>(pc_Object.at(i)).at<float>(1, 0) = output.at(i).confidence;        
    }
};

//-----------------------------------------> Algorithm PointCloud Creator <-----------------------------------------------

//-------------------------------------------------------------------------------------------------------------
AlgorithmPC::AlgorithmPC() : PointCloudCreator()
{

};

//-------------------------------------------------------------------------------------------------------------
AlgorithmPC::AlgorithmPC(double minRange, double maxRange) : PointCloudCreator(minRange, maxRange)
{
    
};

//-------------------------------------------------------------------------------------------------------------
AlgorithmPC::~AlgorithmPC()
{

};

//-------------------------------------------------------------------------------------------------------------
void AlgorithmPC::startPCProcess(cloud_pointer &pointCloud, rs2::frameset &frames)
{
    this->frameFilter(frames);

    rs2::frame color_frame, depth_frame;

    depth_frame = frames.get_depth_frame();

    this->intrinsic = rs2::video_stream_profile(depth_frame.get_profile()).get_intrinsics();

    color_frame = frames.get_color_frame();
    rs2::video_frame colorized_depth = color_frame.as<rs2::video_frame>();

    int stride = colorized_depth.get_stride_in_bytes();
    uint8_t* ptr = (uint8_t*)colorized_depth.get_data();

    pointCloud->points.resize(depth_frame.as<rs2::depth_frame>().get_height()*depth_frame.as<rs2::depth_frame>().get_width());

    size_t index = 0;

    for(int y = 0; y < depth_frame.as<rs2::depth_frame>().get_height(); ++y)
    {
        for(int x = 0; x < depth_frame.as<rs2::depth_frame>().get_width(); ++x)
        {
            float P[3];

            P[2] = depth_frame.as<rs2::depth_frame>().get_distance(x, y);

            if(P[2] > this->minRange)
            {
                P[0] = (float)(x-intrinsic.ppx)*P[2]/intrinsic.fx;
                P[1] = (float)(y-intrinsic.ppy)*P[2]/intrinsic.fy;
                P[2] = P[2];

                pointCloud->points.at(index).x =P[0];
                pointCloud->points.at(index).y =P[1];
                pointCloud->points.at(index).z =P[2];
                pointCloud->points.at(index).r =int(ptr[y * stride + (3*x) + 2]);
                pointCloud->points.at(index).g =int(ptr[y * stride + (3*x) + 1]);
                pointCloud->points.at(index).b =int(ptr[y * stride + (3*x) + 0]);

                index++;
            }
        }
    }
    pointCloud->points.resize(index);
};

//-------------------------------------------------------------------------------------------------------------
void AlgorithmPC::pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_Mat &pc_Object)
{
    this->frameFilter(frames);

    rs2::frame color_frame, depth_frame;

    depth_frame = frames.get_depth_frame();

    this->intrinsic = rs2::video_stream_profile(depth_frame.get_profile()).get_intrinsics();

    pc_Object.resize(output.size());

    for(int i = 0; i < output.size(); ++i)
    {
        cv::Mat mask_object = mask( cv::Range(output.at(i).box.y, (output.at(i).box.y + output.at(i).box.height)), 
                                    cv::Range(output.at(i).box.x, (output.at(i).box.x + output.at(i).box.width)));
        pc_Object.at(i).resize(2); 
        pc_Object.at(i).at(0).create(cv::countNonZero(mask_object), 3, CV_32FC1);
        pc_Object.at(i).at(1).create(2, 1, CV_32FC1);

        int sum_points = 0;

        for(int j = output.at(i).box.y; j < (output.at(i).box.y + output.at(i).box.height); ++j)
        {
            for(int k = output.at(i).box.x; k < (output.at(i).box.x + output.at(i).box.width); ++k)
            {
                if(mask.at<uchar>(j, k) == 0) continue;

                float z = depth_frame.as<rs2::depth_frame>().get_distance(k, j);

                if(z < this->minRange && z > this->maxRange) continue;

                pc_Object.at(i).at(0).at<float>(sum_points, 0) = (float)(k - this->intrinsic.ppx)*z/this->intrinsic.fx;
                pc_Object.at(i).at(0).at<float>(sum_points, 1) = (float)(j - this->intrinsic.ppy)*z/this->intrinsic.fy;
                pc_Object.at(i).at(0).at<float>(sum_points, 2) = z;
                sum_points++;
            }
        }
        pc_Object.at(i).at(1).at<int>(0, 0) = output.at(i).class_id;
        pc_Object.at(i).at(1).at<float>(1, 0) = output.at(i).confidence;
    }
};

//-------------------------------------------------------------------------------------------------------------
void AlgorithmPC::pcObjectCollection(rs2::frameset &frames, const cv::Mat &mask, const std::vector<Detection> &output, cloud_container_PCL &pc_Object)
{
    this->frameFilter(frames);

    rs2::frame color_frame, depth_frame;

    depth_frame = frames.get_depth_frame();
    cv::Mat depth = Convert::frame_to_mat(depth_frame);

    this->intrinsic = rs2::video_stream_profile(depth_frame.get_profile()).get_intrinsics();

    color_frame = frames.get_color_frame();
    rs2::video_frame colorized_depth = color_frame.as<rs2::video_frame>();

    int stride = colorized_depth.get_stride_in_bytes();
    uint8_t* ptr = (uint8_t*)colorized_depth.get_data();

    pc_Object.resize(output.size());

    cv::Mat mask_tmp;

    for(int i = 0; i < output.size(); ++i)
    {
        std::get<0>(pc_Object.at(i)).reset(new point_cloud);
        std::get<1>(pc_Object.at(i)).create(2, 1, CV_32FC1);

        mask_tmp = mask(cv::Range(output.at(i).box.y, (output.at(i).box.y + output.at(i).box.height)), 
                        cv::Range(output.at(i).box.x, (output.at(i).box.x + output.at(i).box.width)));

        std::get<0>(pc_Object.at(i))->points.resize(cv::countNonZero(mask_tmp));

        // std::cout << "Number of non-zero pixels img cut: " << cv::countNonZero(mask_tmp) << std::endl;
        // std::cout << "Number of non-zero pixels: " << cv::countNonZero(mask) << std::endl;

        size_t index = 0;
        for(int j = output.at(i).box.y; j < (output.at(i).box.y + output.at(i).box.height); ++j)
        {
            for(int k = output.at(i).box.x; k < (output.at(i).box.x + output.at(i).box.width); ++k)
            {
                if(mask.at<uchar>(j, k) == 0) continue;

                float P[3];

                // P[2] = depth_frame.as<rs2::depth_frame>().get_distance(k, j);
                P[2] = depth.at<uint16_t>(k, j)*1e-5;

                if(P[2] < this->minRange && P[2] > this->maxRange) continue;

                P[0] = (float)(k - this->intrinsic.ppx)*P[2]/this->intrinsic.fx;
                P[1] = (float)(j - this->intrinsic.ppy)*P[2]/this->intrinsic.fy;
                P[2] = P[2];

                std::get<0>(pc_Object.at(i))->points.at(index).x = P[0];
                std::get<0>(pc_Object.at(i))->points.at(index).y = P[1];
                std::get<0>(pc_Object.at(i))->points.at(index).z = P[2];
                std::get<0>(pc_Object.at(i))->points.at(index).r = int(ptr[j * stride + (3*k) + 2]);
                std::get<0>(pc_Object.at(i))->points.at(index).g = int(ptr[j * stride + (3*k) + 1]);
                std::get<0>(pc_Object.at(i))->points.at(index).b = int(ptr[j * stride + (3*k) + 0]);

                index++;
            }
        }

        // std::cout << "Number of points: " << index << std::endl;
        std::get<0>(pc_Object.at(i))->points.resize(index);
        this->pcFilter(std::get<0>(pc_Object.at(i)), std::get<0>(pc_Object.at(i)));
        std::get<1>(pc_Object.at(i)).at<int>(0, 0) = output.at(i).class_id;
        std::get<1>(pc_Object.at(i)).at<float>(1, 0) = output.at(i).confidence;
    }
};

//-------------------------------------------------------------------------------------------------------------
