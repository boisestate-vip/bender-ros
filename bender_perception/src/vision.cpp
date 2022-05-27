#include <bender_perception/vision.h>


LaneDetection::LaneDetection(ros::NodeHandle &nh, int device_id) :
    device_id_(device_id),
    input_topic_(""),
    it_(nh),
    tf_listener_(tf_buffer_)
{
    // Starts capture device
    cam_capture_.open(device_id_);
    if (!cam_capture_.isOpened()) {
        ROS_ERROR("Cannot open camera ID %d", device_id_);
        nh.shutdown();
    }

    // Read image and store it to img_src_
    cam_capture_.read(img_src_);
    if (img_src_.empty()) {
        ROS_ERROR("Cannot read from camera ID %d", device_id_);
        nh.shutdown();
    } 
    init(nh);

}


LaneDetection::LaneDetection(ros::NodeHandle &nh, string input_topic, string output_topic) :
    device_id_(UINT8_MAX),
    input_topic_(input_topic),
    output_topic_(output_topic),
    it_(nh),
    tf_listener_(tf_buffer_)
{
    // Subscribe to input image
    std::string topic = nh.resolveName(input_topic_);
    input_sub_ = it_.subscribeCamera(topic, 1, &LaneDetection::readImage, this);
    init(nh);
}


void LaneDetection::init(ros::NodeHandle &nh)
{
    output_pub_ = it_.advertise(output_topic_, 1);
    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_from_image", 1);
    btl_.set_output_frame("bender_camera");

    ros::NodeHandle vision_nh(nh, "vision");
    vision_nh.param("scale", params.scale, params.scale);
    vision_nh.param("laser_dist_scale_x", params.laser_dist_scale_x, params.laser_dist_scale_x);
    vision_nh.param("laser_dist_scale_y", params.laser_dist_scale_y, params.laser_dist_scale_y);
    vision_nh.param("gamma", params.gamma, params.gamma);
    vision_nh.param("num_colors", params.num_colors, params.num_colors);
    vision_nh.param("smooth_kernel_size", params.smooth_kernel_size, params.smooth_kernel_size);
    vision_nh.param("roi_from_top", params.roi_from_top, params.roi_from_top);
    vision_nh.param("roi_from_bot", params.roi_from_bot, params.roi_from_bot);
    vision_nh.param("color1_thresh_lb", params.color_thresh_lb[0], params.color_thresh_lb[0]);
    vision_nh.param("color2_thresh_lb", params.color_thresh_lb[1], params.color_thresh_lb[1]);
    vision_nh.param("color3_thresh_lb", params.color_thresh_lb[2], params.color_thresh_lb[2]);
    vision_nh.param("color1_thresh_ub", params.color_thresh_ub[0], params.color_thresh_ub[0]);
    vision_nh.param("color2_thresh_ub", params.color_thresh_ub[1], params.color_thresh_ub[1]);
    vision_nh.param("color3_thresh_ub", params.color_thresh_ub[2], params.color_thresh_ub[2]);
    vision_nh.param("projection_distance", params.projection_distance, params.projection_distance);
    vision_nh.param("threshold_adaptive", params.threshold_adaptive, params.threshold_adaptive);
    vision_nh.param("threshold_lock", params.threshold_lock, params.threshold_lock);
    vision_nh.param("adaptive_threshold_method", params.adaptive_type, params.adaptive_type);
    vision_nh.param("adaptive_threshold_mean_subtract", params.adaptive_mean_subtract, params.adaptive_mean_subtract);
    vision_nh.param("adaptive_threshold_block_size", params.adaptive_block_size, params.adaptive_block_size);
    vision_nh.param("threshold_type", params.threshold_type, params.threshold_type);
    vision_nh.param("color_conversion", params.color_type, params.color_type);

    if (!(params.scale > 0.0 && params.scale <= 1.0)) 
    {
        ROS_WARN("The parameter `scale' must be in the range (0,1]. Got %.1f, ignoring and using `scale' = 1.0", params.scale);
        params.scale = 1.0;
    }
    if (!(params.gamma > 0 && params.gamma <= 100)) 
    {
        ROS_WARN("The parameter `gamma' must be in the range (0,100]. Got %d, ignoring and using `gamma' = 5.0", params.num_colors);
        params.gamma = 5.0;
    }
    if (!(params.num_colors > 1 && params.num_colors <= 255)) 
    {
        ROS_WARN("The parameter `num_colors' must be in the range [2,255]. Got %d, ignoring and using `num_colors' = 2", params.num_colors);
        params.num_colors = 2;
    }

    dynamic_recfg_server_ = std::make_shared<dynamic_reconfigure::Server<VisionConfig>>(vision_nh);
    dynamic_reconfigure::Server<VisionConfig>::CallbackType cb = std::bind(
        &LaneDetection::reconfigureCB, this, std::placeholders::_1, std::placeholders::_2);
    dynamic_recfg_server_->setCallback(cb);

}


LaneDetection::~LaneDetection()
{
}


void LaneDetection::readImage()
{
    cam_capture_.read(img_src_);
}


void LaneDetection::readImage(const sensor_msgs::ImageConstPtr &img_msg, 
                              const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    // Get camera model that produced the ROS image msg
    cam_model_.fromCameraInfo(info_msg);
    
    // Convert incoming ROS image msg to OpenCV msg
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        img_src_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert image. cv_bridge exception: %s", e.what());
        return;
    }
}


void LaneDetection::reconfigureCB(VisionConfig& config, uint32_t level)
{
    // Lock this to prevent parameters changing mid-cycle
    boost::mutex::scoped_lock l(config_mutex_);

    params.scale = config.scale;
    params.laser_dist_scale_x = config.laser_dist_scale_x;
    params.laser_dist_scale_y = config.laser_dist_scale_y;
    params.laser_dist_offset = config.laser_dist_offset;
    btl_.set_dist_scale(params.laser_dist_scale_x);
    params.gamma = config.gamma;
    params.num_colors = config.num_colors;
    params.smooth_kernel_size = config.smooth_kernel_size;
    params.roi_from_top = config.roi_from_top;
    params.roi_from_bot = config.roi_from_bot;
    params.color_thresh_lb[0] = config.color1_thresh_lb;
    params.color_thresh_lb[1] = config.color2_thresh_lb;
    params.color_thresh_lb[2] = config.color3_thresh_lb;
    params.color_thresh_ub[0] = config.color1_thresh_ub;
    params.color_thresh_ub[1] = config.color2_thresh_ub;
    params.color_thresh_ub[2] = config.color3_thresh_ub;
    if (config.projection_distance != params.projection_distance)
    {
        params.projection_distance = config.projection_distance;
        computeHomography();
    }
    params.threshold_adaptive = config.threshold_adaptive;
    params.threshold_lock = config.threshold_lock;
    params.adaptive_type = config.adaptive_threshold_method;
    params.adaptive_block_size = config.adaptive_threshold_block_size;
    params.adaptive_mean_subtract = config.adaptive_threshold_mean_subtract;
    params.threshold_type = config.threshold_type;
    params.color_type = config.color_conversion;
}


void LaneDetection::applyColorThreshold()
{
    const auto lb = Scalar(params.color_thresh_lb[0], params.color_thresh_lb[1], params.color_thresh_lb[2]);
    const auto ub = Scalar(params.color_thresh_ub[0], params.color_thresh_ub[1], params.color_thresh_ub[2]);
    Mat mask = Mat::zeros(img_out_.size(), CV_8U);
    Mat dst = Mat::zeros(img_out_.size(), CV_8UC3);
    inRange(img_out_, lb, ub, mask);
    bitwise_and(img_out_, img_out_, dst, mask);
    img_out_ = dst;
}


void LaneDetection::gammaCorrection()
{
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
    {
        p[i] = saturate_cast<uchar>(pow(i / 255.0, params.gamma) * 255.0);
    }
    LUT(img_out_, lookUpTable, img_out_);
}


void LaneDetection::smooth()
{
    /* 
    stackoverflow.com/questions/42065405
    smooth the image with alternative closing and opening
    with an enlarging kernel
    */
    Mat morph = img_out_.clone();
    for (int r = 1; r < params.smooth_kernel_size; r++)
    {
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(2*r+1, 2*r+1));
        morphologyEx(morph, morph, CV_MOP_CLOSE, kernel);
        morphologyEx(morph, morph, CV_MOP_OPEN, kernel);
    }
    img_out_ = morph;
}


void LaneDetection::quantize()
{
    // Convert to float & reshape to a [3 x W*H] Mat 
    // (so every pixel is on a row of it's own)
    Mat data;
    img_out_.convertTo(data, CV_32F);
    data = data.reshape(1, data.total());
    
    // Mat labels, centers;
    const int init_method = has_centers_ ? KMEANS_USE_INITIAL_LABELS | KMEANS_PP_CENTERS : KMEANS_PP_CENTERS;
    double compactness = kmeans(
        data,
        this->params.num_colors, 
        labels_,
        TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 20, 0.05 ),
        4, 
        init_method,
        centers_
    );
    has_centers_ = true;

    // reshape both to a single row of Vec3f pixels:
    centers_ = centers_.reshape(3, centers_.rows);
    data = data.reshape(3, data.rows);

    // replace pixel values with their center value:
    Vec3f *p = data.ptr<Vec3f>();
    for (size_t i=0; i<data.rows; i++) {
        int center_id = labels_.at<int>(i);
        p[i] = centers_.at<Vec3f>(center_id);
    }

    // back to 2d, and uchar:
    img_out_ = data.reshape(3, img_out_.rows);
    img_out_.convertTo(img_out_, CV_8U);
}


void LaneDetection::toBinary()
{
    cvtColor(img_out_, img_out_, COLOR_BGR2GRAY);
    if (params.threshold_adaptive)
    {
        int threshold_type = params.threshold_type;
        if ((threshold_type != THRESH_BINARY) || (threshold_type != THRESH_BINARY_INV))
        {
            ROS_WARN_ONCE("Parameter threshold_type is not valid for adaptiveThreshold method. Use either THRESH_BINARY or THRESH_BINARY_INV.");
            threshold_type = static_cast<int>(THRESH_BINARY);
        }
        int block_size = params.adaptive_block_size;
        block_size += ((block_size % 2) == 0) ? 1 : 0;
        adaptiveThreshold(img_out_, img_out_, 255, 
            static_cast<AdaptiveThresholdTypes>(params.adaptive_type), 
            static_cast<ThresholdTypes>(threshold_type), 
            block_size, 
            params.adaptive_mean_subtract
        );
    } else 
    {
        if (!params.threshold_lock || params.threshold_lock != 255) 
        {
            threshold_val_ = threshold(img_out_, img_out_, 0, 255, static_cast<ThresholdTypes>(params.threshold_type));
        } else if ((params.threshold_type == THRESH_BINARY) || (params.threshold_type == THRESH_BINARY_INV))
        {
            threshold(img_out_, img_out_, threshold_val_, 255, static_cast<ThresholdTypes>(params.threshold_type));
        } else
        {
            threshold(img_out_, img_out_, threshold_val_, 255, THRESH_BINARY);
        }
            
    }
#ifdef HAVE_OPENCV_XIMGPROC
    // ximgproc::thinning(img_out_, img_out_);
#endif
}


void LaneDetection::update()
{
    if (input_topic_.empty()) 
    {
        readImage();
    }
    
    if (!img_src_.empty())
    {
        // Lock this to prevent parameters changing mid-cycle
        boost::mutex::scoped_lock l(config_mutex_);

        // If Homography wasn't already computed, do it once and remember it
        if (!has_homography_)
        {
            computeHomography();
            has_homography_ = true;
        }

        // Crop to ROI
        Range rowrange(params.roi_from_top, img_src_.size().height-params.roi_from_bot);
        Range colrange(Range::all());
        img_src_(rowrange, colrange).copyTo(img_out_);


        if (params.scale != 1.0) 
        { 
            resize(img_out_, img_out_, Size(), params.scale, params.scale); 
        }
        cvtColor(img_out_, img_out_, static_cast<ColorConversionCodes>(params.color_type));
        gammaCorrection();
        applyColorThreshold();
        // quantize();
        toBinary();
        smooth();
        if (params.scale != 1.0) {
            resize(img_out_, img_out_, Size(), 1.0/params.scale, 1.0/params.scale);
        }
        copyMakeBorder(img_out_, img_out_, params.roi_from_top, params.roi_from_bot, 0, 0, BORDER_CONSTANT, 0);
        projectToGrid();

        /*
        img_src_(Range::all(), Range::all()).copyTo(img_out_);
        projectToGrid();
        */
    } 
    else
    {
        ROS_INFO_THROTTLE(2.0, "Waiting for input topic %s", input_topic_.c_str());
    }
}


void LaneDetection::computeHomography()
{
    const Size image_size = img_src_.size();
    const double w = image_size.width;
    const double h = image_size.height;

    // TODO: Load these from a yaml file
    const double alpha = (15-90) * M_PI / 180.0;
    const double beta = 0;
    const double gamma = 0; 
    const double dist = params.projection_distance;

    // Projecion matrix 2D -> 3D
    Matx43d A1(
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0, 0,
        0, 0, 1
    );

    // Rotation matrices Rx, Ry, Rz
    Matx44d RX(
        1, 0, 0, 0,
        0, cos(alpha), -sin(alpha), 0,
        0, sin(alpha), cos(alpha), 0,
        0, 0, 0, 1
    );
    Matx44d RY(
        cos(beta), 0, sin(beta), 0,
        0, 1, 0, 0,
        -sin(beta), 0, cos(beta), 0,
        0, 0, 0, 1
    );
    Matx44d RZ(
        cos(gamma), -sin(gamma), 0, 0,
        sin(gamma), cos(gamma), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );

    // Compose the rotations
    Matx44d R = RX * RY * RZ;

    // T - translation matrix
    Matx44d T(
        1, 0, 0, 0,  
        0, 1, 0, 0,  
        0, 0, 1, cam_model_.fx()/dist,
        0, 0, 0, 1
    ); 
    
    // K - intrinsic matrix 
    Matx34d K(
        cam_model_.fx(), 0, cam_model_.cx(), 0,
        0, cam_model_.fy(), cam_model_.cy(), 0,
        0, 0, 1, 0
    ); 

    // H - Homography
    H_ = K * (T * (R * A1));
}


void LaneDetection::projectToGrid()
{
    warpPerspective(
        img_out_, 
        img_out_, 
        H_, 
        img_out_.size(), 
        INTER_CUBIC | WARP_INVERSE_MAP
    );
}


void LaneDetection::displayOutput()
{
    if (!img_src_.empty() && !img_out_.empty())
    {
        Mat out, display;
        resize(img_out_, out, img_src_.size());
        hconcat(img_src_, out, display);
        namedWindow(wname_, WINDOW_AUTOSIZE);
        imshow(wname_, display);
        waitKey(1);
    }
}


void LaneDetection::publishQuantized()
{
    if (!img_out_.empty()) 
    {
        std::string encoding = typeToString(img_out_.type()).substr(3);
        output_msg_ = cv_bridge::CvImage(std_msgs::Header(), encoding.c_str(), img_out_).toImageMsg();
        output_pub_.publish(output_msg_);
        sensor_msgs::CameraInfoConstPtr info_msg( new sensor_msgs::CameraInfo(cam_model_.cameraInfo()) );
        sensor_msgs::LaserScanPtr scan_msg = btl_.convert_msg(output_msg_, info_msg);
        scan_pub_.publish(scan_msg);
    }
}
