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

    dynamic_recfg_server_ = std::make_shared<dynamic_reconfigure::Server<VisionConfig>>(nh);
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
    ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
        config.int_param, config.double_param, 
        config.str_param.c_str(), 
        config.bool_param?"True":"False", 
        config.size);
}


void LaneDetection::applyThreshold()
{
    inRange(img_src_, Scalar(0, 0, 0), Scalar(255, 255, 255), img_src_);
}


void LaneDetection::gammaCorrection()
{
    const double gamma = 5.0;
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
    {
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
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
    for (int r = 1; r < 5; r++)
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
        this->num_colors, 
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
    cvtColor(img_out_, img_out_, CV_BGR2GRAY);
    // threshold(img_out_, img_out_, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // adaptiveThreshold(img_out_, img_out_, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 3);
    // adaptiveThreshold(img_out_, img_out_, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 2.5);
    adaptiveThreshold(img_out_, img_out_, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    // adaptiveThreshold(img_out_, img_out_, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 2);
    bitwise_not(img_out_, img_out_);
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
        if (!has_homography_)
        {
            computeHomography();
            has_homography_ = true;
        }
        int roi_from_top = 160;
        int roi_from_bot = 60;
        Range rowrange(roi_from_top, img_src_.size().height-roi_from_bot);
        Range colrange(Range::all());
        img_src_(rowrange, colrange).copyTo(img_out_);
        if (scale != 1.0) { resize(img_out_, img_out_, Size(), scale, scale); }
        cvtColor(img_out_, img_out_, COLOR_BGR2HLS);
        gammaCorrection();
        // quantize();
        smooth();
        toBinary();
        if (scale != 1.0) { resize(img_out_, img_out_, Size(), 1.0/scale, 1.0/scale); }
        copyMakeBorder(img_out_, img_out_, roi_from_top, roi_from_bot, 0, 0, BORDER_CONSTANT, 0);
        projectToGrid();
    } 
    else
    {
        ROS_INFO_THROTTLE(2.0, "Waiting for input topic %s", input_topic_.c_str());
    }
}


void LaneDetection::computeHomography()
{
    Size image_size = img_src_.size();
    double w = (double)image_size.width;
    double h = (double)image_size.height;

    // TODO: Load these from a yaml file
    double alpha = (15-90) * M_PI / 180.0;
    double beta = 0;
    double gamma = 0; 
    double dist = 0.6;

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
        0, 0, 1, dist*cam_model_.fx(),  
        0, 0, 0, 1
    ); 
    
    // K - intrinsic matrix 
    Matx34d K(
        cam_model_.fx(), 0, w/2, 0,
        0, cam_model_.fy(), h/2, 0,
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
