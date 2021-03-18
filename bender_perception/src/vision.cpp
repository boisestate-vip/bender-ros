#include <bender_perception/vision.h>

LaneDetection::LaneDetection(ros::NodeHandle *nh, int device_id) :
    device_id_(device_id),
    input_topic_(""),
    it_(*nh)
{
    // Starts capture device
    cam_capture_.open(device_id_);
    if (!cam_capture_.isOpened()) {
        ROS_ERROR("Cannot open camera ID %d", device_id_);
        nh->shutdown();
    }

    // Read image and store it to img_src_
    cam_capture_.read(img_src_);
    if (img_src_.empty()) {
        ROS_ERROR("Cannot read from camera ID %d", device_id_);
        nh->shutdown();
    } 
    init(nh);

}


LaneDetection::LaneDetection(ros::NodeHandle *nh, string input_topic, string output_topic) :
    device_id_(UINT8_MAX),
    input_topic_(input_topic),
    output_topic_(output_topic),
    it_(*nh)
{
    // Subscribe to input image
    std::string topic = nh->resolveName(input_topic_);
    input_sub_ = it_.subscribeCamera(topic, 1, &LaneDetection::readImage, this);
    init(nh);
}


void LaneDetection::init(ros::NodeHandle *nh)
{
    output_pub_ = it_.advertise(output_topic_, 1);
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

    // Get camera model that produced the ROS image msg
    cam_model_.fromCameraInfo(info_msg);
}


void LaneDetection::quantize()
{
    // Convert to float & reshape to a [3 x W*H] Mat 
    // (so every pixel is on a row of it's own)
    Mat data;
    img_out_.convertTo(data, CV_32F);
    data = data.reshape(1, data.total());
    
    Mat labels, centers;
    double compactness = kmeans(
        data, 
        this->num_colors, 
        labels,
        TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0 ),
        1, 
        KMEANS_PP_CENTERS, 
        centers
    );

    // reshape both to a single row of Vec3f pixels:
    centers = centers.reshape(3, centers.rows);
    data = data.reshape(3, data.rows);

    // replace pixel values with their center value:
    Vec3f *p = data.ptr<Vec3f>();
    for (size_t i=0; i<data.rows; i++) {
        int center_id = labels.at<int>(i);
        p[i] = centers.at<Vec3f>(center_id);
    }

    // back to 2d, and uchar:
    img_out_ = data.reshape(3, img_out_.rows);
    img_out_.convertTo(img_out_, CV_8U);
}


void LaneDetection::update()
{
    if (input_topic_.empty()) 
    {
        readImage();
    }
    if (!img_src_.empty())
    {
        resize(img_src_, img_out_, Size(), scale, scale);
        cvtColor(img_out_, img_out_, COLOR_BGR2HSV);
        quantize();
        projectToGrid();
    } else
    {
        ROS_INFO_THROTTLE(2.0, "Waiting for input topic %s", input_topic_.c_str());
    }
}


void LaneDetection::projectToGrid()
{
    Size image_size = img_out_.size();
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
    Mat H(K * (T * (R * A1)));

    warpPerspective(img_out_, img_out_, H, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
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
        output_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out_).toImageMsg();
        output_pub_.publish(output_msg_);
    }
}
