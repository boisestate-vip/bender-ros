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
    input_sub_ = it_.subscribe(input_topic_, 1, &LaneDetection::readImage, this);
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


void LaneDetection::readImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_src_ = cv_ptr->image;
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
    } else
    {
        ROS_INFO_THROTTLE(2.0, "Waiting for input topic %s", input_topic_.c_str());
    }
}


void LaneDetection::projectToGrid()
{
    ;
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
