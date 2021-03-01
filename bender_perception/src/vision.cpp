#include <bender_perception/vision.h>

LaneDetection::LaneDetection(ros::NodeHandle *nh, int device_id) :
    device_id_(device_id),
    input_topic_("")
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

}


LaneDetection::LaneDetection(ros::NodeHandle *nh, string input_topic) :
    device_id_(UINT8_MAX),
    input_topic_(input_topic)
{
    // Subscribe to input image
    input_sub_ = nh->subscribe(input_topic_, 1, &LaneDetection::readImage, this);

    // Wait until message is received
    int attempts = 0;
    while (img_src_.empty() && attempts < 5)
    {
        ROS_INFO("Waiting for %s to be published", input_topic_.c_str());
        ros::Duration(2.0).sleep();
        attempts++;
    }

    if (img_src_.empty())
    {
        ROS_ERROR("No message receieved via %s within 10 seconds. Aborting...", input_topic_.c_str());
        nh->shutdown();
    }

}


LaneDetection::~LaneDetection()
{
}


void LaneDetection::readImage()
{
    const float scale = 0.25;
    cam_capture_.read(img_src_);
    resize(img_src_, img_out_, Size(), scale, scale);
}


void LaneDetection::readImage(const sensor_msgs::ImageConstPtr &msg)
{
    ;
}


void LaneDetection::quantize(const int k)
{
    // Convert to float & reshape to a [3 x W*H] Mat 
    // (so every pixel is on a row of it's own)
    Mat data;
    img_out_.convertTo(data, CV_32F);
    data = data.reshape(1, data.total());
    
    Mat labels, centers;
    double compactness = kmeans(
        data, 
        k, 
        labels,
        TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
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
    cvtColor(img_out_, img_out_, COLOR_BGR2HSV);
    quantize(2);
}


void LaneDetection::displayOutput()
{
    Mat out, display;
    resize(img_out_, out, img_src_.size());
    hconcat(img_src_, out, display);
    namedWindow(wname_, WINDOW_AUTOSIZE);
    imshow(wname_, display);
    waitKey(1);
}

