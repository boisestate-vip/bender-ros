#include <opencv2/opencv.hpp>
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
    input_sub_ = nh->subscribe(input_topic_, 1, &LaneDetection::inputCb, this);
}


LaneDetection::~LaneDetection()
{
}


void LaneDetection::readImage()
{
    ;
}


void LaneDetection::findKMeans(const int k, vector<Point3f> *colors)
{
    ;
}


void LaneDetection::inputCb(const sensor_msgs::ImageConstPtr &msg)
{
    ;
}


