#ifndef BENDER_PERCEPTION_VISION_H
#define BENDER_PERCEPTION_VISION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class LaneDetection
{
    public:
        LaneDetection(ros::NodeHandle *nh, int device_id=0);
        LaneDetection(ros::NodeHandle *nh, string input_topic);
        ~LaneDetection();
        void inputCb(const sensor_msgs::ImageConstPtr &msg);
        void readImage();
        void findKMeans(const int k, vector<Point3f> *colors);
        void projectToGrid();

    private:

        Mat img_src_;
        Mat img_out_;
        VideoCapture cam_capture_;
        const uint8_t device_id_;
        const string wname_ = "Quantized Image";

        ros::Subscriber input_sub_;
        const string input_topic_;
}; 


#endif // BENDER_PERCEPTION_VISION_H