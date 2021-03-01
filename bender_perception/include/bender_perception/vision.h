#ifndef BENDER_PERCEPTION_VISION_H
#define BENDER_PERCEPTION_VISION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

class LaneDetection
{
    public:
        /*
         * Constructor for reading from USB Camera
         */
        LaneDetection(ros::NodeHandle *nh, int device_id=0);


        /*
         * Constructor for reading from ros topic `input_topic_`
         */
        LaneDetection(ros::NodeHandle *nh, string input_topic);


        /*
         * Desctructor
         */
        ~LaneDetection();


        /*
         * Update image source from incoming msg
         */
        void readImage(const sensor_msgs::ImageConstPtr &msg);


        /*
         * Update image source by reading USB camera input
         */
        void readImage();


        /*
         * Compute the two colors to quantize to
         */
        void quantize(const int k);


        /*
         * Update the output with latest source
         */
        void update();


        /*
         * Perform perspective transform 
         */
        void projectToGrid();


        /*
         * Display output in GUI window
         */
        void displayOutput();

    private:

        VideoCapture cam_capture_;
        const uint8_t device_id_;
        const string wname_ = "bender_perception_vision";

        ros::Subscriber input_sub_;
        const string input_topic_;

        Mat img_src_;
        Mat img_out_;
}; 


#endif // BENDER_PERCEPTION_VISION_H