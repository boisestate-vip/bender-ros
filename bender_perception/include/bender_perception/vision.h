#ifndef BENDER_PERCEPTION_VISION_H
#define BENDER_PERCEPTION_VISION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class LaneDetection
{
    public:
        LaneDetection();
        ~LaneDetection();

    private:
        void readImage();
        void applyMask(int convert_type=COLOR_BGR2HSV);
        void findKMeans(const int k, viz::Color *colors);
        void projectToGrid();
        

        Mat img_src_;
        VideoCapture cam_capture_;
        string wname_;
        int8_t grid_[];

}; 

#endif // BENDER_PERCEPTION_VISION_H