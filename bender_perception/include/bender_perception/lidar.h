#ifndef BENDER_PERCEPTION_LIDAR_H
#define BENDER_PERCEPTION_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/*
 * See the following URL for tutorials on
 * writing publisher and subscriber in C++
 * as well as setting up custom msg types
 * and dynamic reconfigure.
 * http://wiki.ros.org/ROSNodeTutorialC++
 */

using namespace std;

class BarrelDetection
{
    public:
        BarrelDetection(ros::NodeHandle *nh);
        ~BarrelDetection();

    private:        
        ros::Subscriber input_sub_;
        const string input_topic_;
}; // class BarrelDetection
    

#endif // BENDER_PERCEPTION_LIDAR_H