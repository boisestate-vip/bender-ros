#include <bender_perception/vision.h>
#include <bender_perception/lidar.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh;
    LaneDetection ld = LaneDetection(&nh, 0);
    BarrelDetection bd = BarrelDetection(&nh);
    
    ros::Rate rate(50);
    while (nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}