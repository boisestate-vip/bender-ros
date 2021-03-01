#include <bender_perception/vision.h>
#include <bender_perception/lidar.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh;
    // LaneDetection ld = LaneDetection(&nh, 0);
    LaneDetection ld = LaneDetection(&nh, "/bender_camera/image");
    BarrelDetection bd = BarrelDetection(&nh);
    
    ros::Rate rate(5);
    while (nh.ok())
    {
        ld.update();
        ld.displayOutput();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}