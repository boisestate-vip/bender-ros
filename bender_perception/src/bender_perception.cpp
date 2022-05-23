#include <bender_perception/vision.h>
#include <bender_perception/lidar.h>


int main (int argc, char **argv)
{
    /* Initialize ROS node and load publish rate param */
    ros::init(argc, argv, "bender_perception_vision");
    ros::NodeHandle nh("~");
    int publish_rate; nh.param("publish_rate", publish_rate, 10);

    /* Image processing class */
    LaneDetection ld(nh, "/bender_camera/image_raw");
    
    /* Lidar processing class */
    BarrelDetection bd(&nh);
    
    /* Spin */
    ros::Rate rate(publish_rate);
    while (nh.ok())
    {
        ld.update();
        // ld.displayOutput();
        ld.publishQuantized();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}