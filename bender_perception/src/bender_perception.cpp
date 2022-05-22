#include <bender_perception/vision.h>
#include <bender_perception/lidar.h>


int main (int argc, char **argv)
{
    /* Initialize ROS node and load publish rate param */
    ros::init(argc, argv, "bender_perception_vision");
    ros::NodeHandle nh("~");
    int publish_rate; nh.param("publish_rate", publish_rate, 10);
    
    /* Image processing parameters */
    double scale; nh.param("scale", scale, 1.0);
    if (!(scale > 0.0 && scale <= 1.0)) 
    {
        ROS_WARN("The parameter `scale' must be in the range (0,1]. Got %.1f, ignoring and using `scale' = 1.0", scale);
        scale = 1.0;
    }
    int num_colors; nh.param("num_colors", num_colors, 2);
    if (!(num_colors > 0 && num_colors <= 255)) 
    {
        ROS_WARN("The parameter `num_colors' must be in the range [2,255]. Got %d, ignoring and using `num_colors' = 2", num_colors);
        num_colors = 2;
    }

    /* Image processing class */
    LaneDetection ld(nh, "/bender_camera/image_raw");
    ld.scale = scale;
    ld.num_colors = num_colors;

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