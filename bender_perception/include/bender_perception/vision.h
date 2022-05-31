#ifndef BENDER_PERCEPTION_VISION_H
#define BENDER_PERCEPTION_VISION_H

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <bender_perception/bitarray_to_laserscan.h>
#include <bender_perception/BenderPerceptionConfig.h>

using namespace cv;
using namespace std;

typedef bender_perception::BenderPerceptionConfig VisionConfig;

class LaneDetection
{
    public:
        /*
         * Constructor for reading from USB Camera
         */
        LaneDetection(ros::NodeHandle &nh, int device_id=0);


        /*
         * Constructor for reading from rostopic `input_topic_`
         */
        LaneDetection(ros::NodeHandle &nh, string input_topic, string output_topic="/bender_perception/image_quantized");


        /*
         * Desctructor
         */
        ~LaneDetection();


        /*
         * Update image source from incoming msg
         */
        void readImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg);


        /*
         * Update image source by reading USB camera input
         */
        void readImage();

        /*
         * Apply color threshold to get rid of objects that are not roads
         */
        void applyColorThreshold();
        void autoThreshold();

        /*
         * Gamma correction for better contrast
         */
        void gammaCorrection();

        /*
         * Smooth the image to some degree by applying alternative morphological
         * closing and opening operations with an enlarging structuring element
         */
        void smooth();

        /*
         * Compute the two colors to quantize to
         */
        void quantize();

        /*
         * Turn quantized image with 2 colors into binary image
         * 
         */
        void toBinary();

        /*
         * Update the output with by processing the latest available source
         */
        void update();


        /*
         * Perform perspective transform 
         */
        void computeHomography();


        /*
         * Perform perspective transform 
         */
        void projectToGrid();


        /*
         * Display output in GUI window
         */
        void displayOutput();


        /*
         * Publish quantized image via image_transport to preserve bandwidth
         */
        void publishQuantized();


        /*
         * Generate contours over the detected lane
         */
        void generateContours();

        
        struct Params {
            double scale = 1;
            int blur_intensity = 5;
            double laser_dist_scale_x = 1.0;
            double laser_dist_scale_y = 1.0;
            double laser_dist_offset = 0.0;
            double gamma = 5.0;
            int num_colors = 2;
            int smooth_kernel_size = 5;
            int roi_from_top = 160;
            int roi_from_bot = 60;
            int color_thresh_lb[3] = {0, 0, 0};
            int color_thresh_ub[3] = {255, 255, 255};
            double projection_distance = 0.6;
            bool threshold_adaptive = true;
            bool threshold_lock = false;
            int adaptive_type = ADAPTIVE_THRESH_GAUSSIAN_C;
            int adaptive_block_size = 11;
            double adaptive_mean_subtract = 2.0;
            int threshold_type = THRESH_BINARY;
            int color_type = COLOR_BGR2HLS;
            int dilation_size = 4;
            int erosion_size = 4;
        } params;

    protected:
        void reconfigureCB(VisionConfig& config, uint32_t level);

    private:
        std::shared_ptr<dynamic_reconfigure::Server<VisionConfig>> dynamic_recfg_server_;
        boost::mutex config_mutex_;

        VideoCapture cam_capture_;
        const uint8_t device_id_;
        const string wname_ = "bender_perception_vision";

        image_transport::ImageTransport it_;

        image_transport::CameraSubscriber input_sub_;
        const string input_topic_;

        image_transport::Publisher output_pub_;
        const string output_topic_;
        sensor_msgs::ImagePtr output_msg_; 

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        image_geometry::PinholeCameraModel cam_model_;

        Mat img_src_;
        Mat img_out_;
        Mat labels_, centers_;
        bool has_centers_ = false;
        double threshold_val_ = 255;
        
        bool has_homography_ = false;
        Matx33d H_;     // Homography matrix computed from extrinsic and intrinsic parameters

        void init(ros::NodeHandle &nh);

        bitarray_to_laserscan::BitArrayToLaserScan btl_;
        ros::Publisher scan_pub_;
}; 


#endif // BENDER_PERCEPTION_VISION_H