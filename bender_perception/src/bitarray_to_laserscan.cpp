#include <bender_perception/bitarray_to_laserscan.h>

namespace bitarray_to_laserscan
{

BitArrayToLaserScan::BitArrayToLaserScan()
  : scan_time_(1./30.)
  , range_min_(0.5)
  , range_max_(50.0)
  , scan_height_(1)
{
}

BitArrayToLaserScan::~BitArrayToLaserScan()
{
}

void BitArrayToLaserScan::set_scan_time(const float scan_time)
{
    scan_time_ = scan_time;
}

void BitArrayToLaserScan::set_range_limits(const float range_min, const float range_max)
{
    range_min_ = range_min;
    range_max_ = range_max;
}

void BitArrayToLaserScan::set_scan_height(const int scan_height)
{
    scan_height_ = scan_height;
}

void BitArrayToLaserScan::set_output_frame(const std::string& output_frame_id)
{
    output_frame_id_ = output_frame_id;
}

double BitArrayToLaserScan::magnitude_of_ray(const cv::Point3d& ray) const 
{
    return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

double BitArrayToLaserScan::angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const
{
    const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
    const double magnitude1 = magnitude_of_ray(ray1);
    const double magnitude2 = magnitude_of_ray(ray2);;
    return acos(dot_product / (magnitude1 * magnitude2));
}

sensor_msgs::LaserScanPtr BitArrayToLaserScan::convert_msg(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cam_model_.fromCameraInfo(info_msg);

    // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
    cv::Point2d raw_pixel_left(0, cam_model_.cy());
    cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
    cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

    cv::Point2d raw_pixel_right(image_msg->width-1, cam_model_.cy());
    cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
    cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

    cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
    cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
    cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

    const double angle_max = angle_between_rays(left_ray, center_ray);
    const double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image

    // Fill in laserscan message
    sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
    scan_msg->header = image_msg->header;
    scan_msg->header.stamp = ros::Time::now();
    if (output_frame_id_.length() > 0) {
        scan_msg->header.frame_id = output_frame_id_;
    }
    scan_msg->angle_min = angle_min;
    scan_msg->angle_max = angle_max;
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (image_msg->width - 1);
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_min = range_min_;
    scan_msg->range_max = range_max_;

    // Calculate and fill the ranges
    const uint32_t ranges_size = image_msg->width;
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
    // scan_msg->intensities.assign(ranges_size, 0.0);
    if (image_msg->encoding == "8UC1" | image_msg->encoding == "mono8")
    {
        convert(image_msg, cam_model_, scan_msg);
    }

    return scan_msg;
}

void BitArrayToLaserScan::convert(const sensor_msgs::ImageConstPtr& image_msg, 
    const image_geometry::PinholeCameraModel& cam_model,
    const sensor_msgs::LaserScanPtr& scan_msg)
{
    // Use correct principal point from calibration
    const int origin_x = image_msg->width / 2;
    const int origin_y = image_msg->height;

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    const double unit_scaling = 12.0;  // mm to m
    const float constant_x = unit_scaling / cam_model.fx();

    const uint8_t* this_row = reinterpret_cast<const uint8_t*>(&image_msg->data[0]);
    const int row_step = image_msg->step / sizeof(uint8_t);

    for (int v=0; v<(int)image_msg->height; ++v, this_row+=row_step)
    {
        for (int u=0; u<(int)image_msg->width; ++u) // Loop over each pixel in row
        {
            const uint8_t pixelval = this_row[u];
            if (pixelval == 0)
            {
                continue;
            }
            const double x = -u + origin_x;
            const double y = -v + origin_y;
            const double th = atan2(x,y);
            const double r = hypot(x,y) * constant_x;
            const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
            // std::cout << "index = " << std::to_string(index) << std::endl;
            // std::cout << "(u,v) = (" << u << "," << v << "), theta = " << std::to_string(th*180.0/M_PI) << std::endl;
            if (scan_msg->angle_max >= th && th >= scan_msg->angle_min && r >= range_min_) {
                const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
                scan_msg->ranges[index] = r;
            }
        }
    }
    // for (auto& range : scan_msg->ranges)
    // {
    //     if (range == std::numeric_limits<float>::infinity())
    //     {
    //         range = 40.0;
    //     }
    //     continue;
    // }


    // const int offset = (int)(center_y - scan_height_/2);
    // depth_row += offset*row_step; // Offset to center of image

    // for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step) 
    // {
    //     for (int u = 0; u < (int)image_msg->width; ++u) // Loop over each pixel in row
    //     {
    //         const uint8_t depth = depth_row[u];

    //         double r = depth; // Assign to pass through NaNs and Infs
    //         const double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
    //         const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
    //         // if (depth != 0) // Not NaN or Inf
    //         // { 
    //         //     // Calculate in XYZ
    //         //     double x = (u - center_x) * depth * constant_x;
    //         //     double z = depth * 0.001;
    //         //     // Calculate actual distance
    //         //     r = hypot(x, z);
    //         // }
    //         if (index >= 0) {
    //             scan_msg->ranges[index] = 5.0;
    //         }
    //     }
    // }
}

}