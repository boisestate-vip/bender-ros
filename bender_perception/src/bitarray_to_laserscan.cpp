#include <bender_perception/bitarray_to_laserscan.h>

namespace bitarray_to_laserscan
{

BitArrayToLaserScan::BitArrayToLaserScan()
  : scan_time_(1./30.)
  , range_min_(0.45)
  , range_max_(10.0)
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

void BitArrayToLaserScan::set_output_frame(const std::string& output_frame_id)
{
    output_frame_id_ = output_frame_id;
}

sensor_msgs::LaserScanPtr BitArrayToLaserScan::convert_msg(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cam_model_.fromCameraInfo(info_msg);

    // Fill in laserscan message
    sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
    scan_msg->header = image_msg->header;
    if (output_frame_id_.length() > 0) {
        scan_msg->header.frame_id = output_frame_id_;
    }

    return scan_msg;
}

}