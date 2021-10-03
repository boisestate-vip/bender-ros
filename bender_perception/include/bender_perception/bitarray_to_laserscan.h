#ifndef BITARRAY_TO_LASERSCAN
#define BITARRAY_TO_LASERSCAN

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace bitarray_to_laserscan
{

class BitArrayToLaserScan
{
	public:
		BitArrayToLaserScan();
		~BitArrayToLaserScan();

		/**
		 * @brief Converts the information in a sensor_msgs::Image to a sensor_msgs::LaserScan.
		 * 
		 * @param image_msg a sensor_msgs::Image binary image
		 * @param info_msg CameraInfo associated with image_msg
		 * @return sensor_msgs::LaserScanPtr containing the info presented in sensor_msgs
		 * 
		 */
		sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& image_msg,
																					const sensor_msgs::CameraInfoConstPtr& info_msg);

		/**
		 * Sets the scan time parameter.
		 *
		 * This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as
		 * "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
		 * left to the user to set correctly.
		 *
		 * @param scan_time The value to use for outgoing sensor_msgs::LaserScan.
		 *
		 */
		void set_scan_time(const float scan_time);

		/**
		 * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
		 *
		 * range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
		 * angular increment.  range_max is used to set the output message.
		 *
		 * @param range_min Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
		 * @param range_max Maximum range to use points in the output scan.
		 *
		 */
		void set_range_limits(const float range_min, const float range_max);

		/**
		 * Sets the frame_id for the output LaserScan.
		 *
		 * Output frame_id for the LaserScan.  
		 *
		 * @param output_frame_id Frame_id to use for the output sensor_msgs::LaserScan.
		 *
		 */
		void set_output_frame(const std::string& output_frame_id);

	private:
		image_geometry::PinholeCameraModel cam_model_;
		
		double scan_time_;
		double range_min_;
		double range_max_;
		
		std::string output_frame_id_;

}; // class BitArrayToLaserScan

}; // namespace bitarray_to_laserscan

#endif // BITARRAY_TO_LASERSCAN
