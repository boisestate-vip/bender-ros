#ifndef BITARRAY_TO_LASERSCAN
#define BITARRAY_TO_LASERSCAN

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace bitarray_to_laserscan
{

/**
 * @brief Create a laser scan message from a binary image, using its camera info for geometry
 * Modifed from depthimage_to_laserscan package.
 */
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
		 * Sets the number of image rows to use in the output LaserScan.
		 *
		 * scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
		 * angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
		 * can be used to vertically compress obstacles into a single LaserScan.
		 *
		 * @param scan_height Number of pixels centered around the center of the image to compress into the LaserScan.
		 *
		 */
		void set_scan_height(const int scan_height);

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
		int scan_height_;
		
		std::string output_frame_id_;
		
		/**
		 * Computes euclidean length of a cv::Point3d (as a ray from origin)
		 *
		 * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
		 *
		 * @param ray The ray for which the magnitude is desired.
		 * @return Returns the magnitude of the ray.
		 *
		 */
		double magnitude_of_ray(const cv::Point3d& ray) const;


		/**
		 * Computes the angle between two cv::Point3d
		 *
		 * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
		 * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
		 *
		 * @param ray1 The first ray
		 * @param ray2 The second ray
		 * @return The angle between the two rays (in radians)
		 *
		 */
		double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;

		/**
		* Converts the image to a laserscan
		*
		* This uses a method to inverse project each pixel into a LaserScan angular increment.  
		* This method first projects the pixel forward into Cartesian coordinates, then calculates 
		* the range and angle for this point.  When multiple points coorespond to
		* a specific angular measurement, then the shortest range is used.
		*
		* @param image_msg The image message.
		* @param cam_model The image_geometry camera model for this image.
		* @param scan_msg The output LaserScan.
		*
		*/
		void convert(const sensor_msgs::ImageConstPtr& image_msg, const image_geometry::PinholeCameraModel& cam_model,
        			 const sensor_msgs::LaserScanPtr& scan_msg);

}; // class BitArrayToLaserScan

}; // namespace bitarray_to_laserscan

#endif // BITARRAY_TO_LASERSCAN
