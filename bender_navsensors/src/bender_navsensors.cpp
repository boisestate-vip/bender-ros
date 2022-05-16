#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

mavros_msgs::State state;
double lat, longi;
double init_lat, init_long;
double x_dist, y_dist;
bool init;
int coord_conv_factor = 111139;

void get_state(const mavros_msgs::State::ConstPtr& msg)
{
    state = *msg;
}

void get_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
      lat = *msg.latitude.data;
      longi = *msg.longitude.data;

      if (!init)
      {
          init_lat = lat;
          init_long = longi;
          init = true;
          return;
      }

      // Math here to turn current lat/long and initial value into meters, save into x_dist and y_dist
	x_dist = (abs(lat) - abs(init_lat)) * coord_conv_factor;;
	y_dist = (abs(longi) - abs(init_long)) * coord_conv_factor;
}      

int main (int argc, char **argv)
{

        init = false;

    ros::init(argc, argv, "navsensors");
    ros::NodeHandle nh;

    ros::Subscriber state_listener = nh.subscribe<mavros_msgs::State>("mavros/state", 10, get_state);
        ros::Subscriber gps_listener = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, get_gps);

    ros::ServiceClient arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::Publisher armed_pub = nh.advertise<std_msgs::Bool>("bender_navsensors/state/enabled", 10);
    ros::Publisher conn_pub = nh.advertise<std_msgs::Bool>("bender_navsensors/state/connected", 10);
        ros::Publisher x_pub = nh.advertise<std_msgs::Float64>("bender_navsensors/position/x", 10);
        ros::Publisher y_pub = nh.advertise<std_msgs::Float64>("bender_navsensors/position/y", 10);

        ros::Rate loop_rate(20);

    mavros_msgs::CommandBool arm;
    arm.request.value = true;

    std_msgs::Bool armed;
    std_msgs::Bool conn;
        std_msgs::Float64 x_msg;
        std_msgs::Float64 y_msg;
	
	while (ros::ok())
      {
          armed.data = state.armed;
       conn.data = state.connected;
                  x_msg.data = x_dist;
                  y_msg.data = y_dist;
 
          armed_pub.publish(armed);
          conn_pub.publish(conn);
                  x_pub.publish(x_msg);
                  y_pub.publish(y_msg);
          if (!armed.data)
          {
              arming.call(arm);
          }
 
          ros::spinOnce();
          loop_rate.sleep();
    }
 
      return 0;
 
 }
