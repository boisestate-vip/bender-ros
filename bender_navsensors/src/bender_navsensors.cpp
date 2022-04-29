#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>

mavros_msgs::State state;
void get_state(const mavros_msgs::State::ConstPtr& msg)
{
	state = *msg;
}

int main (int argc, char **argv)
{

	ros::init(argc, argv, "navsensors");
	ros::NodeHandle nh;

	ros::Subscriber state_listener = nh.subscribe<mavros_msgs::State>("mavros/state", 10, get_state);

	ros::ServiceClient arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	ros::Publisher armed_pub = nh.advertise<std_msgs::Bool>("bender_navsensors/state/enabled", 10);
	ros::Publisher conn_pub = nh.advertise<std_msgs::Bool>("bender_navsensors/state/connected", 10);

        ros::Rate loop_rate(20);

	mavros_msgs::CommandBool arm;
	arm.request.value = true;

	std_msgs::Bool armed;
	std_msgs::Bool conn;

	while (ros::ok())
	{
		armed.data = state.armed;
		conn.data = state.connected;

		armed_pub.publish(armed);
		conn_pub.publish(conn);

		if (!armed.data)
		{
			arming.call(arm);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

