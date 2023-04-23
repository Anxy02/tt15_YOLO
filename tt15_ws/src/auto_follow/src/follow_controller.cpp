#include <auto_follow/reactive_controller.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "follow_controller");
	ros::NodeHandle nh("~/follow_controller");
	ReactiveController reactive_controller;
	ROS_ERROR("reactive_controller.initialize(nh) begin");
	reactive_controller.initialize(nh);
	ROS_ERROR("reactive_controller.initialize(nh) end");

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}