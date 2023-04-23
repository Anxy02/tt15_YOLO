#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h> 
#include <sensor_msgs/Joy.h>
#include <iostream>

using namespace std;

int main (int argc,char** argv)
{
	ros::init(argc,argv, "response_test");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Rate loop_rate(25);
	double v_x = 0;
	double dv_x = 0.2;
	double omega = 0;
	int count = 0;
	bool flag = false;
	while (ros::ok())
	{
		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		geometry_msgs::Twist v;

		v.linear.x = v_x;
		v.angular.z = omega;

		pub.publish(v);
		if(flag)
		{
			if(count > 200 && v_x > 0.0001)
			{
				// v_x = 0.4;
				// omega = 0.4;
				v_x -= dv_x;
				count -= 100;
			}
		}
		else
		{
			if(count > 200 && v_x < 0.7-0.0001)
			{
				// v_x = 0.4;
				// omega = 0.4;
				v_x += dv_x;
				count -= 100;
			}
			else if(v_x > 0.6 - 0.01)
			{
				flag = true;
			}
		}
		count++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
