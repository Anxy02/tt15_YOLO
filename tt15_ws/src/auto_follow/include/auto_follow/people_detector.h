#ifndef PEOPLE_DETECTOR_H
#define PEOPLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>

class PeopleDetector
{
public:
	PeopleDetector(){}
	~PeopleDetector(){}
	bool initialize(ros::NodeHandle& nh);
	void laserFilterMean( std::vector <double> *vector_r, unsigned size );
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& msg);
	void findPattern( std::string str, std::string pattern, std::list <int> *element_found );
	void validatePattern( std::list <int> *Pattern_list, int TYPE,	std::vector <int> flank_id0,	std::vector <int> flank_id1, std::vector <double> laser_x, std::vector <double> laser_y);
	void humanPose( std::vector <double> *r_x, std::vector <double> *r_y, std::list <int> Pattern_list, int TYPE,	std::vector <int> flank_id0,	std::vector <int> flank_id1, std::vector <double> laser_x, std::vector <double> laser_y );
	void validateDistance();
	void humanPoseInit();
	void poseFilter();
	void run();
	inline double dist2D( double x0, double y0, double x1, double y1 )
	{
		return sqrt(pow(x0-x1, 2)+pow(y0-y1, 2));
	}

private:
	bool initialized_ = false;
	ros::NodeHandle nh_;
	ros::Publisher	node_pub_;
	ros::Publisher	pos_pub_;
	ros::Subscriber node_sub_;
	std::string laser_topic_ = "/olelidar/scan";
	double ANTRO_a0_ = 0.02;
	double ANTRO_a1_ = 0.2;
	double init_x_;
	double init_y_;
	int seq_counter_ = 0;
	bool sensor_on_ = false;
	bool init_done_ = false;
	std::vector <double> rec_x_;
	std::vector <double> rec_y_;
	std::vector <double> now_pos_;
	std::vector <double> pub_pos_;
	std::string sensor_frame_id_;
	boost::mutex rec_mutex_;
	double max_range_ = 20;
	ros::Time lost_start_t_;
	double duration_t_ = 2;
	bool lost_pos_ = false;
	bool need_to_reinit_;
};

#endif	//PEOPLE_DETECTOR_H