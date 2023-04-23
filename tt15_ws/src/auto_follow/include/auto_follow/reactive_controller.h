#ifndef REACTIVE_CONTROLLER_H
#define REACTIVE_CONTROLLER_H

#include <auto_follow/collision_avoidance/configuration_layer.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <mutex>

class ReactiveController
{
public:
	ReactiveController();
	void initialize(ros::NodeHandle& nh);
	~ReactiveController(){}
	bool checkVelocityFeasibility(float v, float w);
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel, geometry_msgs::Twist current_cmd_vel);
	void computeVelocityCommandsWithDWA(geometry_msgs::Twist& cmd_vel, geometry_msgs::Twist current_cmd_vel);
	void updateGoalSituation();
	// void updateSafetySituation();
	void scanCache(const sensor_msgs::LaserScan& msg);
	void scanCallBack(const sensor_msgs::LaserScan& msg);
	void peoplePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void odomCallback(const nav_msgs::Odometry& msg);
	// void setCmdVel(const geometry_msgs::Twist cmd_vel);
	// void setCurrentVel(const geometry_msgs::Twist cmd_vel);
	// void publishSubgoal();
	void publishCostmap();
	inline int getIndexInCostlist(int index_v, int index_w)
	{
		return index_v * w_size_ + index_w;
	}

	inline void indexToVW(int index, int& index_v, int& index_w)
	{
		index_v = index / w_size_;
		index_w = index - (index_v * w_size_);
	}
	inline double dist2D(double x0, double y0, double x1, double y1)
	{
		return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
	}

private:
	bool initialized_ = false;
	bool got_new_goal_ = false;
	bool visualization_ = true;
	ros::Subscriber odom_sub_;
	ros::Subscriber scan_sub_;
	ros::Subscriber people_pos_sub_;
	ros::Publisher vel_pub_;
	ros::Publisher subgoal_pub_;
	ros::Publisher reachable_configurations_pub_;
	ros::Publisher obstacle_point_cloud_pub_;
	ros::Publisher costmap_boundary_pub_;
	ros::Publisher goal_l_alpha_point_pub_;
	ros::Publisher collision_l_alpha_pub_;
	float goal_l_, goal_alpha_;
	float subgoal_l_, subgoal_alpha_;
	float collision_l_, collision_alpha_;
	geometry_msgs::PointStamped goal_l_alpha_point_;
	geometry_msgs::PointStamped collision_l_alpha_;
	nav_msgs::Path costmap_boundary_;
	float inscribed_radius_;
	float D_safe_;
	bool scan_cache_flag_;
	float max_range_;		// 激光雷达的最大范围
	float angle_min_;
	float angle_increment_;
	int sample_num_;
	float local_max_range_ = 4;
	float controller_frequency_ = 20;
	float min_turning_radius_ = 0.98;
	float base_to_scan_dist_ = 0.852;
	float safety_margin_ = 0.05;
	float min_w_ = -0.4;
	float max_w_ = 0.4;
	float min_v_ = -0.4;//todo: 调大
	float max_v_ = 0.4;
	float max_acc_theta_ = 0.2;
	float max_dec_theta_ = 0.2;
	float max_acc_x_ = 0.2;
	float max_dec_x_ = 0.2;
	float v_size_ = 40;
	float w_size_ = 40;
	float dt_ = 0.6;
	float alpha_cost_factor = 50.0;
	float l_cost_factor = 10.0;
	float v_cost_factor = 5;
	std::string command_topic_ = "/tt15_cmd_vel";
	std::vector<float> v_list_;
	std::vector<float> w_list_;
	std::vector<float> cmd_cost_list_;
	std::vector<float> rho_list_;
	std::vector<float> theta_list_;
	std::vector<float> cos_theta_list_;
	std::vector<float> sin_theta_list_;
	sensor_msgs::PointCloud reachable_configurations_;
	int seq_ = 0;
	ConfigurationLayer configuration_layer_;
	geometry_msgs::PointStamped goal_point_in_base_;
	geometry_msgs::PoseStamped goal_pose_in_global_;
	geometry_msgs::Twist current_vel_;
	geometry_msgs::Twist latest_vel_;
	geometry_msgs::Twist zero_vel_;
	std::mutex vel_mutex_;
	std::mutex current_vel_mutex_;
	LaserPoint goal_point_;
	LaserPoint subgoal_point_;
	// LaserPoint virtual_goal_point_;
	// float alpha_;
	// float beta_;
	// float phi_vg_;
    GoalSituation goal_situation_;
    SubgoalSituation subgoal_situation_;
    SafetySituation safety_situation_;
    CollisionSituation collision_situation_;
	double Kp_l_ = 0.4;
	double Kp_a_ = 0.38;
	double dist_threshold_ = 0.5;
	double init_theta_ = 0;
};

#endif	//REACTIVE_CONTROLLER_H