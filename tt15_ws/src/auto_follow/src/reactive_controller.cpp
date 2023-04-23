#include <auto_follow/reactive_controller.h>
#include <chrono>

ReactiveController::ReactiveController()
{
}

void ReactiveController::initialize(ros::NodeHandle& nh)
{
	std::vector<geometry_msgs::Point> footprint;
	footprint.resize(4);
	footprint[0].x = 0.33;
	footprint[0].y = 0.26;
	footprint[1].x = -0.33;
	footprint[1].y = 0.26;
	footprint[2].x = -0.33;
	footprint[2].y = -0.26;
	footprint[3].x = 0.33;
	footprint[3].y = -0.26;
	if(!initialized_)
	{
		nh.param("robot/footprint/p1_x", footprint[0].x, footprint[0].x);
		nh.param("robot/footprint/p1_y", footprint[0].y, footprint[0].y);
		nh.param("robot/footprint/p2_x", footprint[1].x, footprint[1].x);
		nh.param("robot/footprint/p2_y", footprint[1].y, footprint[1].y);
		nh.param("robot/footprint/p3_x", footprint[2].x, footprint[2].x);
		nh.param("robot/footprint/p3_y", footprint[2].y, footprint[2].y);
		nh.param("robot/footprint/p4_x", footprint[3].x, footprint[3].x);
		nh.param("robot/footprint/p4_y", footprint[3].y, footprint[3].y);
		nh.param("robot/base_to_scan_dist", base_to_scan_dist_, base_to_scan_dist_);
		nh.param("robot/max_v", max_v_, max_v_);
		nh.param("robot/min_v", min_v_, min_v_);
		nh.param("robot/max_w", max_w_, max_w_);
		nh.param("robot/min_w", min_w_, min_w_);
		nh.param("robot/max_acc_x", max_acc_x_, max_acc_x_);
		nh.param("robot/max_dec_x", max_dec_x_, max_dec_x_);
		nh.param("robot/max_acc_theta", max_acc_theta_, max_acc_theta_);
		nh.param("robot/max_dec_theta", max_dec_theta_, max_dec_theta_);
		nh.param("robot/command_topic", command_topic_, command_topic_);
		nh.param("robot/Kp_l", Kp_l_, Kp_l_);
		nh.param("robot/Kp_a", Kp_a_, Kp_a_);
		nh.param("robot/dist_threshold", dist_threshold_, dist_threshold_);
		nh.param("robot/init_theta", init_theta_, init_theta_);
		// ROS_ERROR("configuration_layer_.initialize(nh, footprint) begin");
		configuration_layer_.initialize(nh, footprint);
		// ROS_ERROR("configuration_layer_.initialize(nh, footprint) end");
		inscribed_radius_ = 0.4;
		D_safe_ = 1;
		scan_cache_flag_ = false;
		v_list_.resize(v_size_);
		w_list_.resize(w_size_);
		cmd_cost_list_.resize(v_size_*w_size_);
		reachable_configurations_.points.resize(v_size_*w_size_);
	    double centre_x = 0;
	    double centre_y = 0;
	    //radius
	    double R=4;//TODO: check
	    double th = 0.0;
	    double delta_th = 0.1;
	    costmap_boundary_.header.frame_id = "base_link";
		goal_l_alpha_point_.header.frame_id = "base_link";
		collision_l_alpha_.header.frame_id = "base_link";
	    geometry_msgs::PoseStamped boundary_pose_stamped;
	    boundary_pose_stamped.header.frame_id="base_link";
	    goal_point_in_base_.header.frame_id="base_link";
		goal_point_.setPointXY(0, 0+base_to_scan_dist_, 0);
    	updateGoalSituation();
		goal_pose_in_global_.header.frame_id="odom";
		reachable_configurations_.header.frame_id = "base_link";

		latest_vel_.linear.x = latest_vel_.linear.y = latest_vel_.angular.z = 0;
		current_vel_.linear.x = current_vel_.linear.y = current_vel_.angular.z = 0;
		zero_vel_.linear.x = zero_vel_.linear.y = zero_vel_.angular.z = 0;

	    while (th < 14)
	    {
	        //compute the coordinates
	        double dt = 0.1;
	        double x =centre_x+R*sin(th);
	        double y = centre_y+R*cos(th);
	        th += delta_th;

	        boundary_pose_stamped.pose.position.x = x;
	        boundary_pose_stamped.pose.position.y = y;

	        costmap_boundary_.poses.push_back(boundary_pose_stamped);
	    }
	    reachable_configurations_pub_ = nh.advertise<sensor_msgs::PointCloud>("/reachable_configurations", 1000, true);
		costmap_boundary_pub_ = nh.advertise<nav_msgs::Path>("/costmap_boundary", 1000, true);
		obstacle_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("/obstacle_point_cloud", 1000, true);
		goal_l_alpha_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/goal_l_alpha", 1000, true);
		collision_l_alpha_pub_ = nh.advertise<geometry_msgs::PointStamped>("/collision_l_alpha", 1000, true);
	    subgoal_pub_ = nh.advertise<visualization_msgs::Marker>("/subgoal", 1000, true);
		vel_pub_ = nh.advertise<geometry_msgs::Twist>(command_topic_, 1000, true);
		scan_sub_ = nh.subscribe("/olelidar/scan", 1, &ReactiveController::scanCallBack, this);
		odom_sub_ = nh.subscribe("/encoder_odom", 1, &ReactiveController::odomCallback, this);	
		people_pos_sub_ = nh.subscribe("/people_pos", 1, &ReactiveController::peoplePosCallback, this);
		initialized_ = true;
	}
}

// void ReactiveController::setCmdVel(const geometry_msgs::Twist cmd_vel)
// {
// 	std::unique_lock<std::mutex> lck (vel_mutex_);
// 	latest_vel_ = cmd_vel;
// }

// void ReactiveController::setCurrentVel(const geometry_msgs::Twist cmd_vel)
// {
// 	current_vel_ = cmd_vel;
// }

bool ReactiveController::checkVelocityFeasibility(float v, float w)
{
	float epsilon = 0.000001;
	float l = abs(v*dt_);
	float r, alpha;
	if(abs(w) < epsilon)
	{
		r = 1000000;
		if(v >= 0)
			alpha = 0;
		else
			alpha = M_PI;
	}
	else
	{
		r = v/w;
		alpha = atan2(w,v);
	}

	if(configuration_layer_.checkFeasibility(l, alpha))
	{
		// 按照当前速度在下一控制周期内无碰，还需确保在遇到紧急情况时以最大加速度能够停下
		float l_un = abs(v*dt_)+abs((v*v)/(2*max_dec_x_));
		float alpha_un = alpha;
		if(configuration_layer_.checkFeasibility(l_un, alpha_un))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}	
}

void ReactiveController::updateGoalSituation()
{
	configuration_layer_.transformToLalpha(goal_point_.x, goal_point_.y, goal_l_, goal_alpha_);
	goal_l_alpha_point_.point.x = goal_l_*cos(goal_alpha_);
	goal_l_alpha_point_.point.y = goal_l_*sin(goal_alpha_);
	subgoal_point_ = goal_point_;
	subgoal_l_ = goal_l_;
	subgoal_alpha_ = goal_alpha_;
	if(configuration_layer_.checkFeasibility(goal_l_, goal_alpha_, collision_l_, collision_alpha_))
	{
		goal_situation_ = GoalSituation::FREE_GOAL;
		// std::cout << "goal_situation_: FREE_GOAL" << std::endl;
	}
	else
	{
    	collision_l_alpha_.point.x = collision_l_*cos(collision_alpha_);
    	collision_l_alpha_.point.y = collision_l_*sin(collision_alpha_);
		goal_situation_ = GoalSituation::DANGEROUS_GOAL;
		// std::cout << "goal_situation_: DANGEROUS_GOAL" << std::endl;
	}
}

void ReactiveController::computeVelocityCommandsWithDWA(geometry_msgs::Twist& cmd_vel, geometry_msgs::Twist current_cmd_vel)
{
	std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point time_current;
	std::chrono::duration<double> time_used;
	reachable_configurations_.header.seq = seq_++;
	geometry_msgs::Point32 reachable_point;

	float current_v = current_cmd_vel.linear.x;
	float current_w = current_cmd_vel.angular.z;
	float v_ub = std::min(max_v_, current_v+max_acc_x_*dt_);
	float v_lb = std::max(min_v_, current_v-max_dec_x_*dt_);
	float w_ub = std::min(max_w_, current_w+max_acc_theta_*dt_);
	float w_lb = std::max(min_w_, current_w-max_dec_theta_*dt_);
	float delta_v = (v_ub-v_lb)/v_size_;
	float delta_w = (w_ub-w_lb)/w_size_;
	ROS_INFO_STREAM("v_ub:" << v_ub << "  v_lb:" << v_lb);
	float v, w, l, r, alpha;
	float epsilon = 0.000001;
	int cmd_cost_size = v_size_*w_size_;
	for(int i = 0; i < cmd_cost_size; i++)
	{
		cmd_cost_list_[i] = 999999;
	}

	// std::cout << "subgoal_l_: " << subgoal_l_ << ", subgoal_alpha_: " << subgoal_alpha_ << std::endl;

	for(int i = 0; i < v_size_; i++)
	{
		std::cout << i;
		v = v_lb+i*delta_v;
		v_list_[i] = v;
		for(int j = 0; j < w_size_; j++)
		{
			w = w_lb+j*delta_w;
			w_list_[j] = w;
			l = abs(v*dt_);
			if(abs(w) < epsilon)
			{
				r = 1000000;
				if(v >= 0)
					alpha = 0;
				else
					alpha = M_PI;
			}
			else
			{
				r = v/w;
				alpha = atan2(w,v);
			}

			// 计算每个(v,w)组合的代价
			int index = getIndexInCostlist(i,j);
			float obs_cost, alpha_cost, l_cost,v_cost;
			// if(configuration_layer_.checkFeasibility(l, alpha))
			if(true)
			{
				// 按照当前速度在下一控制周期内无碰，还需确保在遇到紧急情况时以最大加速度能够停下
				float l_un = abs(v*dt_)+abs((v*v)/(2*max_dec_x_));
				float alpha_un = alpha;
				// if(configuration_layer_.checkFeasibility(l_un, alpha_un))
				if(true)
				{
					obs_cost = static_cast<int>(configuration_layer_.getCost(l, alpha))*1.0/255;
					l_cost = subgoal_l_-l*cos(alpha-subgoal_alpha_);
					alpha_cost = abs(alpha-subgoal_alpha_);
					v_cost = max_v_-v;
					cmd_cost_list_[index] = v_cost_factor * v_cost + obs_cost + l_cost_factor * l_cost + alpha_cost_factor * alpha_cost;
					reachable_point.z = cmd_cost_list_[index];
					// std::cout << "now_cost: " << cmd_cost_list_[index] << " ; now_v: "<<v <<"now_w: "<<w<< std::endl;
				}
				else
				{
					std::cout << "pengzhuang" << i <<std::endl;
					reachable_point.z = 100;
				}
			}
			else
			{
				// cmd_cost_list_[index] = __DBL_MAX__;
				reachable_point.z = 100;
			}
			// std::cout << "(v,w): (" << v << ", " << w << ")";
			// std::cout << ", obs_cost: " << obs_cost;
			// std::cout << ", alpha: " << alpha;
			// std::cout << ", l_cost: " << l_cost << ", alpha_cost: " << alpha_cost << ", cost: " << cmd_cost_list_[index] << std::endl;;
			reachable_point.x = l*cos(alpha);
			reachable_point.y = l*sin(alpha);
			reachable_configurations_.points[i*w_size_+j] = reachable_point;
		}
	}

	float min_cost = 999999;
	int min_index = -1;
	for(int i = 0; i < cmd_cost_size; i++)
	{
		if(cmd_cost_list_[i] < min_cost)
		{
			min_cost = cmd_cost_list_[i];
			min_index = i;
		}
	}
	if(min_cost > 999998)
	{
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
	}
	else
	{
		int index_v, index_w;
		indexToVW(min_index, index_v, index_w);
		cmd_vel.linear.x = v_list_[index_v];
		cmd_vel.angular.z = w_list_[index_w];
		// TODO: 
		{
			l = abs(cmd_vel.linear.x*dt_);
			alpha = atan2(cmd_vel.angular.z, cmd_vel.linear.x);
			double obs_cost_cout,l_cost_cout,alpha_cost_cout, cmd_cost_cout, v_cost_cout;
			obs_cost_cout = static_cast<int>(configuration_layer_.getCost(l, alpha))*1.0/255;
			l_cost_cout = subgoal_l_-l*cos(alpha-subgoal_alpha_);
			alpha_cost_cout = abs(alpha-subgoal_alpha_);
			v_cost_cout = max_v_ - cmd_vel.linear.x;
			cmd_cost_cout = obs_cost_cout + v_cost_cout * v_cost_factor + l_cost_factor * l_cost_cout + alpha_cost_factor * alpha_cost_cout;
			ROS_WARN_STREAM("cost[ obs_cost_cout:"<<obs_cost_cout<<" v_cost:"<< v_cost_cout * v_cost_factor<<"  l_cost:"<<(l_cost_factor * l_cost_cout)<<"  alpha_cost:"<<(alpha_cost_factor * alpha_cost_cout)<<"   all cost:  "<<cmd_cost_cout<<" ]");
			ROS_INFO_STREAM("subgoal_L:" << subgoal_l_ << "  l:  "<< l);
		}
		// std::cout << "best (v,w): (" << v_list_[index_v] << ", " << w_list_[index_w] << ")\n";
	}
	time_current = std::chrono::steady_clock::now();
	time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time_current-time_start);
	// std::cout << "computeVelocityCommandsWithDWA() process time: " << time_used.count() << "s" << std::endl;
}


bool ReactiveController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel, geometry_msgs::Twist current_cmd_vel)
{
	float epsilon = 0.02;
	if(goal_point_.x < base_to_scan_dist_+dist_threshold_ && goal_point_.y < 0.2 && goal_point_.y > -0.2)
	{
    	cmd_vel.linear.x = 0;
    	cmd_vel.angular.z = 0;	
    	// ROS_INFO("reach goal");	
    	return true;	
	}

	else
	{
		// // ROS_ERROR("computeVelocityCommandsWithDWA");
		computeVelocityCommandsWithDWA(cmd_vel, current_cmd_vel);
		// // ROS_ERROR("computeVelocityCommandsWithDWA successfully!");
		return true;
	}
	
}

void ReactiveController::scanCache(const sensor_msgs::LaserScan& msg)
{
	max_range_ = msg.range_max;		
	angle_min_ = msg.angle_min;
	angle_increment_ = msg.angle_increment;
	sample_num_ = static_cast<int>(msg.ranges.size());
	rho_list_ = msg.ranges;
	theta_list_.resize(sample_num_);
	cos_theta_list_.resize(sample_num_);
	sin_theta_list_.resize(sample_num_);
	for(int i = 0; i < sample_num_; i++)
	{
		theta_list_[i] = angle_min_+i*angle_increment_;
		cos_theta_list_[i] = cos(theta_list_[i]);
		sin_theta_list_[i] = sin(theta_list_[i]);
	}
	scan_cache_flag_ = true;
}

void ReactiveController::scanCallBack(const sensor_msgs::LaserScan& msg)
{
	std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point time_current;
	std::chrono::duration<double> time_used;
	ros::Time t0 = ros::Time::now();
	if(!scan_cache_flag_)
	{
		scanCache(msg);
	}
	else
	{
		rho_list_ = msg.ranges;
	}
	
	std::vector<geometry_msgs::Point32> obstacle_list;
	geometry_msgs::Point32 obstacle;
	int sample_step = std::max(1,sample_num_/360);
	double epsilon = 0.001;
	for(int i = 0; i < sample_num_; i += sample_step)
	{
		if(rho_list_[i] <= epsilon)
			continue;
		else if(isnan(rho_list_[i]))
			continue;
		else if(isinf(rho_list_[i]))
			continue;
		else if(rho_list_[i] > local_max_range_)
			continue;
		else
		{
			obstacle.x = rho_list_[i]*cos_theta_list_[i]+base_to_scan_dist_;
			obstacle.y = rho_list_[i]*sin_theta_list_[i];
			obstacle_list.push_back(obstacle);
			// std::cout << "x: " << obstacle.x << ", y: " << obstacle.y << std::endl;
		}
	}
	// ROS_ERROR("configuration_layer_.updateCostmap");
	// std::cout << "obstacle_list.size(): " << obstacle_list.size() << std::endl;
	collision_situation_ = configuration_layer_.updateCostmap(obstacle_list);
	// if(CollisionSituation::NO_COLLISION == collision_situation_)
	// 	ROS_ERROR("NO_COLLISION");
	// else if(CollisionSituation::FORWARD_COLLISION == collision_situation_)
	// 	ROS_ERROR("FORWARD_COLLISION");
	// else if(CollisionSituation::BACKWARD_COLLISION == collision_situation_)
	// 	ROS_ERROR("BACKWARD_COLLISION");
	// ROS_ERROR("configuration_layer_.updateCostmap successfully!");
	obstacle_list.clear();
	std::vector<geometry_msgs::Point32>().swap(obstacle_list);

	geometry_msgs::Twist latest_cmd_vel;

	{
		std::unique_lock<std::mutex> lck (vel_mutex_);
		latest_cmd_vel = latest_vel_;
	}

	if(checkVelocityFeasibility(latest_cmd_vel.linear.x, latest_cmd_vel.angular.z))
	{
		// ROS_ERROR("latest_cmd_vel: (%.4f, %.4f)", latest_cmd_vel.linear.x, latest_cmd_vel.angular.z);
		geometry_msgs::Twist current_cmd_vel;
		{
			std::unique_lock<std::mutex> lck (current_vel_mutex_);
			current_cmd_vel = current_vel_;
		}

		if(latest_cmd_vel.linear.x-current_cmd_vel.linear.x > max_acc_x_)
		{
			latest_cmd_vel.linear.x = current_cmd_vel.linear.x+max_acc_x_;
		}
		else if(latest_cmd_vel.linear.x-current_cmd_vel.linear.x < -max_dec_x_)
		{
			latest_cmd_vel.linear.x = current_cmd_vel.linear.x-max_dec_x_;
		}
		
		vel_pub_.publish(latest_cmd_vel);
	}
	else
	{
		geometry_msgs::Twist current_cmd_vel;
		{
			std::unique_lock<std::mutex> lck (current_vel_mutex_);
			current_cmd_vel = current_vel_;
		}

		computeVelocityCommands(latest_cmd_vel, current_cmd_vel);
		ROS_INFO_STREAM("publish velocity: " << latest_cmd_vel.linear.x <<"  w: " << latest_cmd_vel.angular.z);
		// latest_cmd_vel.linear.x = latest_cmd_vel.linear.x *5;
		vel_pub_.publish(latest_cmd_vel);
		// vel_pub_.publish(zero_vel_);
	}
	// ROS_ERROR("publishCostmap");
	if(visualization_)
		publishCostmap();
	// ROS_ERROR("publishCostmap successfully!");

	// ros::Time t1 = ros::Time::now();
	// ros::Duration duration(t1-t0);
	// // ROS_ERROR("ReactiveController::scanCallBack process time in ms:");
	// std::cout << duration*1000 << std::endl;
	// time_current = std::chrono::steady_clock::now();
	// time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time_current-time_start);
	// std::cout << "process time: " << time_used.count()*1000 << "ms" << std::endl;
}

void ReactiveController::peoplePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	double posx,posy;
	double dis,ang;
	posx = msg->pose.position.x;
	posy = msg->pose.position.y;
    goal_point_.setPointXY(0, posx+base_to_scan_dist_, posy);
    updateGoalSituation();
	dis = dist2D(posx,posy,0,0);
	ang = atan2(posy,posx) - init_theta_;
	double vl = dis*Kp_l_;
	double va = ang*Kp_a_;

	if (vl > max_v_)
		vl = max_v_;

	if (va > max_w_)
		va = max_w_;

	if (dis < dist_threshold_ || fabs(ang) > 0.5)
		vl = 0;

	if (fabs(ang) < 0.1)
		va = 0;

	std::cout << "posx: " << posx << ", posy: " << posy << ", vl: " << vl
				<< ", va: " << va << std::endl;

	{
		std::unique_lock<std::mutex> lck (vel_mutex_);
		latest_vel_.linear.x = vl;
		latest_vel_.angular.z = va;
	}

}

void ReactiveController::odomCallback(const nav_msgs::Odometry& msg)
{
	double v = msg.twist.twist.linear.x;
	double w = msg.twist.twist.angular.z;
	// ROS_INFO("odomCallback");

	{
		std::unique_lock<std::mutex> lck (current_vel_mutex_);
		current_vel_.linear.x = v;
		current_vel_.angular.z = w;
	}
}

// void ReactiveController::updateSafetySituation()
// {
// 	int min_index = 0;
// 	float min_rho = 9999;
// 	for(int i = 0; i < sample_num_; i++)
// 	{
// 		if(rho_list_[i] < min_rho)
// 		{
// 			min_rho = rho_list_[i];
// 			min_index = i;
// 		}
// 	}
// 	if(min_rho >= inscribed_radius_+D_safe_)
// 	{
// 		safety_situation_ = HIGH_SAFETY;
// 	}
// 	else
// 	{
// 		safety_situation_ = LOW_SAFETY;
// 	}
// }

// void ReactiveController::publishSubgoal()
// {
//     visualization_msgs::Marker subgoal_line;
//     visualization_msgs::Marker subgoal_point;
//     visualization_msgs::Marker virtual_goal_line;
//     visualization_msgs::Marker virtual_goal_point;

//     virtual_goal_point.header.frame_id = subgoal_point.header.frame_id = subgoal_line.header.frame_id = virtual_goal_line.header.frame_id = "base_link";
//     virtual_goal_point.header.stamp = subgoal_point.header.stamp = subgoal_line.header.stamp = virtual_goal_line.header.stamp = ros::Time::now();
//     subgoal_point.ns = subgoal_line.ns = "subgoal";
//     virtual_goal_point.ns = virtual_goal_line.ns = "virtual_goal";
//     virtual_goal_point.action = subgoal_point.action = subgoal_line.action = virtual_goal_line.action = visualization_msgs::Marker::ADD;
//     virtual_goal_point.pose.orientation.w = subgoal_point.pose.orientation.w = subgoal_line.pose.orientation.w = virtual_goal_line.pose.orientation.w = 1.0;
//     subgoal_point.id = 0;
//     virtual_goal_point.id = 0;
//     subgoal_line.id = 2;
//     virtual_goal_line.id = 2;
//     virtual_goal_point.type = subgoal_point.type = visualization_msgs::Marker::POINTS;
//     subgoal_line.type = virtual_goal_line.type = visualization_msgs::Marker::LINE_LIST;
//     subgoal_point.scale.x = 0.05;
//     subgoal_point.scale.y = 0.05;
//     subgoal_point.color.g = 1.0;
//     subgoal_point.color.a = 1.0;
//     virtual_goal_point.scale.x = 0.05;
//     virtual_goal_point.scale.y = 0.05;
//     virtual_goal_point.color.b = 1.0;
//     virtual_goal_point.color.a = 1.0;
//     subgoal_line.scale.x = 0.05;
//     subgoal_line.color.b = 1.0;
//     subgoal_line.color.a = 1.0;
//     virtual_goal_line.scale.x = 0.05;
//     virtual_goal_line.color.r = 1.0;
//     virtual_goal_line.color.a = 1.0;
//     geometry_msgs::Point p;
//     p.x = subgoal_point_.x;
//     p.y = subgoal_point_.y;
//     p.z = 0;
//     subgoal_point.points.push_back(p);

//     subgoal_pub_.publish(subgoal_point);

//     goal_l_alpha_point_pub_.publish(goal_l_alpha_point_);
//     collision_l_alpha_pub_.publish(collision_l_alpha_);
// }

void ReactiveController::publishCostmap()
{
	// // ROS_ERROR("costmap_boundary_pub_.publish");
	costmap_boundary_pub_.publish(costmap_boundary_);
	// // ROS_ERROR("costmap_boundary_pub_.publish successfully!");
	// // ROS_ERROR("obstacle_point_cloud_pub_.publish");
	obstacle_point_cloud_pub_.publish(configuration_layer_.getObstaclePointCloud());
	// // ROS_ERROR("obstacle_point_cloud_pub_.publish successfully!");
	// // ROS_ERROR("reachable_configurations_pub_.publish");
	reachable_configurations_pub_.publish(reachable_configurations_);
	// // ROS_ERROR("reachable_configurations_pub_.publish successfully!");
}

/*
程序流程如下：

机器人通过传感器获取人的位置信息，并将其作为目标点更新到机器人的运动控制模块中。

当机器人接收到里程计信息时，会调用odomCallback函数，获取当前机器人的线速度和角速度，并将其存储到current_vel_中。

程序将获取到的人的位置信息与机器人当前所在位置进行比较，使用比例控制器计算出机器人应该达到的线速度vl和角速度va。

如果计算出来的速度超过了机器人的最大速度max_v_和最大角速度max_w_，则将其限制在最大值之内。

判断当前距离是否小于距离阈值dist_threshold_，或者当前角度是否大于0.5弧度。如果是，则停止机器人运动；否则，继续按照计算出的速度指令运动。

在运动控制的同时，程序还会将当前的costmap信息进行可视化，包括机器人感知范围内的障碍物信息、机器人到达不了的区域信息以及机器人可以到达的区域信息。

最后，程序将计算出的最新速度指令latest_vel_存储到vel_mutex_中，供机器人运动控制模块使用。
*/