#define _USE_MATH_DEFINES
#include <auto_follow/collision_avoidance/configuration_layer.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

ConfigurationLayer::ConfigurationLayer() :
	costmap_(nullptr),
	latest_costmap_(nullptr),
	seg1_(nullptr),
	seg2_(nullptr),
	seg3_(nullptr),
	seg4_(nullptr),
	seen_(nullptr),
	cached_costs_(nullptr),
	cached_distances_(nullptr),
	initialized_(false)
{

}

void ConfigurationLayer::initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>& footprint)
{
	if(!initialized_)
	{
		if(static_cast<int>(footprint.size()) != 4)
		{
			// ROS_ERROR("ERROR: footprint.size() should be 4!");
		}
		else
		{
			footprint_ = footprint;
			seg_step_ = 0.05;
			seg1_ = new Segment(footprint[0].x, footprint[0].y, footprint[1].x, footprint[1].y, seg_step_);
			seg2_ = new Segment(footprint[1].x, footprint[1].y, footprint[2].x, footprint[2].y, seg_step_);
			seg3_ = new Segment(footprint[2].x, footprint[2].y, footprint[3].x, footprint[3].y, seg_step_);
			seg4_ = new Segment(footprint[3].x, footprint[3].y, footprint[0].x, footprint[0].y, seg_step_);
		}
        nh.param("configuration_layer/resolution_l", params_.resolution_l, params_.resolution_l);
        nh.param("configuration_layer/origin_l", params_.origin_l, params_.origin_l);
        nh.param("configuration_layer/max_l", params_.max_l, params_.max_l);
        nh.param("configuration_layer/resolution_alpha", params_.resolution_alpha, params_.resolution_alpha);
        nh.param("configuration_layer/origin_alpha", params_.origin_alpha, params_.origin_alpha);
        nh.param("configuration_layer/max_alpha", params_.max_alpha, params_.max_alpha);
        nh.param("configuration_layer/cell_inflation_radius", params_.cell_inflation_radius, params_.cell_inflation_radius);
        nh.param("configuration_layer/weight_l", params_.weight_l, params_.weight_l);
        nh.param("configuration_layer/weight_alpha", params_.weight_alpha, params_.weight_alpha);
		size_l_ = (int)((params_.max_l-params_.origin_l)/params_.resolution_l);
		size_alpha_ = (int)((params_.max_alpha-params_.origin_alpha)/params_.resolution_alpha);
		seen_size_ = size_l_*size_alpha_;
		seen_ = NULL;
		computeCaches();
		initCostmap();
		obstacle_point_cloud_.header.frame_id = "base_link";
		initialized_ = true;
	}
	else
	{
        ROS_WARN("configuration_layer has already been initialized.");
	}
}

ConfigurationLayer::~ConfigurationLayer()
{
	if(costmap_)
		delete[] costmap_;
	if(seg1_)
		delete seg1_;
	if(seg2_)
		delete seg2_;
	if(seg3_)
		delete seg3_;
	if(seg4_)
		delete seg4_;
	if(seen_)
		delete[] seen_;
	if(cached_costs_)
		delete[] cached_costs_;
	if(cached_distances_)
		delete[] cached_distances_;
}

void ConfigurationLayer::computeCaches()
{
	cached_costs_ = new unsigned char*[params_.cell_inflation_radius + 2];
	cached_distances_ = new float*[params_.cell_inflation_radius + 2];

	for (int i = 0; i <= params_.cell_inflation_radius + 1; ++i)
	{
		cached_costs_[i] = new unsigned char[params_.cell_inflation_radius + 2];
		cached_distances_[i] = new float[params_.cell_inflation_radius + 2];
		for (int j = 0; j <= params_.cell_inflation_radius + 1; ++j)
		{
			cached_distances_[i][j] = hypot(i, j);
		}
	}

	for (int i = 0; i <= params_.cell_inflation_radius + 1; ++i)
	{
		for (int j = 0; j <= params_.cell_inflation_radius + 1; ++j)
		{
			cached_costs_[i][j] = computeCost(i, j);
		}
	}
}

void ConfigurationLayer::inflateCostmap()
{
	int obstacle_list_size = static_cast<int>(obstacle_l_list_.size());
	if(NULL == seen_)
	{
    	seen_ = new bool[seen_size_];
	}
	memset(seen_, false, seen_size_ * sizeof(bool));

	std::vector<CellData>& obs_bin = inflation_cells_[0.0];
	for(int i = 0; i < obstacle_list_size; i++)
	{
		int index = getIndex(obstacle_l_list_[i], obstacle_alpha_list_[i]);
		obs_bin.push_back(CellData(index, obstacle_l_list_[i], obstacle_alpha_list_[i], obstacle_l_list_[i], obstacle_alpha_list_[i]));
	}

	// Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
	// can overtake previously inserted but farther away cells
	std::map<float, std::vector<CellData> >::iterator bin;
	for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
	{
		for (int i = 0; i < bin->second.size(); ++i)
		{
			// process all cells at distance dist_bin.first
			const CellData& cell = bin->second[i];

			int index = cell.index_;

			// ignore if already visited
			if (seen_[index])
			{
				continue;
			}

			seen_[index] = true;

			int m_l = cell.m_l_;
			int m_alpha = cell.m_alpha_;
			int s_l = cell.s_l_;
			int s_alpha = cell.s_alpha_;

			// assign the cost associated with the distance from an obstacle to the cell
			unsigned char cost = costLookup(m_l, m_alpha, s_l, s_alpha);
			unsigned char old_cost = costmap_[index];
			costmap_[index] = std::max(old_cost, cost);

			// attempt to put the neighbors of the current cell onto the inflation list
			if (m_l > 0)
				enqueue(index - size_alpha_, m_l - 1, m_alpha, s_l, s_alpha);
			if (m_alpha > 0)
				enqueue(index - 1, m_l, m_alpha - 1, s_l, s_alpha);
			if (m_l < size_l_ - 1)
				enqueue(index + size_alpha_, m_l + 1, m_alpha, s_l, s_alpha);
			if (m_alpha < size_alpha_ - 1)
				enqueue(index + 1, m_l, m_alpha + 1, s_l, s_alpha);
		}
	}
	inflation_cells_.clear();

	int costmap_size = size_l_*size_alpha_;
	int obstacle_point_cnt = 0;
	for(int i = 0; i < costmap_size; i++)
	{
		if(costmap_[i] == 0)
			continue;
		else
		{
			obstacle_point_cnt++;
		}
	}

	{
		std::unique_lock<std::mutex> lck (obstacle_mutex_);
		obstacle_point_cloud_.header.seq = seq_++;
		obstacle_point_cloud_.points.clear();
		std::vector<geometry_msgs::Point32>().swap(obstacle_point_cloud_.points);
		obstacle_point_cloud_.points.resize(obstacle_point_cnt);
		geometry_msgs::Point32 obstacle_point;

		for(int i = 0; i < costmap_size; i++)
		{
			if(costmap_[i] == 0)
				continue;
			else
			{
				int m_l, m_alpha;
				indexToCells(i, m_l, m_alpha);
				float l, alpha;
				mapToWorld(m_l, m_alpha, l, alpha);
				obstacle_point.x = l*cos(alpha);
				obstacle_point.y = l*sin(alpha);
				obstacle_point.z = costmap_[i];
				obstacle_point_cloud_.points.push_back(obstacle_point);
			}
		}
	}
}

CollisionSituation ConfigurationLayer::updateCostmap(std::vector<geometry_msgs::Point32>& obstacle_list)
{
	std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point time_current;
	std::chrono::duration<double> time_used;
	ros::Time t0 = ros::Time::now();

	CollisionSituation collision_situation = CollisionSituation::NO_COLLISION;
	float xi, yi, xf, yf, xs, ys, l, alpha;
	float a;
	int m_l, m_alpha;
	int count = 0;
	obstacle_l_list_.clear();
	obstacle_alpha_list_.clear();
	std::vector<int>().swap(obstacle_l_list_);
	std::vector<int>().swap(obstacle_alpha_list_);
	int obs_length = static_cast<int>(obstacle_list.size());
	obstacle_l_list_.resize(1./seg_step_*4*obs_length);
	obstacle_alpha_list_.resize(1./seg_step_*4*obs_length);
	resetCostmap();

	geometry_msgs::Point32 obstacle_point;
	for(int i = 0; i < obs_length; i++)
	{
		xf = obstacle_list[i].x;
		yf = obstacle_list[i].y;
		if(checkPointInFootprint(xf, yf))
		{
			if(xf < 0 && CollisionSituation::BACKWARD_COLLISION != collision_situation)
			{
				collision_situation = CollisionSituation::BACKWARD_COLLISION;
				for(int m_alpha = 0; m_alpha < size_alpha_; m_alpha++)
				{
					float alpha = params_.origin_alpha+m_alpha*params_.resolution_alpha;
					if(alpha > M_PI/2 || alpha < -M_PI/2)
						costmap_[getIndex(1, m_alpha)] = 255;					
				}
			}
			else if(xf >= 0 && CollisionSituation::FORWARD_COLLISION != collision_situation)
			{
				collision_situation = CollisionSituation::FORWARD_COLLISION;
				for(int m_alpha = 0; m_alpha < size_alpha_; m_alpha++)
				{
					float alpha = params_.origin_alpha+m_alpha*params_.resolution_alpha;
					if(alpha > -M_PI/2 && alpha < M_PI/2)
						costmap_[getIndex(1, m_alpha)] = 255;					
				}
			}
			continue;
		}
		for(int j = 0; j < seg1_->size_; j++)
		{
			seg1_->getXY(j, xi, yi);
			a = ((yf*yf-yi*yi)+(xf*xf-xi*xi))*(pow((yf-yi),2)+pow((xf-xi),2))/
				(pow(yf-yi,4)+2*(xf*xf+xi*xi)*pow(yf-yi,2)+pow(xf*xf-xi*xi,2));
			xs = a*(xf+xi);
			ys = a*(yf-yi);
			transformToLalpha(xs, ys, l, alpha);
			if(worldToMap(l, alpha, m_l, m_alpha))
			{
				costmap_[getIndex(m_l, m_alpha)] = 255;
				obstacle_l_list_.push_back(m_l);
				obstacle_alpha_list_.push_back(m_alpha);
				obstacle_point.x = l*cos(alpha);
				obstacle_point.y = l*sin(alpha);
				obstacle_point.z = 255;
			}
		}
		for(int j = 0; j < seg2_->size_; j++)
		{
			if(xf > 0)
				break;
			seg2_->getXY(j, xi, yi);
			a = ((yf*yf-yi*yi)+(xf*xf-xi*xi))*(pow((yf-yi),2)+pow((xf-xi),2))/
				(pow(yf-yi,4)+2*(xf*xf+xi*xi)*pow(yf-yi,2)+pow(xf*xf-xi*xi,2));
			xs = a*(xf+xi);
			ys = a*(yf-yi);
			transformToLalpha(xs, ys, l, alpha);
			if(worldToMap(l, alpha, m_l, m_alpha))
			{
				costmap_[getIndex(m_l, m_alpha)] = 255;
				obstacle_l_list_.push_back(m_l);
				obstacle_alpha_list_.push_back(m_alpha);
				obstacle_point.x = l*cos(alpha);
				obstacle_point.y = l*sin(alpha);
				obstacle_point.z = 255;
			}
		}
		for(int j = 0; j < seg3_->size_; j++)
		{
			seg3_->getXY(j, xi, yi);
			a = ((yf*yf-yi*yi)+(xf*xf-xi*xi))*(pow((yf-yi),2)+pow((xf-xi),2))/
				(pow(yf-yi,4)+2*(xf*xf+xi*xi)*pow(yf-yi,2)+pow(xf*xf-xi*xi,2));
			xs = a*(xf+xi);
			ys = a*(yf-yi);
			transformToLalpha(xs, ys, l, alpha);
			if(worldToMap(l, alpha, m_l, m_alpha))
			{
				costmap_[getIndex(m_l, m_alpha)] = 255;
				obstacle_l_list_.push_back(m_l);
				obstacle_alpha_list_.push_back(m_alpha);
				obstacle_point.x = l*cos(alpha);
				obstacle_point.y = l*sin(alpha);
				obstacle_point.z = 255;
			}
		}
		for(int j = 0; j < seg4_->size_; j++)
		{
			if(xf < 0)
				break;
			seg4_->getXY(j, xi, yi);
			a = ((yf*yf-yi*yi)+(xf*xf-xi*xi))*(pow((yf-yi),2)+pow((xf-xi),2))/
				(pow(yf-yi,4)+2*(xf*xf+xi*xi)*pow(yf-yi,2)+pow(xf*xf-xi*xi,2));
			xs = a*(xf+xi);
			ys = a*(yf-yi);
			transformToLalpha(xs, ys, l, alpha);
			if(worldToMap(l, alpha, m_l, m_alpha))
			{
				costmap_[getIndex(m_l, m_alpha)] = 255;
				obstacle_l_list_.push_back(m_l);
				obstacle_alpha_list_.push_back(m_alpha);
				obstacle_point.x = l*cos(alpha);
				obstacle_point.y = l*sin(alpha);
				obstacle_point.z = 255;
			}
		}
	}
	// ROS_ERROR("inflateCostmap() begin");
	inflateCostmap();
	// ROS_ERROR("inflateCostmap() end");
	// ROS_ERROR("updateLatestCostmap() begin");
	updateLatestCostmap();
	// ROS_ERROR("updateLatestCostmap() end");

	// ros::Time t1 = ros::Time::now();
	// ros::Duration duration(t1-t0);
	// // ROS_ERROR("updateCostmap process time in ms:");
	// std::cout << duration*1000 << std::endl;
	// time_current = std::chrono::steady_clock::now();
	// time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time_current-time_start);
	// std::cout << "process time: " << time_used.count()*1000 << "ms" << std::endl;
	
	return collision_situation;
}

bool ConfigurationLayer::checkFeasibility(float l, float alpha)
{
	int m_l, m_alpha;
	if(worldToMap(l, alpha, m_l, m_alpha))
	{
		std::unique_lock<std::mutex> lck (latest_costmap_mutex_);
		for(int i = 0; i <= m_l; i++)
		{
			if(255 == latest_costmap_[getIndex(i, m_alpha)])
			{
				// mapToWorld(i, m_alpha, collision_l, collision_alpha);
				// std::cout << "collision (l, alpha): (" << collision_l << ", " << collision_alpha << ")" << std::endl;
				return false;
			}
		}
	}
	else
	{
		std::unique_lock<std::mutex> lck (latest_costmap_mutex_);
		for(int i = 0; i < size_l_; i++)
		{
			if(255 == latest_costmap_[getIndex(i, m_alpha)])
			{
				// mapToWorld(i, m_alpha, collision_l, collision_alpha);
				// std::cout << "collision (l, alpha): (" << collision_l << ", " << collision_alpha << ")" << std::endl;
				return false;
			}			
		}
	}
	return true;
}

bool ConfigurationLayer::checkFeasibility(float l, float alpha, float &collision_l, float &collision_alpha)
{
	int m_l, m_alpha;
	if(worldToMap(l, alpha, m_l, m_alpha))
	{
		std::unique_lock<std::mutex> lck (latest_costmap_mutex_);
		for(int i = 0; i <= m_l; i++)
		{
			if(255 == latest_costmap_[getIndex(i, m_alpha)])
			{
				mapToWorld(i, m_alpha, collision_l, collision_alpha);
				// std::cout << "collision (l, alpha): (" << collision_l << ", " << collision_alpha << ")" << std::endl;
				return false;
			}
		}
	}
	else
	{
		std::unique_lock<std::mutex> lck (latest_costmap_mutex_);
		for(int i = 0; i < size_l_; i++)
		{
			if(255 == latest_costmap_[getIndex(i, m_alpha)])
			{
				mapToWorld(i, m_alpha, collision_l, collision_alpha);
				// std::cout << "collision (l, alpha): (" << collision_l << ", " << collision_alpha << ")" << std::endl;
				return false;
			}			
		}
	}
	return true;
}
