#ifndef CONFIGURATION_LAYER_H
#define CONFIGURATION_LAYER_H
#include <ros/ros.h>
#include <auto_follow/collision_avoidance/configuration_utils.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <cmath>
#include <map>
#include <mutex>

class CellData
{
public:
	CellData(int i, int l, int alpha, int sl, int salpha) :
		index_(i), m_l_(l), m_alpha_(alpha), s_l_(sl), s_alpha_(salpha)
	{
	}
	~CellData(){}

	int index_;
	int m_l_, m_alpha_;
	int s_l_, s_alpha_;
};

class ConfigurationLayer
{
	class Segment
	{
	public:
		Segment(float x_1, float y_1, float x_2, float y_2, float step)
		{
			x_1_ = std::min(x_1, x_2);
			x_2_ = std::max(x_1, x_2);
			y_1_ = std::min(y_1, y_2);
			y_2_ = std::max(y_1, y_2);
			step_ = step;
			float dx = x_2_-x_1_;
			float dy = y_2_-y_1_;
			length_ = sqrt(dx*dx+dy*dy);
			size_ = static_cast<int>((length_+step_/2)/step_)+1;
		}
		~Segment(){}
		void getXY(int index, float &x, float &y)
		{
			x = x_1_+(x_2_-x_1_)*index*step_/length_;
			y = y_1_+(y_2_-y_1_)*index*step_/length_;
		}
		float x_1_;
		float y_1_;
		float x_2_;
		float y_2_;
		float step_;
		float length_;
		float size_;
	};
public:
	ConfigurationLayer();
	~ConfigurationLayer();

	void initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>& footprint);

	inline void transformToLalpha(const float &x, const float &y, float &l, float &alpha)
	{
		if(y == 0)
		{
			l = abs(x);
			alpha = 0;
		}
		else
		{
			l = abs((x*x+y*y)/(2*y)*atan2(2*x*y, x*x-y*y));
			if(x >= 0)
			{
				alpha = atan2(2*y, x*x+y*y);
			}
			else
			{
				alpha = sgn(y)*3.14159265-atan2(2*y, x*x+y*y);
			}
		}
	}


	/**
	* @brief  Convert from map coordinates to world coordinates
	* @param  m_l The l map coordinate
	* @param  m_alpha The alpha map coordinate
	* @param  w_l Will be set to the associated world l coordinate
	* @param  w_alpha Will be set to the associated world alpha coordinate
	*/
	inline void mapToWorld(int m_l, int m_alpha, float& w_l, float& w_alpha)
	{
		w_l = params_.origin_l + (m_l + 0.5) * params_.resolution_l;
		w_alpha = params_.origin_alpha + (m_alpha + 0.5) * params_.resolution_alpha;
	}


	/**
	* @brief  Convert from world coordinates to map coordinates
	* @param  w_l The l world coordinate
	* @param  w_alpha The alpha world coordinate
	* @param  m_l Will be set to the associated map l coordinate
	* @param  m_alpha Will be set to the associated map alpha coordinate
	* @return True if the conversion was successful (legal bounds) false otherwise
	*/
	inline bool worldToMap(float w_l, float w_alpha, int& m_l, int& m_alpha)
	{
		if (w_l < params_.origin_l || w_alpha < params_.origin_alpha)
			return false;

		m_l = (int)((w_l - params_.origin_l) / params_.resolution_l);
		m_alpha = (int)((w_alpha - params_.origin_alpha) / params_.resolution_alpha);

		if (m_l < size_l_ && m_alpha < size_alpha_)
			return true;

		return false;
	}

	/**
	* @brief  Given two map coordinates... compute the associated index
	* @param m_l The l coordinate
	* @param m_alpha The alpha coordinate
	* @return The associated index
	*/
	inline int getIndex(int m_l, int m_alpha)
	{
		return m_l * size_alpha_ + m_alpha;
	}

	/**
	* @brief  Given an index... compute the associated map coordinates
	* @param  index The index
	* @param  m_l Will be set to the l coordinate
	* @param  m_alpha Will be set to the alpha coordinate
	*/
	inline void indexToCells(int index, int& m_l, int& m_alpha)
	{
		m_l = index / size_alpha_;
		m_alpha = index - (m_l * size_alpha_);
	}
	void initCostmap()
	{
		costmap_size_ = size_l_*size_alpha_;
		if(costmap_)
			delete[] costmap_;
		if(latest_costmap_)
			delete[] latest_costmap_;
		costmap_ = new unsigned char[costmap_size_];
		latest_costmap_ = new unsigned char[costmap_size_];
		memset(costmap_, 0, costmap_size_*sizeof(unsigned char));
		memset(latest_costmap_, 0, costmap_size_*sizeof(unsigned char));
	}
	void resetCostmap()
	{
		memset(costmap_, 0, size_l_*size_alpha_*sizeof(unsigned char));
		// for(int i = 0; i < size_l_*size_alpha_; i++)
		// 	costmap_[i] = 0;
	}

	CollisionSituation updateCostmap(std::vector<geometry_msgs::Point32>& obstacle_list);
	void updateLatestCostmap()
	{
		std::unique_lock<std::mutex> lck (latest_costmap_mutex_);
		for(int i = 0; i < costmap_size_; i++)
			latest_costmap_[i] = costmap_[i];
	}

	inline sensor_msgs::PointCloud getObstaclePointCloud()
	{
		std::unique_lock<std::mutex> lck (obstacle_mutex_);
		return obstacle_point_cloud_;
	}

	inline unsigned char getCost(float l, float alpha)
	{
		int m_l, m_alpha;
		if(worldToMap(l, alpha, m_l, m_alpha))
		{
			return costmap_[getIndex(m_l, m_alpha)];
		}
		else
		{
			return 0;
		}
	}

	void inflateCostmap();

	void computeCaches();

	inline unsigned char computeCost(int d_l, int d_alpha)
	{
		unsigned char cost = 0;
		if (0 == d_l && d_alpha < 3)
			cost = 255;
		else if(0 == d_alpha && d_l < 3)
			cost = 255;
		else
		{
			float l = d_l*params_.resolution_l;
			float alpha = d_alpha*params_.resolution_alpha;
			float factor = exp(-1.0*params_.weight_l*l)*exp(-1.0*params_.weight_alpha*alpha);
			cost = (unsigned char)(254 * factor);
		}
		return cost;
	}

	inline float distanceLookup(int m_l, int m_alpha, int s_l, int s_alpha)
	{
		int d_l = abs(m_l - s_l);
		int d_alpha = abs(m_alpha - s_alpha);
		return cached_distances_[d_l][d_alpha];
	}

	inline unsigned char costLookup(int m_l, int m_alpha, int s_l, int s_alpha)
	{
		int d_l = abs(m_l - s_l);
		int d_alpha = abs(m_alpha - s_alpha);
		return cached_costs_[d_l][d_alpha];
	}

	inline void enqueue(int index, int m_l, int m_alpha, int s_l, int s_alpha)
	{
		if (!seen_[index])
		{
			// we compute our distance table one cell further than the inflation radius dictates so we can make the check below
			double distance = distanceLookup(m_l, m_alpha, s_l, s_alpha);

			// we only want to put the cell in the list if it is within the inflation radius of the obstacle point
			if (distance > params_.cell_inflation_radius)
				return;

			// push the cell data onto the inflation list and mark
			inflation_cells_[distance].push_back(CellData(index, m_l, m_alpha, s_l, s_alpha));
		}
	}

	inline float getCrossProduct(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p)
	{
		return ((p2.x-p1.x)*(p.y-p1.y)-(p.x-p1.x)*(p2.y-p1.y));
	}

	inline bool checkPointInFootprint(float x, float y)
	{
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		bool is_point_in_footprint = 
			(getCrossProduct(footprint_[0],footprint_[1],p)*getCrossProduct(footprint_[2],footprint_[3],p) >= 0)
		 && (getCrossProduct(footprint_[1],footprint_[2],p)*getCrossProduct(footprint_[3],footprint_[0],p) >= 0);
		return is_point_in_footprint;
	}

	bool checkFeasibility(float l, float alpha);
	bool checkFeasibility(float l, float alpha, float &collision_l, float &collision_alpha);

private:
	ros::NodeHandle nh_;
	bool initialized_ = false;
	unsigned char* costmap_ = NULL;
	unsigned char* latest_costmap_ = NULL;
	int costmap_size_;
	std::vector<geometry_msgs::Point> footprint_;
	Segment* seg1_;
	Segment* seg2_;
	Segment* seg3_;
	Segment* seg4_;
	float seg_step_;
	sensor_msgs::PointCloud obstacle_point_cloud_;
	std::mutex obstacle_mutex_;
	std::mutex latest_costmap_mutex_;
	std::vector<int> obstacle_l_list_;
	std::vector<int> obstacle_alpha_list_;
	int seq_ = 0;
	int size_l_;
	int size_alpha_;
	bool* seen_;
	int seen_size_;
	unsigned char** cached_costs_;
	float** cached_distances_;
	std::map<float, std::vector<CellData> > inflation_cells_;
    struct Parameters
    {
		float resolution_l = 0.02;
		float resolution_alpha = 3.14159265/180;
		float origin_l = 0;
		float origin_alpha = -3.14159265;
		float max_l = 4;
		float max_alpha = 3.14159265;
		int cell_inflation_radius = 5;
		float weight_l = 10;
		float weight_alpha = 30;
    } params_;


};


#endif //CONFIGURATION_LAYER_H
