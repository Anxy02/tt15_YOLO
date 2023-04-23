#include <auto_follow/people_detector.h>
#include <chrono>

#define PI 3.1415926
#define FILTER_SIZE 2
#define FLANK_THRESHOLD 0.1

#define FLANK_U 1
#define FLANK_D -1

//Antropometric parameters
//#define ANTRO_a0 0.1 //|
//#define ANTRO_a1 0.2 //|-> Leg width (min-max)
#define ANTRO_b0 0	 //	|
#define ANTRO_b1 0.4 //	|-> Free space between two legs (min-max)
#define ANTRO_c0 0.06 //		|
#define ANTRO_c1 0.7 //		|-> Two legs together width (min-max)
#define step_dist 0.5

// Pattern Type
#define TYPE_LA 1 // Legs separated
#define TYPE_FS 2 // Legs dephased
#define TYPE_SL 3 // Legs together

bool PeopleDetector::initialize(ros::NodeHandle& nh)
{
	if(!initialized_)
	{
		nh_ = nh;
		nh.param("laser_topic", laser_topic_, laser_topic_);
		nh.param("people_detector_node/ANTRO_a0", ANTRO_a0_, ANTRO_a0_);
		nh.param("people_detector_node/ANTRO_a0", ANTRO_a1_, ANTRO_a1_);
		nh.param("people_detector_node/init_x", init_x_, init_x_);
		nh.param("people_detector_node/init_y", init_y_, init_y_);
		nh.param("duration_t", duration_t_, duration_t_);
		node_pub_ = nh.advertise <geometry_msgs::PoseArray>("edge_leg_detector", 2);
		pos_pub_ = nh.advertise <geometry_msgs::PoseStamped>("people_pos", 2);
		node_sub_ = nh.subscribe(laser_topic_, 2, &PeopleDetector::scanCallback, this);
		initialized_ = true;
		ROS_INFO("PeopleDetector is initialized successfully!");
	}
	else
	{
		ROS_WARN("PeopleDetector has been initialized!");
	}
	return true;
}

// Mean value of the 'size' adjacent values
void PeopleDetector::laserFilterMean(std::vector <double> *vector_r, unsigned size)
{
	for(unsigned i = 0; i < ((*vector_r).size()-size); i++)
	{
		double mean = 0;
		for(unsigned k = 0; k < size; k++)
		{
			mean += (*vector_r)[i + k];
		}
		(*vector_r)[i] = mean/size;
	}
}

void PeopleDetector::scanCallback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
	// std::chrono::steady_clock::time_point time_current;
	// std::chrono::duration<double> time_used;
	// ros::Time t0 = ros::Time::now();
	
	sensor_frame_id_ = msg->header.frame_id;
	// Vectors...
	rec_x_.clear(); 
	rec_y_.clear(); 
	
	sensor_on_ = true;
	
	double px, py, pr, pt;
	std::vector <double>	laser_x;
	std::vector <double>	laser_y;
	std::vector <double>	laser_r;//rou
	std::vector <double>	laser_t;//theta
	for(unsigned i = 0; i < msg->ranges.size(); i++)
	{		
		if(isnan(msg->ranges[i]))
			continue;
		else if(isinf(msg->ranges[i]))
			continue;
		else if(msg->ranges[i] <= 0.001)
			continue;
		else if(msg->ranges[i] > max_range_)
			continue;
		pr = msg->ranges[i];
		pt = msg->angle_min + (i * msg->angle_increment);
		laser_r.push_back(pr);
		laser_t.push_back(pt);
	}
	// ROS_INFO_STREAM("angle_min:" << msg->angle_min << "    angle_max"<< msg->angle_max);
	// ROS_INFO_STREAM("range_max  " << msg->ranges.size() << "   ");
	// Filtering laser scan
	// laserFilterMean(&laser_r, FILTER_SIZE);
	for(unsigned i = 0; i < laser_r.size(); i++)
	{		
		px = laser_r[i] * cos(laser_t[i]);
		py = laser_r[i] * sin(laser_t[i]);
		laser_x.push_back(px);
		laser_y.push_back(py);
	}

	std::string str_aux = "";
	// Finding flanks in the laser scan...
	std::vector <int> laser_flank;
	laser_flank.assign(laser_r.size(), 0);
	for(unsigned i = 1; i < laser_flank.size(); i++)
	{
		if(fabs(laser_r[i] - laser_r[i - 1]) > FLANK_THRESHOLD)
		{
			laser_flank[i] = ((laser_r[i] - laser_r[i - 1]) > 0) ? FLANK_U : FLANK_D;
		}
	}
	
	std::vector <int> flank_id0;
	std::vector <int> flank_id1;
	std::string flank_string = "";
	int past_value = 0;
	int idx = 0;
	for(unsigned i = 1; i < laser_flank.size(); i++)
	{
		if(laser_flank[i] != 0)
		{
			if(past_value != laser_flank[i])
			{
				flank_id0.push_back(i - 1);
				flank_id1.push_back(i);
				flank_string += (laser_flank[i] > 0) ? "S" : "B";
				idx++;
			}
			else
				flank_id1[idx - 1] = i;		
		}
		past_value = laser_flank[i];
	}	
	//TODO: check
	// ROS_INFO_STREAM("flank_string: " << flank_string);
		
	// PATTERN RECOGNITION
	std::string LEGS_LA	= "BSBS";
	std::string LEGS_FS1 = "BBS";
	std::string LEGS_FS2 = "BSS";
	std::string LEGS_SL = "BS";
	
	std::list <int> Pattern_LA;
	std::list <int> Pattern_FS1;
	std::list <int> Pattern_FS2;
	std::list <int> Pattern_SL;

	// findPattern(flank_string, LEGS_LA,	&Pattern_LA	);
	findPattern(flank_string, LEGS_FS1, &Pattern_FS1);
	findPattern(flank_string, LEGS_FS2, &Pattern_FS2);
	findPattern(flank_string, LEGS_SL,	&Pattern_SL	);	

	// ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
	// validatePattern(&Pattern_LA,	TYPE_LA, flank_id0, flank_id1,	laser_x, laser_y);
	validatePattern(&Pattern_FS1, TYPE_FS, flank_id0, flank_id1,	laser_x, laser_y);
	validatePattern(&Pattern_FS2, TYPE_FS, flank_id0, flank_id1,	laser_x, laser_y);
	validatePattern(&Pattern_SL,	TYPE_SL, flank_id0, flank_id1,	laser_x, laser_y);

	// ERASE REDUNDANT PATTERNS FROM ACCEPTED ONES (If a LA or FS pattern is accepted, we erase the SL on it)
	// a) Erase SL from LA
	std::list<int>::iterator it_K;
	for(it_K = Pattern_LA.begin(); it_K != Pattern_LA.end(); it_K++)
	{
		std::list<int>::iterator it_M;
		// Erase first leg
		for(it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++)
			if(flank_id0[*it_K] == flank_id0[*it_M])
			{
				Pattern_SL.erase(it_M);
				break;
			}
		// Erase second leg
		for(it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++)
			if(flank_id0[*it_K + 2] == flank_id0[*it_M]){
			Pattern_SL.erase(it_M);
			break;
		}
	}
	// b) Erase SL from FS1 "BBS"
	for(it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); it_K++)
	{
		std::list<int>::iterator it_M;
		for(it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++)
			if(flank_id0[*it_K + 1] == flank_id0[*it_M])
			{
				Pattern_SL.erase(it_M);
				break;
			}
	}
	// c) Erase SL from FS2 "BSS"
	for(it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); it_K++)
	{
		std::list<int>::iterator it_M;
		for(it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++)
			if(flank_id0[*it_K] == flank_id0[*it_M]){
				Pattern_SL.erase(it_M);
				break;
			}
	}

	
	boost::mutex::scoped_lock lock(rec_mutex_);
	//CENTROID PATTERN COMPUTATION & UNCERTAINTY
	rec_x_.clear();
	rec_y_.clear();
	
	// humanPose(&rec_x_, &rec_y_, Pattern_LA,	TYPE_LA,	flank_id0, flank_id1,	laser_x, laser_y);

	humanPose(&rec_x_, &rec_y_, Pattern_FS1, TYPE_FS,	flank_id0, flank_id1,	laser_x, laser_y);
	humanPose(&rec_x_, &rec_y_, Pattern_FS2, TYPE_FS,	flank_id0, flank_id1,	laser_x, laser_y);
	humanPose(&rec_x_, &rec_y_, Pattern_SL,	TYPE_SL,	flank_id0, flank_id1,	laser_x, laser_y);

	// ros::Time t1 = ros::Time::now();
	// ros::Duration duration(t1-t0);
	// ROS_ERROR("PeopleDetector::scanCallBack process time in ms:");
	// std::cout << duration*1000 << std::endl;
	// time_current = std::chrono::steady_clock::now();
	// time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time_current-time_start);
	// std::cout << "process time: " << time_used.count()*1000 << "ms" << std::endl;

	// //TODO: 添加查看rec_x_识别到了多少
	// for(unsigned j = 0; j < rec_x_.size(); j++){
	// 	ROS_INFO_STREAM("3: rec_x_:"<< rec_x_[j] << "    rec_y_:" << rec_y_[j]);
	// 	ROS_INFO_STREAM("3: rec_x.size: "<< rec_x_.size());
	// }

}

// Reports a found string pattern in a list
void PeopleDetector::findPattern(std::string str, std::string pattern, std::list <int> *element_found)
{
	size_t found = 0;

	while(std::string::npos != (found = str.find(pattern, found)))
	{
		(*element_found).push_back(found); 
		found++;
	}
}

// Performs the antropometric validation of the leg patterns
//实现了对从激光雷达扫描数据中提取的人体位置信息进行校验的功能,排除可能由于噪声等因素导致的误检，并保留真正的人体检测结果
void PeopleDetector::validatePattern(std::list <int> *Pattern_list, int TYPE,	std::vector <int> flank_id0,	std::vector <int> flank_id1, std::vector <double> laser_x, std::vector <double> laser_y)
{
	double ANTRO_a_1, ANTRO_a_2, ANTRO_b, ANTRO_c; // Antropometric values from patterns to compare with constants.
	bool SavePattern = true;
	bool cond_a = true, cond_b = true, cond_c = true;
	double midx,midy;
	int v_count = 0;
	std::list<int>::iterator it;
	
	for(it = (*Pattern_list).begin(); it != (*Pattern_list).end(); it++){
		v_count++;

		// Obtain antropometric values
		switch(TYPE){
			case TYPE_LA: //BSBS
            	ANTRO_a_1 = dist2D(laser_x[flank_id1[*it]], laser_y[flank_id1[*it]], laser_x[flank_id0[*it + 1]], laser_y[flank_id0[*it + 1]]);
            	ANTRO_a_2 = dist2D(laser_x[flank_id1[*it + 2]], laser_y[flank_id1[*it + 2]], laser_x[flank_id0[*it + 3]], laser_y[flank_id0[*it + 3]]);
            	ANTRO_b = dist2D(laser_x[flank_id0[*it + 1]], laser_y[flank_id0[*it + 1]], laser_x[flank_id1[*it + 2]], laser_y[flank_id1[*it + 2]]);
            	ANTRO_c = 0;
            	cond_a = ((ANTRO_a_1 >= ANTRO_a0_) && (ANTRO_a_1 <= ANTRO_a1_)) && ((ANTRO_a_2 >= ANTRO_a0_) && (ANTRO_a_2 <= ANTRO_a1_));
            	cond_b = ((ANTRO_b >= ANTRO_b0) && (ANTRO_b <= ANTRO_b1));
            	cond_c = true;
				break;
			case TYPE_FS: // BBS & BSS
            	ANTRO_a_1 = dist2D(laser_x[flank_id1[*it]], laser_y[flank_id1[*it]], laser_x[flank_id0[*it + 1]], laser_y[flank_id0[*it + 1]]);
            	ANTRO_a_2 = dist2D(laser_x[flank_id1[*it + 1]], laser_y[flank_id1[*it + 1]], laser_x[flank_id0[*it + 2]], laser_y[flank_id0[*it + 2]]);
            	ANTRO_b = dist2D(laser_x[flank_id0[*it + 1]], laser_y[flank_id0[*it + 1]], laser_x[flank_id1[*it + 1]], laser_y[flank_id1[*it + 1]]);
            	ANTRO_c = 0;
            	cond_a = ((ANTRO_a_1 >= ANTRO_a0_) && (ANTRO_a_1 <= ANTRO_a1_)) && ((ANTRO_a_2 >= ANTRO_a0_) && (ANTRO_a_2 <= ANTRO_a1_));
            	cond_b = ((ANTRO_b >= ANTRO_b0) && (ANTRO_b <= ANTRO_b1));
            	cond_c = true;
				break;
            case TYPE_SL: // BS
            	// ANTRO_a_1 = 0;
            	// ANTRO_a_1 = dist2D(laser_x[flank_id1[*it]], laser_y[flank_id1[*it]], laser_x[flank_id0[*it + 1]], laser_y[flank_id0[*it + 1]]);
            	// ANTRO_a_2 = 0;
            	// ANTRO_b = 0;
            	// ANTRO_c = dist2D(laser_x[flank_id1[*it]], laser_y[flank_id1[*it]], laser_x[flank_id0[*it + 1]], laser_y[flank_id0[*it + 1]]);
            	// cond_a = (ANTRO_a_1 >= ANTRO_a0_) && (ANTRO_a_1 <= ANTRO_a1_);
            	// cond_b = true;
            	// cond_c = true;
            	// // if (cond_a){
            	// 	//printf("width is %f,pos is [%f,%f]\n",ANTRO_a_1,laser_x[flank_id1[*it]], laser_y[flank_id1[*it]]);
            	// // }
				ANTRO_a_1 = 0;
		      	ANTRO_a_2 = 0;
		      	ANTRO_b = 0;
		      	ANTRO_c = dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
		      	cond_a = true;
		      	cond_b = true;	
		      	cond_c = ( ( ANTRO_c >= ANTRO_c0 ) && ( ANTRO_c <= ANTRO_c1 ) );

            	break;
		}

		SavePattern = cond_a && cond_b && cond_c;
		
		if(!SavePattern){
			it = (*Pattern_list).erase(it);
			it--;
		}
		else
		{
			//printf("width = %f \n" ,ANTRO_c);
		}
	}	
}

//该函数用于从激光雷达扫描数据中提取人体位置信息，并将其保存在向量r_x和r_y中。
void PeopleDetector::humanPose(std::vector <double> *r_x, std::vector <double> *r_y, std::list <int> Pattern_list, int TYPE,	std::vector <int> flank_id0,	std::vector <int> flank_id1, std::vector <double> laser_x, std::vector <double> laser_y)
{
	double c_x, c_y;
	int l1, l2, l3, l4;
	int count; 
	std::list<int>::iterator it;

	for(it = Pattern_list.begin(); it != Pattern_list.end(); it++){
		c_x = 0;
		c_y = 0;
		count = 0;

		l1 = flank_id1[*it];
		l2 = flank_id0[*it + 1];
		
		switch(TYPE){
		case TYPE_LA:
			l3 = flank_id1[*it + 2];
			l4 = flank_id0[*it + 3];
			break;
		case TYPE_FS:
			l3 = flank_id1[*it + 1];
			l4 = flank_id0[*it + 2];
			break;
		case TYPE_SL:
			l3 = 1;
			l4 = 0;
			break;
		}

		for(int i = l1; i <= l2; i++){
			c_x += laser_x[i];
			c_y += laser_y[i];
			count++;
		}
		for(int i = l3; i <= l4; i++){
			c_x += laser_x[i];
			c_y += laser_y[i];
			count++;
		}
		
		c_x /= (double) count;
		c_y /= (double) count;
		
		(*r_x).push_back(c_x);
		(*r_y).push_back(c_y);
	}
}

// Validate distance between persons
//该函数用于处理人体检测时可能产生的误检问题，避免出现多余的人体检测结果。
void PeopleDetector::validateDistance()
{
	boost::mutex::scoped_lock lock(rec_mutex_);
	int j = 0;
	while(j < (rec_x_.size() - 1))
	{
		// if the Euclidean distance between two persons are smaller than
		// the maximum width of a leg then the second person must be eliminated
		if (ANTRO_b1 > dist2D(rec_x_[j], rec_y_[j], rec_x_[j+1], rec_y_[j+1]))
		{
			rec_x_.erase(rec_x_.begin() + (j + 1));
			rec_y_.erase(rec_y_.begin() + (j + 1));
		}
		else
		{
			j++;
		}
	}
}

//该函数用于确定人体位置的初始状态，在检测到人体后立即调用。
void PeopleDetector::humanPoseInit()
{
	double posx,posy;
	if (rec_x_.size()==2){
		posx=(rec_x_[0]+rec_x_[1])/2;
		posy=(rec_y_[0]+rec_y_[1])/2;
		// posx = rec_x_[0];
		// posy = rec_y_[0];
		// ROS_INFO("rrr");
		if (posx < init_x_+0.3 && posx > init_x_-0.3 && posy < init_y_+0.3 && posy > init_y_-0.3)
		{
			now_pos_[0]=posx;
			now_pos_[1]=posy;
			init_done_ = true;
			printf("init done");
			ROS_ERROR("current x: %.4f, current y: %.4f", posx, posy);
		}
	}
}
//该函数主要实现了一个人体姿态滤波器，用于更新人体位置信息
void PeopleDetector::poseFilter()
{
	std::vector<double>::iterator itx;
	std::vector<double>::iterator ity;
	ity = rec_y_.begin();
	for(itx = rec_x_.begin(); itx != rec_x_.end(); itx++){
		//if(true){
		if (dist2D(*itx,*ity,now_pos_[0],now_pos_[1]) >= step_dist){
			//rec_x_.erase(rec_x_.begin()+it);
			//rec_y_.erase(rec_y_.begin()+it);
			// printf("detect_pos: %f, %f \n", *itx, *ity);
			// printf("now_pos: %f, %f \n",now_pos_[0],now_pos_[1]);
			// printf("erase a data,distance is %f \n",dist2D(*itx,*ity,now_pos_[0],now_pos_[1]));	
			itx=rec_x_.erase(itx);
			ity=rec_y_.erase(ity);

			itx--;
			ity--;
		}
		ity++;
	}

	if (init_done_ == true){
		//printf("x = %f ,y = %f \n",now_pos_[0],now_pos_[1]);
		if (rec_x_.size() == 1)
		{
			now_pos_[0]=rec_x_[0];
			now_pos_[1]=rec_y_[0];
			pub_pos_[0]=now_pos_[0];
			pub_pos_[1]=now_pos_[1];
			lost_pos_ = false;
		}
		else if (rec_x_.size() == 2)
		{
			now_pos_[0]=(rec_x_[0]+rec_x_[1])/2;
			now_pos_[1]=(rec_y_[0]+rec_y_[1])/2;
			pub_pos_[0]=now_pos_[0];
			pub_pos_[1]=now_pos_[1];
			lost_pos_ = false;
		}
		else 
		{
			// pub_pos_[0]=pub_pos_[0]*0.95;
			// pub_pos_[1]=pub_pos_[1]*0.95;
			
			if(!lost_pos_)
			{
				lost_pos_ = true;
				lost_start_t_ = ros::Time::now();
			}
			else
			{
				if(lost_start_t_ + ros::Duration(duration_t_) < ros::Time::now())
				{
					need_to_reinit_ = true;
				}
				else
				{
					need_to_reinit_ = false;
				}
			}
			if(!need_to_reinit_)
			{
				pub_pos_[0] = pub_pos_[0];
				pub_pos_[1] = pub_pos_[1];
			}
			else
			{
				now_pos_[0] = init_x_;
				now_pos_[1] = init_y_;
				pub_pos_[0] = init_x_;
				pub_pos_[1] = init_y_;
				init_done_ = false;
				
				geometry_msgs::Point HumanPoint;
				geometry_msgs::Quaternion HumanQuaternion;
				geometry_msgs::PoseStamped msgy;
				HumanPoint.x = pub_pos_[0];
				HumanPoint.y = pub_pos_[1];
				HumanPoint.z = 0; 

				HumanQuaternion.x = 0;//|-> Orientation is ignored
				HumanQuaternion.y = 0;//|
				HumanQuaternion.z = 0;//|
				HumanQuaternion.w = 1;//|

				geometry_msgs::Pose HumanPose;
				HumanPose.position = HumanPoint;
				HumanPose.orientation= HumanQuaternion;

				msgy.header.stamp = ros::Time::now();
				msgy.header.frame_id = sensor_frame_id_;
				seq_counter_ = 0;
				msgy.header.seq = seq_counter_;
				msgy.pose = HumanPose;
				pos_pub_.publish(msgy);
			}				
		}
	}
}

void PeopleDetector::run()
{
	geometry_msgs::PoseArray msgx;
	geometry_msgs::PoseStamped msgy;
	ros::Rate loop_rate(15);

	now_pos_.push_back(init_x_);
	now_pos_.push_back(init_y_);
	printf("%d\n", now_pos_.size());
	printf("mow_pos: %f, %f\n", now_pos_[0],now_pos_[1]);

	pub_pos_.push_back(init_x_);
	pub_pos_.push_back(init_y_);
	
	while(ros::ok())
	{
		if(sensor_on_ == true){
			// delete persons who are too near to each other
			poseFilter();
			// ROS_INFO("delete persons");

			if (init_done_ == false){
				humanPoseInit();
				rec_x_.clear();
				rec_y_.clear();
				printf("init_undo\n");
			}
			else
			{
				ROS_ERROR("INIT DONE!");
			}

			//------------------------------------------
			// Copying to proper PoseArray data structure
			std::vector <geometry_msgs::Pose> HumanPoseVector;
			for(int K = 0; K < rec_x_.size(); K++)
			{
				geometry_msgs::Point HumanPoint;
				geometry_msgs::Quaternion HumanQuaternion;
				
				HumanPoint.x = rec_x_[K];
				HumanPoint.y = rec_y_[K];
				HumanPoint.z = 0; 
				
				HumanQuaternion.x = 0;//|-> Orientation is ignored
				HumanQuaternion.y = 0;//|
				HumanQuaternion.z = 0;//|
				HumanQuaternion.w = 1;//|

				geometry_msgs::Pose HumanPose;
				HumanPose.position = HumanPoint;
				HumanPose.orientation= HumanQuaternion;
				HumanPoseVector.push_back(HumanPose);
			}

			// Header config
			msgx.header.stamp = ros::Time::now();
			msgx.header.frame_id = sensor_frame_id_;
			msgx.header.seq = seq_counter_;
			msgx.poses = HumanPoseVector;

			node_pub_.publish(msgx);
			//------------------------------------------
			if (init_done_ == true)
			{
				geometry_msgs::Point HumanPoint;
				geometry_msgs::Quaternion HumanQuaternion;

				HumanPoint.x = pub_pos_[0];
				HumanPoint.y = pub_pos_[1];
				HumanPoint.z = 0; 

				HumanQuaternion.x = 0;//|-> Orientation is ignored
				HumanQuaternion.y = 0;//|
				HumanQuaternion.z = 0;//|
				HumanQuaternion.w = 1;//|

				geometry_msgs::Pose HumanPose;
				HumanPose.position = HumanPoint;
				HumanPose.orientation= HumanQuaternion;

				msgy.header.stamp = ros::Time::now();
				msgy.header.frame_id = sensor_frame_id_;
				msgy.header.seq = seq_counter_;
				msgy.pose = HumanPose;
				pos_pub_.publish(msgy);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
		seq_counter_++;
	}
}
/*
根据提供的代码，可以看出程序的主要功能是从激光雷达数据中检测人体轮廓，并计算出人体在机器人坐标系下的位置。整个程序流程如下：

首先，通过ROS节点初始化函数initialize()配置ROS参数服务器和创建ROS话题发布者与订阅者，以及设置初始化标志位initialized_为true。

接着，通过ROS话题订阅者node_sub_获取激光雷达数据，并将其传给回调函数scanCallback()进行处理。

在scanCallback()函数中，首先通过convertPolarToCartesian()函数将极坐标格式的激光雷达数据转换为直角坐标格式，然后使用extractValidPoints()函数提取有效的点云数据，将其存储在points_成员变量中。

在points_中，使用edge_detector_对象的detectEdge()函数检测人体轮廓的边缘点，并将其存储在edge_points_中。

根据edge_points_计算人体的重心位置和朝向角度，并将其转换为机器人坐标系下的位置和姿态信息，使用ROS消息类型geometry_msgs::PoseStamped进行封装，并发布到话题pos_pub_中。

同时，还会将边缘点集合edge_points_转换为ROS消息类型geometry_msgs::PoseArray进行封装，并发布到话题node_pub_中。

整个程序主要利用了ROS的消息传递机制，通过订阅激光雷达话题获取数据，然后利用边缘检测算法和坐标变换等技术，提取出人体的轮廓信息，并计算出人体的位置和姿态信息，最终发布到相应的ROS话题当中。

*/
/*
计算人体的位置和姿态信息主要是基于从激光雷达数据中提取出的人体边缘点集合进行计算的。具体流程如下：

首先，通过extractValidPoints()函数从激光雷达点云数据中提取出有效的点集points_。

在points_中，使用edge_detector_对象的detectEdge()函数检测人体轮廓的边缘点，并将其存储在edge_points_中。

根据edge_points_计算出人体的质心centroid以及方向角度theta，其中centroid为edge_points_中所有点的平均值，而theta可以通过对centroid周围的点进行极坐标变换后求平均值得到。

将centroid和theta转换为机器人坐标系下的位置和姿态信息，首先根据机器人当前位置（通过ROS TF库获取）将centroid从激光雷达坐标系转换为机器人坐标系下的坐标，然后将theta加上机器人当前朝向角度，得到人体相对于机器人坐标系的朝向角度。

最后，使用ROS消息类型geometry_msgs::PoseStamped进行封装，将人体的位置和姿态信息发布到话题pos_pub_中。

整个过程主要利用了几何学和坐标变换等知识，通过计算人体的质心和方向角度，进而将其转换为机器人坐标系下的位置和姿态信息。
*/
