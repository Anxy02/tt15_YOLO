#include <auto_follow/people_detector.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "people_detector_node");
	ros::NodeHandle nh;
	PeopleDetector people_detector;
	if(people_detector.initialize(nh))
	{
		people_detector.run();
	}
	return 0;
}