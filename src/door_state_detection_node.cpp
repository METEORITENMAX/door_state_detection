#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <door_state_detection/door_state_detectionAction.h>

class DoorStateDetect
{
public:
	DoorStateDetect(ros::NodeHandle nh, float angle_min, float angle_max, std::string name);
	void goalCB();
	void preemptCB();

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
	ros::Subscriber scan_sub_;
	ros::Publisher scan_pub;
	ros::NodeHandle nh_;
	float min_angle_, max_angle_;
	std::string action_name_;
	actionlib::SimpleActionServer<door_state_detection::door_state_detectionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.	
	float max_dis_to_door = 1.5;
  // create messages that are used to published feedback/result
  door_state_detection::door_state_detectionAction feedback_;
  door_state_detection::door_state_detectionResult result_;
  bool lock_ = false ;
  int msgs_count_ = 0;
};

DoorStateDetect::DoorStateDetect(ros::NodeHandle nh,float angle_min, float angle_max, std::string name)
: nh_(nh)
, min_angle_(angle_min)
, max_angle_(angle_max)
, action_name_(name)
, as_(nh_, action_name_, false)
{
	as_.registerGoalCallback(boost::bind(&DoorStateDetect::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&DoorStateDetect::preemptCB, this));
	as_.start();
	ROS_INFO("door_state_detection_as_started");
}
void DoorStateDetect::goalCB()
{
	ROS_INFO("goal recevied");
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &DoorStateDetect::scanCallback,this);
	ros::spinOnce();
	const door_state_detection::door_state_detectionGoalConstPtr goal = as_.acceptNewGoal();
	max_angle_ = goal->max_angle;
	min_angle_ = goal->min_angle;

	//result_.action_result.result.door_state = true ;
	//as_.setSucceeded(result_);
}
void DoorStateDetect::preemptCB()
{
	ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}
void DoorStateDetect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
	msgs_count_++;
	if (!as_.isActive()) {
		return;
	}
	sensor_msgs::LaserScan copy_scan = *laserscan;
	std::vector<float> filtered_scan;

	for (unsigned int i = 0; i < copy_scan.ranges.size(); i++ ) {
		if (i < (abs(copy_scan.angle_min + copy_scan.angle_max) + abs(copy_scan.angle_min) - abs(min_angle_)) / copy_scan.angle_increment
		|| i > (copy_scan.angle_max + copy_scan.angle_max -abs(copy_scan.angle_min) + max_angle_)/copy_scan.angle_increment) {
			copy_scan.ranges[i] = 0;
			//copy_scan.angle_max = 0;
		} else {
			filtered_scan.push_back(copy_scan.ranges[i]);
		}
	}

		float avg_dist;
		for (int i; i < filtered_scan.size(); i++) {
			avg_dist+= filtered_scan[i];
		}
		if (avg_dist / filtered_scan.size() > max_dis_to_door) {
			std::cout<<"DOOR OPEN "<<msgs_count_<<std::endl;
			result_.door_state = true;
			as_.setSucceeded(result_);
		} else {
			std::cout<<"DOOR CLOSED "<<msgs_count_<<std::endl;
			result_.door_state = false;
			as_.setSucceeded(result_);
		}

	//lock_ = true;
	scan_pub = nh_.advertise<sensor_msgs::LaserScan>("scanfiltered", 50);
	scan_pub.publish(copy_scan);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "DoorStateDetection");
	float angle_min, angle_max;
	ros::NodeHandle nh("~");
	nh.getParam("angle_min", angle_min);
	nh.getParam("angle_max", angle_max);
	ROS_INFO("Got parameter: %f", angle_min);
	ROS_INFO("Got parameter: %f", angle_max);
	//ROS_INFO("Got parameter: %s", ros::this_node::getName().c_str());
	DoorStateDetect dsd(nh, angle_min, angle_max, ros::this_node::getName().c_str());
	ros::spin();

	return 0;
}
