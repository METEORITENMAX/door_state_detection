#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <door_state_detection/door_state_detectionAction.h>


class DoorStateDetect
{
public:
	DoorStateDetect(float angle_min, float angle_max, std::string name, ros::NodeHandle nh);
	void executeCB(const door_state_detection::door_state_detectionGoalConstPtr &goal);

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
	ros::Subscriber scan_sub_;
	ros::Publisher scan_pub;
	ros::NodeHandle nh_;
	bool success_ = false;
	float min_angle_, max_angle_;
	float max_dis_to_door = 1.5;
	std::mutex cv_m_;
	std::condition_variable cv_;
	bool ready_ = false;
	bool processed_ = false;
	bool wait_con = true;

protected:
	actionlib::SimpleActionServer<door_state_detection::door_state_detectionAction> as_;
	std::string action_name_;
	door_state_detection::door_state_detectionFeedback feedback_;
	door_state_detection::door_state_detectionResult result_;
};

DoorStateDetect::DoorStateDetect(float angle_min, float angle_max, std::string name, ros::NodeHandle nh)
: min_angle_(angle_min)
, max_angle_(angle_max)
, action_name_(name)
, nh_(nh)
,as_(nh_, name, boost::bind(&DoorStateDetect::executeCB, this, _1), false)

{
	ROS_INFO("door_state_detection");
	as_.start();
	ROS_INFO("action server started");
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 10, &DoorStateDetect::scanCallback,this);
}
void DoorStateDetect::executeCB(const door_state_detection::door_state_detectionGoalConstPtr &goal)
{
	ros::Rate r(1);
	min_angle_ = goal->min_angle;
	max_angle_ = goal->max_angle;
	ROS_INFO("%s: Executing, check door state with min_angle %f and max_angle %f"
	,action_name_.c_str(), goal->min_angle, goal->max_angle);


	/*
	std::unique_lock<std::mutex> lk(cv_m_);
	std::cerr << "Waiting... \n";
	this->cv_.wait(lk,[this]{return ready_;});
	std::cerr << "...finished waiting ready == true\n";
	*/
	while(wait_con){};
	ROS_INFO("goal reached");
	if (success_) {
		result_.door_state = true;
		as_.setSucceeded(result_);
	} else {
		result_.door_state = false;
		as_.setSucceeded(result_);
	}

}

void DoorStateDetect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
	ROS_INFO("SCAN MSG RECEIVED");
	/*
	std::this_thread::sleep_for(std::chrono::seconds(1));
    {
        std::lock_guard<std::mutex> lk(cv_m_);
        std::cerr << "Notifying...\n";
    }
    cv_.notify_all();

    std::this_thread::sleep_for(std::chrono::seconds(1));
	*/

	sensor_msgs::LaserScan copy_scan = *laserscan;
	std::vector<float> filtered_scan;
	for(unsigned int i = 0; i < copy_scan.ranges.size(); i++ ) {
		if(i < (abs(copy_scan.angle_min + copy_scan.angle_max) + abs(copy_scan.angle_min) - abs(min_angle_)) / copy_scan.angle_increment
		|| i > (copy_scan.angle_max + copy_scan.angle_max -abs(copy_scan.angle_min) + max_angle_)/copy_scan.angle_increment){
			copy_scan.ranges[i] = 0;
			//copy_scan.angle_max = 0;
		}else{
			filtered_scan.push_back(copy_scan.ranges[i]);
		}
		float avg_dist;
		for(int i; i < filtered_scan.size(); i++){
			avg_dist+= filtered_scan[i];
		}
		if(avg_dist / filtered_scan.size() > max_dis_to_door){
			std::cout<<"DOOR OPEN"<<std::endl;
			success_ = true;
			//as_.setSucceeded(result_);
		} else {
			std::cout<<"DOOR CLOSED"<<std::endl;
			success_ = false;
		}
	}
	scan_pub = nh_.advertise<sensor_msgs::LaserScan>("scanfiltered", 50);
	scan_pub.publish(copy_scan);
	wait_con = false;
	/*
	{
        std::lock_guard<std::mutex> lk(cv_m_);
        ready_ = true;
        std::cerr << "Notifying again...\n";
    }
    cv_.notify_all();
	*/
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
	DoorStateDetect laserscan_filter(angle_min, angle_max, "DoorStateDetection", nh);
	ros::spin();

	return 0;
}
