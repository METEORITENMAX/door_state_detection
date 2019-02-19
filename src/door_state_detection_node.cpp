#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <door_state_detection/door_state_detection.h>

class DoorStateDetect
{
public:
	DoorStateDetect(float angle_min, float angle_max);

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
	ros::Subscriber scan_sub_;
	ros::Publisher scan_pub;
	ros::NodeHandle nh_;
	float min_angle_, max_angle_;
	float max_dis_to_door = 1.5;
};

DoorStateDetect::DoorStateDetect(float angle_min, float angle_max)
: min_angle_(angle_min)
, max_angle_(angle_max)
{

	//nh_.param("button_A", btn_A_, btn_A_);
	ROS_INFO("door_state_detection_node");
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 10, &DoorStateDetect::scanCallback,this);
}

void DoorStateDetect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
	//ROS_INFO("MSG RECEIVED");
	sensor_msgs::LaserScan copy_scan = *laserscan;
	std::vector<float> filtered_scan;
	for(unsigned int i = 0; i < copy_scan.ranges.size(); i++ ){
		//ROS_INFO("MSG FILTER CASE %d",copy_scan.ranges.size());
		/*float new_angle_min = (abs(copy_scan.angle_min + copy_scan.angle_max) + abs(copy_scan.angle_min) - abs(min_angle_)) / copy_scan.angle_increment;
		float new_angle_max = (copy_scan.angle_max + copy_scan.angle_max -abs(copy_scan.angle_min) + max_angle_)/copy_scan.angle_increment;
		std::cout<<new_angle_min<<std::endl;
		std::cout<<new_angle_max<<std::endl;*/
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
		if(avg_dist / filtered_scan.size() > max_dis_to_door)
			std::cout<<"DOOR OPEN"<<std::endl;
		else
			std::cout<<"DOOR CLOSED"<<std::endl;
	}
	scan_pub = nh_.advertise<sensor_msgs::LaserScan>("scanfiltered", 50);
	scan_pub.publish(copy_scan);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laserscan_filter");
	float angle_min, angle_max;
	ros::NodeHandle nh("~");
	nh.getParam("angle_min", angle_min);
	nh.getParam("angle_max", angle_max);
	ROS_INFO("Got parameter: %f", angle_min);
	ROS_INFO("Got parameter: %f", angle_max);
	DoorStateDetect laserscan_filter(angle_min, angle_max);
	while(1)
	{ros::spinOnce();}

	return 0;
}
