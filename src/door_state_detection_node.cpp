#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

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
	sensor_msgs::LaserScan filtered_scan = *laserscan;
	for(unsigned int i = 0; i < filtered_scan.ranges.size(); i++ ){
		//ROS_INFO("MSG FILTER CASE %d",filtered_scan.ranges.size());
		float new_angle_min = ((filtered_scan.angle_min + filtered_scan.angle_max) + fabs(min_angle_)) / filtered_scan.angle_increment;
		float new_angle_max = ((filtered_scan.angle_max + filtered_scan.angle_max) - max_angle_)/ filtered_scan.angle_increment;
		std::cout<<new_angle_min<<std::endl;
		std::cout<<new_angle_max<<std::endl;
		if(i < ((filtered_scan.angle_min + filtered_scan.angle_max) + fabs(min_angle_)) / filtered_scan.angle_increment
		|| i > ((filtered_scan.angle_max + filtered_scan.angle_max) - max_angle_)/ filtered_scan.angle_increment ){
			filtered_scan.ranges[i] = 0;
			//filtered_scan.angle_max = 0;
		}
	}
	scan_pub = nh_.advertise<sensor_msgs::LaserScan>("scanfiltered", 50);
	scan_pub.publish(filtered_scan);
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
