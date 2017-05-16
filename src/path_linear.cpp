#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <nav_msgs/Path.h>
#include <math.h>
#include <vector>
#include <exception>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace nav_msgs;
using namespace tf2;
#define PI 3.14159265

class Path_L {

public:
	Path_L();
	void createPath();
	void spin();

private:
	NodeHandle n;
	Subscriber sub;
	Publisher pathPub;
	nav_msgs::Path path;
	geometry_msgs::PoseStamped currPos;
	nav_msgs::Path path_local;
	vector<PoseStamped> poses;
	tf2::Quaternion q;
	geometry_msgs::TransformStamped odom_trans;
	std::vector<geometry_msgs::PoseStamped> plan;

};

Path_L::Path_L() {

	pathPub = n.advertise < nav_msgs::Path > ("path", 50);

	createPath();

}

void Path_L::createPath() {

	double dx = 7 / 300.0;

	path.header.stamp = Time::now();
	path.header.frame_id = "world";

	currPos.pose.position.x = 1;
	currPos.pose.position.y = 3;
	currPos.pose.position.z = 0;

	currPos.pose.orientation.x = 1;
	currPos.pose.orientation.y = 0;
	currPos.pose.orientation.z = 0;
	currPos.pose.orientation.w = 0;

	currPos.header.seq = path.header.seq + 1;
	path.header.frame_id = "world";
	path.header.stamp = Time::now();
	currPos.header = path.header;
	currPos.header.frame_id = "world";

	poses.push_back(currPos);

	for (int i = 1; i < 121; i++) {

		currPos.pose.position.x += dx;
		currPos.pose.position.y += 0;
		currPos.pose.position.z = 0;

		currPos.pose.orientation.x = 1;
		currPos.pose.orientation.y = 0;
		currPos.pose.orientation.z = 0;
		currPos.pose.orientation.w = 0;

		currPos.header.seq = path.header.seq + 1;
		path.header.stamp = Time::now();
		currPos.header.stamp = path.header.stamp;
		poses.push_back(currPos);
	}
	for (int i = 121; i < 181; i++) {

		currPos.pose.position.x += dx;
		currPos.pose.position.y += dx;
		currPos.pose.position.z = 0;

		currPos.pose.orientation.x = 0.9238795;
		currPos.pose.orientation.y = 0.38268343;
		currPos.pose.orientation.z = 0;
		currPos.pose.orientation.w = 0;

		currPos.header.seq = path.header.seq + 1;
		path.header.stamp = Time::now();
		currPos.header.stamp = path.header.stamp;
		poses[i] = currPos;
	}
	for (int i = 181; i < 330; i++) {

		currPos.pose.position.x += dx;
		currPos.pose.position.y += 0;
		currPos.pose.position.z = 0;

		currPos.pose.orientation.x = 1;
		currPos.pose.orientation.y = 0;
		currPos.pose.orientation.z = 0;
		currPos.pose.orientation.w = 0;

		currPos.header.seq = path.header.seq + 1;
		path.header.stamp = Time::now();
		currPos.header.stamp = path.header.stamp;
		poses.push_back(currPos);
	}

}
void Path_L::spin() {
			ros::Rate loop_rate(50);
	for (int i = 0; i < 330; i++) {

		currPos = poses.front();
		

		currPos.header.seq = path.header.seq + 1;
		path.header.stamp = ros::Time::now();
		currPos.header.stamp = path.header.stamp;
		plan.push_back(currPos);
			//ROS_INFO("***************************\n");
		path.poses = plan;

		poses.erase(poses.begin());
}
		while (ros::ok()) {
			pathPub.publish(path);
			ros::spinOnce();
			loop_rate.sleep();

		}

	
}

int main(int argc, char **argv)

{
	init(argc, argv, "pathLinear");
	Path_L obj;
	obj.createPath();
	obj.spin();

	return 0;

}

