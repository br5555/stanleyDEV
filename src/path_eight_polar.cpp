#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <nav_msgs/Path.h>
#include <math.h>
#include <vector>
#include "geometry_msgs/Quaternion.h"

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
	vector<geometry_msgs::PoseStamped> poses;
	tf2::Quaternion q;
	geometry_msgs::TransformStamped odom_trans;
	std::vector<geometry_msgs::PoseStamped> plan;

};

Path_L::Path_L() {

	pathPub = n.advertise < nav_msgs::Path > ("path", 50);
	createPath();

}

void Path_L::createPath() {

	double x0 = 3;
	double y0 = 4.5;
	double R = 1.5;

	double alpha_start = 0;
	double alpha_end = 360;
	double step = alpha_end / 500;

	path.header.stamp = Time::now();
	path.header.frame_id = "world";
	double alpha_old = alpha_start;

	for (int i = 0; i < 1000; i++) {

		if (i == 500) {
			x0 = 6;
			y0 = 4.5;
			R = 1.5;
			alpha_old = 180 +alpha_start;
		}

		double x = x0 + R * cos(alpha_old * PI / 180.0);
		double y = y0 + R * sin(alpha_old * PI / 180.0);
		currPos.pose.position.x = x;
		currPos.pose.position.y = y;
		currPos.pose.position.z = 0;

		double theta = atan2(x - x0, -y + y0);

		q.setRPY(0, 0, theta);

	currPos.pose.orientation.x = q.x();
     currPos.pose.orientation.y = q.y();
     currPos.pose.orientation.z = q.z();
     currPos.pose.orientation.w = q.w();
		poses.push_back(currPos);
		alpha_old += step;

	}

}
void Path_L::spin() {
	Rate loop_rate(50);
	for (int i = 0; i < 1000; i++) {
	
		currPos = poses.front();
		

		currPos.header.seq = path.header.seq + 1;
		path.header.stamp = Time::now();
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
	init(argc, argv, "pathEight");
	Path_L obj;
	obj.createPath();
	obj.spin();

	return 0;

}

