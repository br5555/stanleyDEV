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
	 nav_msgs::Path gui_path;
	geometry_msgs::PoseStamped currPos;
	nav_msgs::Path path_local;
	vector<geometry_msgs::PoseStamped> poses;
	tf2::Quaternion q;
	geometry_msgs::TransformStamped odom_trans;
	std::vector<geometry_msgs::PoseStamped> plan;

};

Path_L::Path_L() {

	pathPub = n.advertise < nav_msgs::Path > ("path", 1);
	createPath();

}

void Path_L::createPath() {

	double x0 = 4.5;
	double y0 = 4.5;
	double R = 1.5;

	double alpha_start = 0;
	double alpha_end = 360;
	double step = alpha_end / 500;

	path.header.stamp = Time::now();
	path.header.frame_id = "world";

	double x = x0 + R * cos(alpha_start * PI / 180.0);
	double y = y0 + R * sin(alpha_start * PI / 180.0);

	currPos.pose.position.x = x;
	currPos.pose.position.y = y;
	currPos.pose.position.z = 0;
	double alpha_old = alpha_start;

	double theta = atan2(0.0, 0.0);


    q.setRPY(0, 0, theta);

	currPos.pose.orientation.x = q.x();
     currPos.pose.orientation.y = q.y();
     currPos.pose.orientation.z = q.z();
     currPos.pose.orientation.w = q.w();
	poses.push_back(currPos);

	for (int i = 1; i < 500; i++) {

		alpha_old += step;
		x = x0 + R * cos(alpha_old * PI / 180.0);
		y = y0 + R * sin(alpha_old * PI / 180.0);
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
		
		
	}

}
void Path_L::spin() {
			Rate loop_rate(50);
	for (int i = 0; i < 500; i++) {

		currPos = poses.front();
		double x = currPos.pose.position.x;
		double y = currPos.pose.position.y;
		ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n",x, y,0.0);
		currPos.header.seq = path.header.seq + 1;
		path.header.stamp = Time::now();
		currPos.header.stamp = path.header.stamp;
		//ROS_INFO("uuuuuuuuuuuuuuuuhhhhuuuuuuuuuuuuuuuuuuuuuuuuuuu\n");
		
		plan.push_back(currPos);
			//ROS_INFO("***************************\n");
		path.poses = plan;
			//ROS_INFO("opopopopopopopop\n");
		

		
		poses.erase(poses.begin());
		}
		//ROS_INFO("++++++++++++++++++++++++++++++++++\n");
		while (ros::ok()) {
			pathPub.publish(path);
			spinOnce();
			loop_rate.sleep();

		}

	
}

int main(int argc, char **argv)

{
	init(argc, argv, "pathCircle");
	Path_L obj;
	obj.createPath();
	obj.spin();

	return 0;

}

