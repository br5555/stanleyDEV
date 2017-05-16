#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h> 
//#include <base_global_planner/world_model.h>
//#include <base_global_planner/costmap_model.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Transform.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Spawn.h>
#include <math.h>
#include <vector>
#include <string.h>
#include <float.h>
#include <algorithm>  
#include "geometry_msgs/Quaternion.h"  
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define PI 3.14159265
#define E 2.718281828

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace nav_msgs;
using namespace tf2;
using namespace tf2_ros;

class StanleyPlanner {

public:
	StanleyPlanner();
	void run();
	void pose_callback(const Odometry data);
	void path_callback(const Path data);
	bool getCondition();
	double pauza;
private:
	NodeHandle n;
	geometry_msgs::TransformStamped transformStamped;
	Subscriber pathSub;
	Subscriber odometrySub;
	Publisher speedPub;
	Publisher plotPub;
	Publisher timePub;
	PoseStamped final_pose;
	Path path;
	Twist cmd_vel;
	double k;
	double v;
	double k2;
	double yaw;
	Odometry state;
	bool condition;
	string childframe;
	string parentframe;
	double x;
	double y;
	int i;
	int j;
	double proslo;
	double pocetno;
	double velocity;


};

bool StanleyPlanner::getCondition() {
	return condition;
}

void StanleyPlanner::pose_callback(const Odometry msg) {
    
    	proslo = pocetno;
    	pocetno= ros::Time::now().toSec();
    if(!condition){
        return;
    }
    //ros::Duration(pauza).sleep();

    
	state = msg;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	while (true) {
		int i = 0;
		try {
			transformStamped = tfBuffer.lookupTransform("world", "robot0",
					ros::Time(0));
			break;
		} catch (tf2::TransformException &ex) {
			continue;
			ROS_INFO("Pao sam %d", i++);

		}
	}

	tf2::Quaternion quaternion = tf2::Quaternion(state.pose.pose.orientation.x,
			state.pose.pose.orientation.y, state.pose.pose.orientation.z,
			state.pose.pose.orientation.w);

	double roll, pitch;

	tf2::Matrix3x3 m(quaternion);

	m.getRPY(roll, pitch, yaw);

	parentframe = state.header.frame_id;
	childframe = state.child_frame_id;

	Rate loop_rate(50);

	std_msgs::Float32 cross_track_err;
	std_msgs::Float32MultiArray time_plot;

	double robot_X = state.pose.pose.position.x
			+ transformStamped.transform.translation.x;
	double robot_Y = state.pose.pose.position.y
			+ transformStamped.transform.translation.y;
	double robot_Head = yaw;

	double e = 10000;
	double predznak = 1.0;
	int heading_path = 0;
	PoseStamped some_pose;

	some_pose = path.poses.at(i);

	if (i == path.poses.size() - 1) {
		i = path.poses.size() - 1;
		cmd_vel.linear.x = 0;
	}
	x = some_pose.pose.position.x;
	y = some_pose.pose.position.y;
    ROS_INFO("%d", i);
	double test_distance_x = pow(x - robot_X, 2);
	double test_distance_y = pow(y - robot_Y, 2);
	double test_distance = sqrt(test_distance_y + test_distance_x);
	if(test_distance < 0.2){
	  	i++;
	}
	 //pauza = 2;//test_distance / (v*10);
	//ROS_INFO("Pauza %d je %f",i, pauza);
	if (abs(e) > abs(test_distance)) {
		PointStamped tocka_mapa, tocka_robot;

		double a = x;
		double b = y;

		final_pose = some_pose;
		tocka_mapa.point.x = x;
		tocka_mapa.point.y = y;
		tocka_mapa.point.z = 0.0;
		tocka_mapa.header.frame_id = parentframe;

		double deltaX = some_pose.pose.position.x
				- transformStamped.transform.translation.x;
		double deltaY = some_pose.pose.position.y
				- transformStamped.transform.translation.y;

		
        
		if (deltaY < 0)
			predznak = -1;
		else
			predznak = 1;

		e = test_distance * predznak;

	}

	cross_track_err.data = e;

	quaternion = tf2::Quaternion(final_pose.pose.orientation.x,
			final_pose.pose.orientation.y, final_pose.pose.orientation.z,
			final_pose.pose.orientation.w);

	double heading_puta;
	m = tf2::Matrix3x3(quaternion);

	m.getRPY(roll, pitch, heading_puta);

	double heading_err = -1 * (robot_Head - heading_puta);
	double arg = (k * e) / v;

	double arctg = atan2(arg, 1);
	double omega = k2 * (heading_err + arctg);

	if (omega > 0.8)
		omega = 0.8;
	else if (omega < -0.8)
		omega = -0.8;
	cmd_vel.angular.z = omega;

	
	
	double kvel = pow(E,-10*abs(heading_err));
	velocity = (test_distance / (pocetno - proslo))*kvel;
	if(abs(heading_err)>=PI){
		velocity = 0;
	}
	
	cmd_vel.linear.x = velocity;

	speedPub.publish(cmd_vel);

	plotPub.publish(cross_track_err);
    spinOnce();
    ROS_INFO("Nikad se ne izvodim");



}

void StanleyPlanner::path_callback(const Path msg) {
	path = msg;
	condition = true;

}

StanleyPlanner::StanleyPlanner() {

	pathSub = n.subscribe("path", 1, &StanleyPlanner::path_callback, this);
	odometrySub = n.subscribe("robot0/odom", 1, &StanleyPlanner::pose_callback,
			this);
	speedPub = n.advertise < Twist > ("robot0/cmd_vel", 1);
	plotPub = n.advertise < Float32 > ("/plot", 1);
	timePub = n.advertise < Float32MultiArray > ("time", 1);
	k = 0.1;
	v = 0.2;
	k2 = 10;
	yaw = 0;
	velocity  = 0;
	condition = false;
	childframe = "";
	parentframe = "";
	i = 0;
	j = 0;
	pauza = 0.0;

}

int main(int argc, char **argv)

{
	init(argc, argv, "Stanley");
	StanleyPlanner obj;
	while (true) {

		spinOnce();

	}

	return 0;

}

