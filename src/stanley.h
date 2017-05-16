#ifndef STANLEY_PLANNER_H_
#define STANLEY_PLANNER_H_


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
//	StanleyPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
//	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
//	bool makePlan(const geometry_msgs::PoseStamped& start,
//			const geometry_msgs::PoseStamped& goal,
//			std::vector<geometry_msgs::PoseStamped>& plan);

private:
	//costmap_2d::Costmap2DROS* costmap_ros_;
	//double step_size_, min_dist_from_robot_;
	//costmap_2d::Costmap2D* costmap_;
	//base_global_planner::WorldModel* world_model_;
	//double footprintCost(double x_i, double y_i, double theta_i);
	//bool initialized_;
	NodeHandle n;
    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);
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
	PoseStamped some_pose;
	Odometry state;
	bool condition;
	string childframe;
	string parentframe;
	double x;
	double y;

};


#endif
