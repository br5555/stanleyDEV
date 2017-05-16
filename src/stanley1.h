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
//: public nav_core::BaseLocalPlanner 
namespace local_planner {
class StanleyPlanner{

public:
	StanleyPlanner();
	void pose_callback(const nav_msgs::Odometry data);
	void path_callback(const nav_msgs::Path data);
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
	//tf2::TransformBroadcaster odom_broadcaster;
	ros::NodeHandle n;
    geometry_msgs::TransformStamped transformStamped;
	ros::Subscriber pathSub, odometrySub;
	ros::Publisher speedPub, timePub, plotPub, odom_pub;
	geometry_msgs::PoseStamped final_pose, some_pose;
	nav_msgs::Path path;
	geometry_msgs::Twist cmd_vel;
	double k, v, k2, yaw, ks, x, y, proslo, pocetno, velocity, pauza;
	int i, j;
	nav_msgs::Odometry state_now, state_previous, state;
	ros::Time now, previous;
	bool condition;
	std::string childframe, parentframe;
	



};

};
#endif
