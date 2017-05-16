#include "stanley1.h"

//PLUGINLIB_EXPORT_CLASS(local_planner::StanleyPlanner, nav_core::BaseLocalPlanner)

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace nav_msgs;
using namespace tf2;

#define PI 3.14159265
#define E 2.718281828


namespace local_planner {



void StanleyPlanner::pose_callback(const Odometry msg) {
	proslo = pocetno;
	pocetno = ros::Time::now().toSec();
	if (!condition || (pocetno - proslo) > 1) {
		return;
	}


	state = msg;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	while (true) {
		
		try {
		    tfBuffer.lookupTransform("world", "robot0",
					ros::Time(0));
			transformStamped = tfBuffer.lookupTransform("world", "robot0",
					ros::Time(0));
			break;
		} catch (tf2::TransformException &ex) {
		    //ROS_INFO("Pao sam");
			continue;
			

		}
	}
		 tf2::Quaternion quaternion(state.pose.pose.orientation.x,
					state.pose.pose.orientation.y,
					state.pose.pose.orientation.z,
					state.pose.pose.orientation.w);
			
			double roll, pitch;

			tf2::Matrix3x3 m(quaternion);
			m.getRPY(roll, pitch, yaw);
			
	double robot_X =  transformStamped.transform.translation.x;
	double robot_Y = transformStamped.transform.translation.y;
	double a11 = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
	double a12 = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
	double a21 = robot_X - path.poses[i].pose.position.x;
	double a22 = robot_Y - path.poses[i].pose.position.y;
	double Xprod = a11*a22-a12*a21;
	int sign;
	if(Xprod >0){
	    sign =-1;
	}else if(Xprod <0){
	    sign=1;
	}else{
	    sign =0;
	}
	double angleRobot = yaw;//atan2(robot_Y, robot_X);


	//ROS_INFO("yaw je %f robot %f | %f", yaw, robot_X, robot_Y);

	double anglePath = atan2(
			path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y,
			path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
			//if(anglePath > PI){
			  //  anglePath -=(2*PI);
			//}else if(anglePath < -PI){
			  //  anglePath +=(2*PI);
			//}
			
			//if(yaw > PI){
			  //  yaw -=(2*PI);
			//}else if(yaw < -PI){
			  //  yaw +=(2*PI);
			//}
    double angg = atan(
			(path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y)/
			(path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x));
	double delta = -angleRobot + anglePath;
    
	double modulDistance = sqrt(pow(path.poses[i+1].pose.position.y-robot_Y, 2)+pow(path.poses[i+1].pose.position.x-robot_X, 2));
            ROS_INFO(" %f |%f |%f| %f", path.poses[i + 1].pose.position.y ,robot_Y ,
			path.poses[i + 1].pose.position.x ,robot_X);
			
			double angle = delta + sign*atan2(k * modulDistance,1);
            //ROS_INFO("Angle je %f PathAngle je %f", angleRobot, anglePath);
            //ROS_INFO("Delta je %f modul je %f", delta, modulDistance);
			
			double omega = angle/(10*(pocetno - proslo)) ;
            //ROS_INFO("Omega je %f ", omega);
            
            
            double kvel = pow(E,-6*abs(delta));
			velocity = 0.2*kvel;//(modulDistance / (pocetno - proslo))*kvel;
			if(abs(delta)>=PI){
			    velocity = 0;
			}
           
            //ROS_INFO("Omega je %d", i);
           
            if(omega < -2){
                omega = -2;
            }else if(omega > 2){
                omega = 2;
            }
            
            
			cmd_vel.angular.z =omega;
			cmd_vel.linear.x = velocity;

			speedPub.publish(cmd_vel);
			
			if(modulDistance < 0.5){
			     i++;
			}
           
			spinOnce();
			



	}

	void StanleyPlanner::path_callback(const Path msg) {
		path = msg;
		condition = true;
	}

	StanleyPlanner::StanleyPlanner() {

		pathSub = n.subscribe("path", 1, &StanleyPlanner::path_callback, this);
		odometrySub = n.subscribe("robot0/odom", 1,
				&StanleyPlanner::pose_callback, this);
		speedPub = n.advertise < Twist > ("robot0/cmd_vel", 1);
		plotPub = n.advertise < Float32 > ("/plot", 1);
		timePub = n.advertise < Float32MultiArray > ("time", 1);
		odom_pub = n.advertise < nav_msgs::Odometry > ("odom", 1);
		k = 1;
		v = 0.2;
		k2 = 0.8;
		yaw = 0;
		ks = 10;
		condition = false;
		childframe = "";
		parentframe = "";
		i = 0;
	    j = 0;

	}}
;

	int main(int argc, char **argv)

	{
		init(argc, argv, "Stanley");
		local_planner::StanleyPlanner obj;
		ros::Duration(5.).sleep();
		while(true){
		    spinOnce();
		}

		return 0;

	}


