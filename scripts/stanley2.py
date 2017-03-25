#!/usr/bin/env python
import sys
import roslib
import rospy
import rospkg
from std_msgs.msg import Empty, String, Float64, Float32MultiArray, Float32
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped, Transform
import math
from nav_msgs.msg import Odometry, Path
import tf



class algorithmS():

    def __init__(self):
        self.k=1.5
        self.v=0.2
        self.k2=0.8  #0.13
        rospy.Subscriber("robot0/odom", Odometry, self.pose_callback)
        rospy.Subscriber("path", Path, self.path_callback)
        self.speedPub = rospy.Publisher("robot0/cmd_vel", Twist, queue_size=1)
        self.plotPub = rospy.Publisher("/plot", Float32, queue_size=1)
        self.timePub = rospy.Publisher("time", Float32MultiArray, queue_size=1)
        self.cmd_vel=Twist() #brzina koju cemo slati
        self.yaw=0
        self.t= tf.TransformListener()
        self.parentframe=""
        self.childframe=""
        self.cmd_vel.linear.x=self.v
        self.path=Path()
        self.condition=False
        self.state=Odometry()

    def path_callback(self, data):
        if self.condition is False:
            self.path=data
            self.condition=True
            



    def pose_callback(self, data):
        self.state= data
        quaternion = (
            self.state.pose.pose.orientation.x,
            self.state.pose.pose.orientation.y,
            self.state.pose.pose.orientation.z,
            self.state.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        self.yaw = euler[2]
        self.parentframe=self.state.header.frame_id
        self.childframe=self.state.child_frame_id
    
    def run(self):
    
        
        r=rospy.Rate(50)
        begin = rospy.Time.now() #pocetak vremena
        cross_track_err=Float32()
        time_plot=Float32MultiArray()
        cond=True
        counter=0
        index_old=0
        while not rospy.is_shutdown():

            robot_X, robot_Y=self.state.pose.pose.position.x, self.state.pose.pose.position.y
            robot_Head=self.yaw
            some_pose=PoseStamped()

            e=10000
            predznak=1.0
            heading_path=0 #samo za linearnu!!!!!
            #u for petlji racunamo najmanju udaljenost od putanje (crosstrack error)
            i=0
            for some_pose in self.path.poses:
    

                x=some_pose.pose.position.x
                y=some_pose.pose.position.y

                test_distance_x=(x-robot_X)**2
                test_distance_y=(y-robot_Y)**2
                test_distance=math.sqrt(test_distance_y+test_distance_x)

                if abs(e) > abs(test_distance):
                    tocka_mapa=PointStamped()
                    final_pose=PoseStamped()
                    a, b = x, y 

                    #uzimamo poziciju u globalnim, pretvaramo u robotske i provjeravamo predznak, kako bi znali 
                    #je li robot lijevo ili  desno od patha
                    final_pose=some_pose
                    tocka_mapa.point.x=x
                    tocka_mapa.point.y=y
                    tocka_mapa.point.z=0.0
                    tocka_mapa.header.frame_id=self.parentframe

                    tocka_robot=self.t.transformPoint(self.childframe, tocka_mapa)

 
                    if  tocka_robot.point.y < 0:
                        predznak=-1
                    else:
                        predznak=1
                    e=test_distance*predznak
            #print "cross", e
            cross_track_err=float(e)

            
                                                           
            #Odabrali smo najblizu tocku (final_pose), treba nam heading iz quaternion u euler
            quaternion = (
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            heading_puta = euler[2]


            heading_err=-1*(robot_Head-heading_puta)
            arg=(self.k*e)/self.v

            arctg=math.atan(arg)

            print "heading error:", heading_err, "x:", final_pose.pose.position.x, "y:", final_pose.pose.position.y
            omega=self.k2*(heading_err+arctg)  
            #omega=self.k2*arctg+heading_err
            #print "omega", omega
            #print "arg",arctg
            if omega > 0.8:
                omega=0.8
            elif omega < -0.8:
                omega=-0.8

            self.cmd_vel.angular.z=omega
            self.speedPub.publish(self.cmd_vel)
            self.plotPub.publish(cross_track_err)          

            r.sleep()

    



if __name__=="__main__":
    rospy.init_node('Stanley')
    algorithm=algorithmS()
    while True:
        print "cekamo"
        if algorithm.condition is True:
            algorithm.run()
            rospy.spin()
