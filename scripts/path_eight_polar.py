#!/usr/bin/env python
import sys
import roslib
import rospy
import rospkg
from nav_msgs.msg import Path
from std_msgs.msg import Header, String, Float64
from geometry_msgs.msg import PoseStamped, Pose
import math
import tf
class PathL():


    def __init__(self):
        self.pathPub = rospy.Publisher("path", Path, queue_size=1)
        self.path = Path()

    def createPath(self):         
        """
        This method generates a circle using the polar coordinates, so that it is possible
        easy to change the radius or center of the cricle. It is the type Path().
        """
        
        xo = 3
        yo = 4.5
        R = 1.5


        alpha_start = 0
        alpha_end = math.radians(360)
        step = alpha_end / 500

        
        path_local = []
                
        x = xo + R * math.cos(alpha_start)
        y = yo + R * math.sin(alpha_start)
        path_local.append((x, y))
        alpha_old = alpha_start

        for i in range (1, 500):
            alpha_old = alpha_old + step
            x = xo + R * math.cos(alpha_old)           
            y = yo + R * math.sin(alpha_old)
            path_local.append((x, y))                     
            
        for i in range(1, 500):
            x, y = path_local[i]
            x2, y2 = path_local[i - 1]
            test_distance_x = (x - x2) ** 2
            test_distance_y = (y - y2) ** 2
            test_distance = math.sqrt((test_distance_y + test_distance_x))
            # print "udaljenost", test_distance, x, y
 
        xo = 6
        yo = 4.5
        R = 1.5


        alpha_start = math.radians(180)
        alpha_end = math.radians(360)
        step = alpha_end / 500

        
        path2_local = []
                
        x = xo + R * math.cos(alpha_start)
        y = yo + R * math.sin(alpha_start)
        path2_local.append((x, y))
        alpha_old = alpha_start

        for i in range (1, 500):
            alpha_old = alpha_old - step
            x = xo + R * math.cos(alpha_old)           
            y = yo + R * math.sin(alpha_old)
            path2_local.append((x, y))                     
            
        for i in range(1, 500):
            x, y = path2_local[i]
            x2, y2 = path2_local[i - 1]
            test_distance_x = (x - x2) ** 2
            test_distance_y = (y - y2) ** 2
            test_distance = math.sqrt((test_distance_y + test_distance_x))
            # print "udaljenost", test_distance, x, y


        """
        In this part we are changing the path vector of points into Path Array with coordinates, heading and frames
        First we want to append the lower half of the first circle, then the second and finally the upper half of the first circle. 

        """
        self.path.header.frame_id = "world"
        self.path.header.stamp = rospy.Time.now()
        a = 0    
        xo = 3
        yo = 4.5    

        for i in range (250, 500):
            curr_pos = PoseStamped()
            x, y = path_local[i]

            curr_pos.pose.position.x = x
            curr_pos.pose.position.y = y
            curr_pos.pose.position.z = 0
            # calculating orientation
            theta = math.atan2(x - xo, -(y - yo))
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

            curr_pos.pose.orientation.x = quaternion[0]
            curr_pos.pose.orientation.y = quaternion[1]
            curr_pos.pose.orientation.z = quaternion[2]
            curr_pos.pose.orientation.w = quaternion[3]

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)
            print "pozicija", x, y, "theta", a
            a += 1
        xo = 6
        yo = 4.5
        R = 1.5
        for i in range (0, 250):
            curr_pos = PoseStamped()
            x, y = path2_local[i]

            curr_pos.pose.position.x = x
            curr_pos.pose.position.y = y
            curr_pos.pose.position.z = 0
            # calculating orientation
            theta = math.atan2(-(x - xo), (y - yo))
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

            curr_pos.pose.orientation.x = quaternion[0]
            curr_pos.pose.orientation.y = quaternion[1]
            curr_pos.pose.orientation.z = quaternion[2]
            curr_pos.pose.orientation.w = quaternion[3]

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)
            # print "pozicija", x, y, "theta", a
            a += 1

        for i in range (250, 500):
            curr_pos = PoseStamped()
            x, y = path2_local[i]

            curr_pos.pose.position.x = x
            curr_pos.pose.position.y = y
            curr_pos.pose.position.z = 0
            # calculating orientation
            theta = math.atan2(-(x - xo), (y - yo))
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

            curr_pos.pose.orientation.x = quaternion[0]
            curr_pos.pose.orientation.y = quaternion[1]
            curr_pos.pose.orientation.z = quaternion[2]
            curr_pos.pose.orientation.w = quaternion[3]

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)
            # print "pozicija", x, y, "theta", a
            a += 1
        xo = 3
        yo = 4.5
        for i in range (0, 250):
            curr_pos = PoseStamped()
            x, y = path_local[i]

            curr_pos.pose.position.x = x
            curr_pos.pose.position.y = y
            curr_pos.pose.position.z = 0
            # calculating orientation
            theta = math.atan2(x - xo, -(y - yo))
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

            curr_pos.pose.orientation.x = quaternion[0]
            curr_pos.pose.orientation.y = quaternion[1]
            curr_pos.pose.orientation.z = quaternion[2]
            curr_pos.pose.orientation.w = quaternion[3]

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)
            # print "pozicija", x, y, "theta", a
            a += 1
            
        r = rospy.Rate(50)


        while not rospy.is_shutdown():
          #  print "publishamo"   
            self.pathPub.publish(self.path)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('pathCircle')
    path = PathL()
    path.createPath()
    rospy.spin()
