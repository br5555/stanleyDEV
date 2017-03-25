#!/usr/bin/env python
import sys
import roslib
import rospy
import rospkg
from nav_msgs.msg import Path
from std_msgs.msg import Header, String, Float64
from geometry_msgs.msg import PoseStamped, Pose

class PathL():


    def __init__(self):
        self.pathPub = rospy.Publisher("path", Path, queue_size=1)
        self.path = Path()

    def createPath(self):         
    
    # Creating a linear path which goes from (1,3) to (8,3)
    # Theta equals zero which is in quaternion (1, 0, 0, 0)
       
   
        dx = 7 / 300.0
        path_local = [(1, 3)]

        curr_pos = PoseStamped()

        curr_pos.pose.position.x = 1
        curr_pos.pose.position.y = 3
        curr_pos.pose.position.z = 0

        curr_pos.pose.orientation.x = 1
        curr_pos.pose.orientation.y = 0
        curr_pos.pose.orientation.z = 0
        curr_pos.pose.orientation.w = 0

        curr_pos.header.seq = self.path.header.seq + 1
        self.path.header.frame_id = "world"  #---------------->header je world
        self.path.header.stamp = rospy.Time.now()
        curr_pos.header = self.path.header
        curr_pos.header.frame_id = "world"

        self.path.poses.append(curr_pos)

        for i in range (1, 121):
            curr_pos = PoseStamped()
            x, y = path_local[i - 1]
            path_local.append((x + dx, y))

            curr_pos.pose.position.x = x + dx
            curr_pos.pose.position.y = y
            curr_pos.pose.position.z = 0

            curr_pos.pose.orientation.x = 1
            curr_pos.pose.orientation.y = 0
            curr_pos.pose.orientation.z = 0
            curr_pos.pose.orientation.w = 0

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)

        for i in range (121, 181):
            curr_pos = PoseStamped()
            x, y = path_local[i - 1]
            path_local.append((x + dx, y + dx))

            curr_pos.pose.position.x = x + dx
            curr_pos.pose.position.y = y + dx
            curr_pos.pose.position.z = 0

            # angle of 45 degrees
            curr_pos.pose.orientation.x = 0.9238795
            curr_pos.pose.orientation.y = 0.38268343
            curr_pos.pose.orientation.z = 0
            curr_pos.pose.orientation.w = 0

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)

        for i in range (181, 330):
            curr_pos = PoseStamped()
            x, y = path_local[i - 1]
            path_local.append((x + dx, y))

            curr_pos.pose.position.x = x + dx
            curr_pos.pose.position.y = y
            curr_pos.pose.position.z = 0

            curr_pos.pose.orientation.x = 1
            curr_pos.pose.orientation.y = 0
            curr_pos.pose.orientation.z = 0
            curr_pos.pose.orientation.w = 0

            curr_pos.header.seq = self.path.header.seq + 1
            self.path.header.stamp = rospy.Time.now()
            curr_pos.header.stamp = self.path.header.stamp
            self.path.poses.append(curr_pos)
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            print "publishamo", 'dx', dx   
            self.pathPub.publish(self.path)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('pathLinear')
    path = PathL()
    path.createPath()
    rospy.spin()
