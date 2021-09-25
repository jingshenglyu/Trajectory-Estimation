#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
import random
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from std_msgs.msg import Int8

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2

class ExploringMaze():
    def __init__(self):  
        rospy.init_node('exploring_maze_pro', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # Pause time at each target position (Unit: s)
        self.rest_time = rospy.get_param("~rest_time", 0.5)  

        # Posting of control robots  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5) 
 
        # Create a Subscriber, subscribe to /exploring_cmd
        rospy.Subscriber("/exploring_cmd", Int8, self.cmdCallback)

        # Subscribe to the move_base server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  
        
        # 60s waiting time limit  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
 
        # Variables for saving runtime   
        start_time = rospy.Time.now()  
        running_time = 0  

        rospy.loginfo("Starting exploring maze")  
        
        # Set up the initial position
        start_location = Pose(Point(0, 0, 0), Quaternion(0.000, 0.000, 0.709016873598, 0.705191515089))  
 
        locations = []  

        locations.append(Pose(Point(-0.002, 7.663, 0.000),  Quaternion(0.000, 0.000, 0.709016873598, 0.705191515089)))  
        locations.append(Pose(Point(3.273, 4.079, 0.000),  Quaternion(0.000, 0.000, 0.0123661873459, 0.999923535782)))
        locations.append(Pose(Point(3.529, 7.763, 0.000),  Quaternion(0.000, 0.000, -0.163554119075, 0.986534363382)))
        locations.append(Pose(Point(2.729, 1.168, 0.000),  Quaternion(0.000, 0.000, -0.709519194356, 0.704686109442)))  
        locations.append(Pose(Point(7.525, 7.936, 0.000),  Quaternion(0.000, 0.000, -0.636628380104, 0.771170737026)))  
        locations.append(Pose(Point(7.352, 0.658, 0.000),  Quaternion(0.000, 0.000, 0.424508317953, 0.905424037669)))  

        # Initial value of the command
        self.exploring_cmd = 0
        
        location = 0
       
        # Start of main loop, random navigation  
        while not rospy.is_shutdown():
			# Set the next random target point
			self.goal = MoveBaseGoal()
			self.goal.target_pose.pose = start_location
			self.goal.target_pose.header.frame_id = 'map'
			self.goal.target_pose.header.stamp = rospy.Time.now()
			
			if self.exploring_cmd is STATUS_EXPLORING:
				self.goal.target_pose.pose=locations[location]
				location =location+1
				if location is 6:
					location = 0
			elif self.exploring_cmd is STATUS_CLOSE_TARGET:
				rospy.sleep(0.1)
				continue
			elif self.exploring_cmd is STATUS_GO_HOME:
				self.goal.target_pose.pose.position.x = 0
				self.goal.target_pose.pose.position.y = 0
		
			# Let the user know the next location
			rospy.loginfo("Going to:" + str(self.goal.target_pose.pose))

			# Moving on to the next position
			self.move_base.send_goal(self.goal)
			
			# Five-minute time limit
			finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

			# Check if you have arrived successfully
			if not finished_within_time:
				self.move_base.cancel_goal()
				rospy.loginfo("Timed out achieving goal")
			else:
				state = self.move_base.get_state()
				if state == GoalStatus.SUCCEEDED:
					rospy.loginfo("Goal succeeded!")
				else:
					rospy.loginfo("Goal failed!")
			# Time spent running
			running_time = rospy.Time.now() - start_time
			running_time = running_time.secs / 60.0

			# Output all information for this navigation
			rospy.loginfo("current time:" + str(trunc(running_time,1))+"min")
        self.shutdown()

    def cmdCallback(self, msg):
        rospy.loginfo("Receive exploring cmd : %d", msg.data)
        self.exploring_cmd = msg.data

        if self.exploring_cmd is STATUS_CLOSE_TARGET:
            rospy.loginfo("Stopping the robot...")  
            self.move_base.cancel_goal() 

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        ExploringMaze()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring maze finished.")
