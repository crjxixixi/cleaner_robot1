#!/usr/bin/env python
#coding=utf-8
'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  

def record_callback(msg):

    strp = msg.data
    
    print("收到：" +strp)
    
    str1 = "513"
    str2 = "电梯口"
    str3 = "514"
    str4 = "左边"
    if  strp.find(str1)!=-1 or strp.find(str2)!=-1 or strp.find(str3)!=-1 or strp.find(str4)!=-1 :
	if strp.find(str1)!=-1 :
	    print(str1)
	    Name = str1
	if strp.find(str2)!=-1 :
	    print(str2)
	    Name = str2
	if strp.find(str3)!=-1 :
	    print(str3)
	    Name = str3
	if strp.find(str4)!=-1 :
	    print(str4)
	    Name = str4
    else :
	print("此句话中没有找到相关位置信息")
    position_name = []
    position_x = []
    position_y = []
    position_z = []

    with open("/home/xt/turbot_ws/position.txt",'r')as f:
	lines = f.readlines()
	for line in lines:
            item = line.strip().split(' ')
	    position_name.append(item[0])
	    position_x.append(item[1])
            position_y.append(item[2])
            position_z.append(item[3])
    if (position_name[0]==Name):
	position = {'x': float(position_x[0]), 'y' : float(position_y[0])}	
    if (position_name[1]==Name):
	position = {'x': float(position_x[1]), 'y' : float(position_y[1])}
    if (position_name[2]==Name):
	position = {'x': float(position_x[2]), 'y' : float(position_y[2])}
    if (position_name[3]==Name):
	position = {'x': float(position_x[3]), 'y' : float(position_y[3])}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    success = navigator.goto(position, quaternion)
    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base failed to reach the desired pose")


  

class GoToPose():
    def __init__(self):

        self.goal_sent = False
	self.last_location = Pose()  
	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True)
	        # Get the initial pose from the user  
        # 获取初始位置(仿真中可以不需要)  
	initial_pose = PoseWithCovarianceStamped()  
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)  
          
        # Make sure we have the initial pose  
        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  
              
        rospy.loginfo("Starting navigation test")  

        navigator = GoToPose()
	rospy.Subscriber("/voiceWords", String, record_callback)
	rospy.spin()

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

