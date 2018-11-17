#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 19:44:25 2018

@author: lawrence
"""

import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist


rospy.init_node('hello')

def move_model(model_name, pose, quaternion):
	pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
	
	model_state = ModelState()

	model_state.model_name='piano2'
	model_state.pose.position.x = pose[0]
	model_state.pose.position.y = pose[1]
	model_state.pose.position.z = pose[2]
	model_state.pose.orientation.x = quaternion[0]
	model_state.pose.orientation.y = quaternion[1]
	model_state.pose.orientation.z = quaternion[2]
	model_state.pose.orientation.w = quaternion[3]
	#print('hello')
	model_state.reference_frame=""
	w = Twist()
	model_state.twist = w
	rate = rospy.Rate(10)

	for i in range(11):
		pub.publish(model_state)
		rate.sleep()

def main():
	move_model("piano2",[10,6,5],[0,0,0,0])

if __name__ == "__main__":
	main()

