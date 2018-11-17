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
	model_state.pose.position.x = 10
	model_state.pose.position.y = 6
	model_state.pose.position.z = 5
	model_state.pose.orientation.x = 0
	model_state.pose.orientation.y = 1
	model_state.pose.orientation.z = 2
	model_state.pose.orientation.w = 3
	#print('hello')
	model_state.reference_frame=""
	w = Twist()
	model_state.twist = w
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(model_state)
		rate.sleep()

move_model("piano2",[10,6,5],[0,0,0,0])

