# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 19:44:25 2018

@author: lawrence
"""

from gazebo_msgs.msg import GazeboState
from gazebo_msgs.msg import gazebo.srv

pub = rospy.Publisher('/gazebo/set_model_state', model_state, queue_size=10)

def move_model(model_name, pose, quaternion):
	model_state = GazeboState()
	model_state.position.x = pose[0]
	model_state.position.y = pose[1]
	model_state.position.z = pose[2]

	model_state.orientation.x = quaternion[0]
	model_state.orientation.x = quaternion[1]
	model_state.orientation.x = quaternion[2]
	model_state.orientation.x = quaternion[3]

	pub.publish(model_state)