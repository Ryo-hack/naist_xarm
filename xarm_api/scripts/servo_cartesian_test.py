#!/usr/bin/env python

import sys
import time
import rospy
from xarm_msgs.srv import *

def servo_cartesian_motion(start_pose, freq, time_secs):
	servo_cart = rospy.ServiceProxy('/xarm/move_servo_cart', Move)
	req = MoveRequest() 
	req.pose = start_pose
	req.mvvelo = 0
	req.mvacc = 0
	req.mvtime = 0 
	loop_num = time_secs*float(freq)
	sleep_sec = 1.0/float(freq)
	ret = 0
	step = 0.3

	try:
		for i in range(int(loop_num)):
			req.pose[0] += step
			res = servo_cart(req)
			if res.ret:
				print("Something Wrong happened calling servo_cart service, index is %d, ret = %d"%(i, res.ret))
				ret = -1
				break
			time.sleep(sleep_sec)
		return ret

	except rospy.ServiceException as e:
		print("servo_cartesian Service call failed: %s"%e)
		return -1


if __name__ == "__main__":
	
	rospy.wait_for_service('/xarm/move_servo_cart')
	rospy.set_param('/xarm/wait_for_finish', True) # return after motion service finish
	
	motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
	set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
	set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	home = rospy.ServiceProxy('/xarm/go_home', Move)

	try:
		motion_en(8,1)
		set_mode(0)
		set_state(0)
		req = MoveRequest() # MoveRequest for go_home
		req.mvvelo = 0.7
		req.mvacc = 3.5
		req.mvtime = 0
		home(req)

	except rospy.ServiceException as e:
		print("Before servo_cartesian, service call failed: %s"%e)
		exit(-1)

	# configurations for servo_cartesian
	set_mode(1)
	set_state(0)

	# starting position for servo_cartesian in Base Coordinate
	start_pose = list(get_position().datas)
	
	# publish frequency in Hz:
	freq = 100 
	# publish time in seconds:
	time_secs = 5.0

	time.sleep(2.0)
	if servo_cartesian_motion(start_pose, freq, time_secs) == 0:
		print("execution finished successfully!")