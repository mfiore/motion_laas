#!/usr/bin/env python

import sys
import rospy
import actionlib
from motion_msgs.msg import * 
from common_msgs.msg import * 

def test_action(action,parameters):
	client=actionlib.SimpleActionClient('motion/motion_plan_execute',motion_msgs.msg.MotionPlanExecuteAction)	
	client.wait_for_server()
	goal=motion_msgs.msg.MotionPlanExecuteGoal(action,parameters)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

if __name__=="__main__":
	rospy.init_node("test_motion_py")
	action_name=sys.argv[1]
	parameters=[]
	i=2
	while i<len(sys.argv):
		parameter=common_msgs.msg.Parameter(sys.argv[i],sys.argv[i+1])
		parameters.append(parameter)
		i=i+2

	result=test_action(action_name,parameters)
	print result.report.status