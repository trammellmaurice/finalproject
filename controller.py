import rospy,time,tf
import numpy as np
from turtleAPI import robot
from Queue import PriorityQueue
from graph import *
import argparse

"""
ROBOT MOVEMENT STUFF
"""

#Functions for PID and djikstras
#calculate distance between goal and current
def distance(og_pos, cur_pos):
	distance = np.sqrt((cur_pos[0]-og_pos[0])**2 + (cur_pos[1]-og_pos[1])**2)
        if distance > 10:
          return 10
	return distance

def newyaw(og_pos, cur_pos):
	yaw = np.arctan2(og_pos[1] - cur_pos[1], og_pos[0] - cur_pos[0])
	return yaw

def yawdif(og_pos, cur_pos):
        yawdif = og_pos[2] - cur_pos[2]
	if yawdif > np.pi/2 or yawdif < -np.pi/2:
		yawdif = 1
	return yawdif

# initialize a graph
G = Graph()

# initialize the robot
r = robot()
cur_pos = r.getMCLPose() # get current robot position


# NEEDS GOAL FROM PATH ACTION PLANNER
#G.build_from_dot("map.dot",cur_pos[0],cur_pos[1],XGOAL,YGOAL)
