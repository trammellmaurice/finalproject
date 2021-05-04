import numpy as np
import json
import argparse
from queue import PriorityQueue

import rospy
import realDemo

import astar
from graph import *

from turtleAPI import robot

from pid import pidController
"""
ACTION PLANNING
**OUTPUT a LIST of actions []
"""

"""
PATHING
"""

"""
#INPUT a LIST of actions
#OUTPUT a LIST of vertices for each travel action
"""
def planPath(destination):
    print("planning path to ",destination)
    print()
    # Robot's current location
    current_position = R.getMCLPose()
    # current_position = (3.22306, -4.1509)
    # Make a graph with start and endpoint from dot file
    G = Graph()
    G.build_from_dot("map.dot",current_position[0],current_position[1],destination[0],destination[1])
    # Djikstras to get path to endpoint
    ### TODO FIX Dijkstras
    pos = G.get_start()
    q = PriorityQueue()
    pos.g = 0
    pos.seen = True
    while pos != G.get_goal():
        for next in G.get_neighbors(pos):
            #if not seen
            if next[0].seen == False:
                next[0].g = pos.g + next[1]
                q.put((next[0].g, next[0]))
                next[0].parent = pos
                next[0].seen = True
            else:
                if next[0].g > (pos.g + next[1]):
                    next[0].g = pos.g + next[1]
                    q.put((next[0].g, next[0]))
                    next[0].parent = pos
        pos = q.get()[1]

    path = []

    path.insert(0, G.get_goal())
    while pos.parent != None:
        path.insert(0, pos.parent)
        pos = pos.parent

    return path


"""
# INPUT a LIST of vertices
# OUTPUT True when done
"""
def travelPath(path):
    for vertex in path:
        # get vertex location
        location = (vertex.x,vertex.y)
        driveMCL(location)
    return True


"""
FIND BALOONS
"""

"""
INPUT color of balloon
OUTPUT None
"""
def balloonSearch(color):
    realDemo.hunt(color)
    return

"""
DRIVE ROBOT
"""

"""
DRIVE MCL
INPUT destination (x,y)
OUTPUT None
"""
def driveMCL(destination):
    # Robot current pose
    current_position = R.getMCLPose()
    # calculate target yaw
    angle = np.arctan2(destination[1] - current_position[1], destination[0] - current_position[0])
    # add angle to destination
    destination = (destination[0],destination[1],angle)
    # find distance to goal
    distance = calculateDistance(destination,current_position)

    # create PID controllers
    linear_controller = pidController(0.2,0,0.1)
    angular_controller = pidController(0.8)

    # find yaw to goal
    destination[2] = calculateAbsoluteYaw(destination,current_position)
    # find difference
    yaw = calculateYaw(destination,current_position)
    angular_controller.update(yaw)

    while yaw > 0.1 and not rospy.is_shutdown():
        # Update position
        current_position = R.getMCLPose()
        # find yaw to goal
        destination[2] = calculateAbsoluteYaw(destination,current_position)
        # find difference
        yaw = calculateYaw(destination,current_position)
        angular_controller.update(yaw)
        R.drive(angular_controller.pid(),0) # only turn
        rate.sleep()
    R.stop()

    while distance > 0.3 and not rospy.is_shutdown():
        # Update position
        current_position = R.getMCLPose()
        # find distance to goal
        distance = calculateDistance(destination,current_position)
        linear_controller.update(distance)
        # find yaw to goal
        destination[2] = calculateAbsoluteYaw(destination,current_position)
        # find difference
        yaw = calculateYaw(destination,current_position)
        angular_controller.update(yaw)
        # Drive
        R.drive(angular_controller.pid(),linear_controller.pid()) # only turn
        rate.sleep()
    R.stop()

"""
HELPER FUNCTIONS
"""
# calculate distance between goal and current
def calculateDistance(goal, current):
    distance = np.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    if distance > 10:
        return 10
    return distance

def calculateAbsoluteYaw(goal, current):
    yaw = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    return yaw

def calculateYaw(goal, current):
    yawdif = goal[2] - current[2]
    if yawdif > np.pi/2 or yawdif < -np.pi/2:
        yawdif = 1
    return yawdif

"""
MAIN
"""

rate = rospy.Rate(20)
R = robot()

# user interface
#PARSING AND CREATING ADJANCEY MATRIX
parser = argparse.ArgumentParser()
parser.add_argument("start_file_name")
parser.add_argument("end_file_name")
args = parser.parse_args()

# file objects
fstart = open(args.start_file_name,)
fend = open(args.end_file_name,)

#JSON to dictionary
data_start = json.load(fstart)
data_end = json.load(fend)

# printouts
print()
print("Start Dictionary")
print(data_start)
print("End Dictionary")
print(data_end)

# get list of actions
actions = astar.aStar((3.22306, -4.1509),data_start,data_end)

print()
print("Actions")
print(actions)
print()

for action in actions:
    # print(action)
    if action[0] == "move":
        print(action)
        print()
        path = planPath(action[1]) # get list of vertices to travel
        travelPath(path)
    elif action[0] == "pickup":
        balloonSearch(action[1])
        print(action)
    elif action[0] == "putdown":
        print(action)
