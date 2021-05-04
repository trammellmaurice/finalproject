import numpy as np
import rospy

from turtleAPI import robot

from pid import pidController
"""
ACTION PLANNING
**OUTPUT a LIST of actions []
"""

"""
"move 1,1"
"""

"""
PATHING
"""

    """
    #INPUT a LIST of actions
    #OUTPUT a LIST of vertices for each travel action
    """
def planPath(destination):
    # Robot's current location
    # Make a graph with start and endpoint from dot file
    # Djikstras to get path to endpoint

    """
    # INPUT a LIST of vertices
    # OUTPUT True when done
    """
def travelPath(path):
    # for vertex in path:
        # Robot current position
        # get vertex location
        #calculate yaw and distance
        # while not at vertex and not rospy.is_shutdown():
            # driveMCL(vertex)
    return True


"""
FIND BALOONS

"""

    """
    INPUT color of balloon
    OUTPUT None
    """
def balloonSearch(color):
    # TODO Balloon color search and depth sense
    pass

"""
DRIVE ROBOT
"""

"""
DRIVE MCL
INPUT destination (x,y)
OUTPUT True
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
    linear_controller = pidController(0.5,0,0.5)
    angular_controller = pidController(0.75)

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

# for action in ACTION_LIST:
    # if action contains "move":
        # path = planPath(destination) # get list of vertices to travel
        # travelPath(path)
    # elif action contains "pickup":
        # balloonSearch(color)
