import rospy,time,tf
import numpy as np
from turtleAPI import robot
from Queue import PriorityQueue
from graph import *
import argparse


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

#PARSING AND CREATING ADJANCEY MATRIX
parser = argparse.ArgumentParser()
parser.add_argument("filename")
parser.add_argument("x_goal",type=float)
parser.add_argument("y_goal",type=float)
parser.add_argument("--x_start",type=float)
parser.add_argument("--y_start",type=float)
args = parser.parse_args()

G = Graph()
r = None
if args.x_start == None:
  r = robot()
  cur_pos = r.getMCLPose()
  G.build_from_dot(args.filename,cur_pos[0],cur_pos[1],args.x_goal,args.y_goal)
else:
  G.build_from_dot(args.filename,args.x_start,args.y_start,args.x_goal,args.y_goal)

#DJIKSTRAS to get the path through graph, path can be represented by a series of X,Y
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

print path

if args.x_start != None:
  quit()
  

#GET X GOAL AND Y GOAL for each part of path 
for vert in path:
  xg = vert.x
  yg = vert.y
  #Get where the robot is
  cur_pos = r.getMCLPose()
  #calculate target yaw
  ang = np.arctan2(yg - cur_pos[1], xg - cur_pos[0])
  og_pos = [xg, yg, ang]
  #initialization
  dist = distance(og_pos, cur_pos)
  ictr = 0 
  integD = [0,0,0,0,0]
  prevD = 0
  integY = [0,0,0,0,0]
  prevY = 0
  #while we are not within 10 centimeters of target go to the next X, Y
  while dist > .3 and not rospy.is_shutdown():
    cur_pos = r.getMCLPose()
    #PID for linear speed
    dist = distance(og_pos, cur_pos)
    plin = .1*dist
    #add distance to integral buffer
    integD[ictr%5] = dist
    ilin = .01*(integD[0] + integD[1] + integD[2] + integD[3] + integD[4])
    #derivative part
    dervD = 0*(dist - prevD)
    #set previous distance to next distance
    prevD = dist
    #add up components
    linSpeed = plin + ilin + dervD

    #PID for angular speed
    og_pos[2] = newyaw(og_pos, cur_pos)
    ya = yawdif(og_pos, cur_pos)
    pang = .8*ya
    #add yawdif to integral
    integY[ictr%5] = ya
    iang = .01*(integY[0] + integY[1] + integY[2] + integY[3] + integY[4])
    #derviative piece
    dervY = 0*(ya-prevY)
    #set prevY to the next one
    prevY = ya
    #add up compnents
    angSpeed = pang + iang + dervY
 
    r.drive(angSpeed, linSpeed)
    ictr+=1
    print cur_pos
    time.sleep(.25)

#stop bot
r.stop()


