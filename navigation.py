from graph import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("filename")
parser.add_argument("x_goal",type=float)
parser.add_argument("y_goal",type=float)
parser.add_argument("--x_start",type=float)
parser.add_argument("--y_start",type=float)
args = parser.parse_args()

G = Graph()
G.build_from_dot(args.filename,2,3,args.x_goal,args.y_goal)

print(G.get_neighbors(G.get_start()))
