from state import *
from PriQue import *
import copy
import math

def dist(t1, t2):
    return math.sqrt((t1[0]-t2[0])**2
            +(t1[1]-t2[1])**2)

def h(curr,goal):
    total = 0
    for i in curr.obj_loc.keys():
        n1 = curr.obj_loc[i]
        n2 = goal.obj_loc[i]
        if n1 == None:
            n1 = (curr.robX,curr.robY)
        total += dist(n1,n2)
    return total/2

def aStar(rob_pos,curr_dict,goal_dict):
    for k in curr_dict:
        curr_dict[k] = tuple(curr_dict[k])
    for k in goal_dict:
        goal_dict[k] = tuple(goal_dict[k])
    brd = State(rob_pos,curr_dict)
    goal = State(rob_pos,goal_dict)
    locs = curr_dict.values()
    objs = goal_dict.keys()
    Q = PriQue()
    closed = set()
    #nodes_expanded = 0

    while brd != goal:
        if not brd in closed:
            #nodes_expanded += 1
            moves = brd.generateMoves(locs,objs)
            #print(brd)
            """successors = []
            for i in moves:
                t = copy.deepcopy(brd)
                t.makeMove(i)
                if not search(brd,t):
                    t.move = i
                    successors.append(t)"""
            closed.add(brd)
            for s in moves:
                #print(s)
                f_cost = brd.depth+1+h(s,goal)
                s.set_parent(brd)
                Q.push(f_cost,s)
            #print()
        """for i in Q.heap:
            print(i)
        print()"""
        brd = Q.pop()

    return get_path(brd)

def main():
    balloons = {'yellow':[-1,-1],
                'green':[4,-1],
                'red':[1,1],
                'blue':[5,5],
                'pink':[10,10]}
    balloons_g = {'yellow':[4,-1],
                'green':[1,1],
                'red':[5,5],
                'blue':[10,10],
                'pink':[-1,-1]}
    m = aStar((0,0),balloons,balloons_g)
    print(m)
    return

if __name__=="__main__":
    main()
