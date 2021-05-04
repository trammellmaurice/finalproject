import numpy as np
import math

def dist(n1,n2):
    return math.sqrt((n1.x-n2.x)**2 +(n1.y-n2.y)**2)

def closest(n, N):
    X = []
    c = dist(n,N[0])
    cl = N[0].label
    for t in N:
        new_dist = dist(n,t)
        if new_dist < c:
            c = new_dist
            cl = t.label
    return cl

class Node:
    def __init__(self,label,x,y):
        self.label = label
        self.x = x
        self.y = y
        self.seen = False
        self.g = 0
        self.parent = None

    def __repr__(self):
        return self.label+' '+str(self.x)+' '+str(self.y)

class Graph:
    def __init__(self):
        self.vert = []
        self.adj_mtrx = []
        self.label_to_i = {}
        self.start = None
        self.goal = None

    def build_from_dot(self,filename,startx,starty,goalx,goaly):
        f = open(filename)
        f.readline()
        edges = set()
        for line in f:
            line1 = line.split('[')
            line2 = line.split('--')
            if len(line1)==2:
                name = line1[0].replace(' ','')
                points = line1[1].split('"')[1]
                points = points.split(',')
                x = float(points[0].replace('(',''))
                y = float(points[1].replace(')',''))
                self.label_to_i[name] = len(self.vert)
                self.vert.append(Node(name,x,y))
            if len(line2)==2:
                edges.add((line2[0].strip(),
                    line2[1].replace(';','').strip()))

        start = Node('start',startx,starty)
        goal = Node('goal',goalx,goaly)
        edges.add(('start',closest(start,self.vert)))
        edges.add(('goal',closest(goal,self.vert)))
        self.label_to_i['start'] = len(self.vert)
        self.vert.append(start)
        self.label_to_i['goal'] = len(self.vert)
        self.vert.append(goal)

        self.adj_mtrx = np.zeros((len(self.vert),len(self.vert)))-1
        for e in edges:
            i = self.label_to_i[e[0]]
            j = self.label_to_i[e[1]]
            d = dist(self.vert[i],self.vert[j])
            self.adj_mtrx[i][j] = d
            self.adj_mtrx[j][i] = d

        #print(self.vert)
        #print(self.adj_mtrx)

        f.close()

    def get_neighbors(self, vertex):
        nbrs = []
        row = self.adj_mtrx[self.label_to_i[vertex.label]]
        for i in range(len(row)):
            if row[i] != -1:
                nbrs.append((self.vert[i],row[i]))
        return nbrs

    def get_start(self):
        return self.vert[-2]

    def get_goal(self):
        return self.vert[-1]
