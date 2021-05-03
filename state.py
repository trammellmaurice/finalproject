import copy

def dist():
    return 0

class State:
    def __init__(self,loc,obj_loc):
        # the obj that is currently being carried by rob
        self.carrying = []
        # the robot's location
        self.robX = loc[0]
        self.robY = loc[1]
        # a list of tuples in form (obj, obj location)
        self.obj_loc = obj_loc
        self.depth = 0
        self.ancestor = None
        self.move = None

    def Drive(self, loc):
        if loc[0]==self.robX and loc[1]==self.robY:
            return None
        move = copy.deepcopy(self)
        move.robX = loc[0]
        move.robY = loc[1]
        return move

    def Pickup(self, obj):
        if len(self.carrying)==2:
            return None
        if obj in carrying:
            return None
        x = self.obj_loc[obj][0]
        y = self.obj_loc[obj][1]
        if self.robX!=x or self.robY!=y:
            return None

        return

    def Putdown(self):
        return

    def generateMoves(self,locs,balloons):
        return

    def set_parent(self, p):
        return
