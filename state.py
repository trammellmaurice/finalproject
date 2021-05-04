import copy

def get_path(B):
    moves = []
    while B.ancestor != None:
        moves.insert(0,(B.action,B.noun))
        B = B.ancestor
    return moves

class State:
    def __init__(self,loc,obj_loc):
        # the obj that is currently being carried by rob
        self.carrying = []
        # the robot's location
        self.robX = loc[0]
        self.robY = loc[1]
        # a dictionary that maps balloons to locations
        self.obj_loc = obj_loc
        self.depth = 0
        self.ancestor = None
        self.action = None
        self.noun = None

    def __repr__(self):
        return (str(self.robX)+' '+str(self.robY)+' '+
                str(self.carrying)+' '+str(self.obj_loc))

    def __hash__(self):
        return hash(repr(self))

    def __eq__(self, other):
        if isinstance(other,State):
            return self.obj_loc == other.obj_loc

    def Drive(self, loc):
        if loc[0]==self.robX and loc[1]==self.robY:
            return None

        move = copy.deepcopy(self)
        move.robX = loc[0]
        move.robY = loc[1]
        move.action = 'move'
        move.noun = (loc[0],loc[1])
        return move

    def Pickup(self, obj):
        if len(self.carrying)==2:
            return None
        if obj in self.carrying:
            return None
        x = self.obj_loc[obj][0]
        y = self.obj_loc[obj][1]
        if self.robX!=x or self.robY!=y:
            return None

        move = copy.deepcopy(self)
        move.carrying.append(obj)
        move.obj_loc[obj] = None
        move.action = 'pickup'
        move.noun = obj
        return move

    def Putdown(self, obj, locs):
        if not obj in self.carrying:
            return None
        curr_loc = (self.robX,self.robY)
        if not curr_loc in locs:
            return None
        
        move = copy.deepcopy(self)
        move.carrying.remove(obj)
        move.obj_loc[obj] = curr_loc
        move.action = 'putdown'
        move.noun = obj
        return move

    def generateMoves(self,locs,balloons):
        new_states = []
        for l in locs:
            s = self.Drive(l)
            if s != None:
                new_states.append(s)
        for b in balloons:
            s = self.Pickup(b)
            if s != None:
                new_states.append(s)
        for b in balloons:
            s = self.Putdown(b,locs)
            if s != None:
                new_states.append(s)

        return new_states

    def set_parent(self, p):
        self.ancestor = p
        self.depth = p.depth+1
        return
