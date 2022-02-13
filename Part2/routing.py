import heapq
import numpy as np
import constants 

MAZE = None
orientation = None
old_pos = None

class Node:
    def __init__(self, pos, parent, p_cost = 0, g_h = 0, manhattan = 0):
        self.pos = pos
        self.parent = parent
        self.p_cost = p_cost
        self.g_h = g_h
        self.manhattan = manhattan

    def __eq__(self, other):
        return (self.g_h == other.g_h) 

    def __ne__(self, other):
        return not (self.g_h == other.g_h)

    def __lt__(self, other):
        return (self.g_h < other.g_h)

    def __gt__(self, other):
        return (self.g_h > other.g_h) 

    def __le__(self, other):
        return (self.g_h < other.g_h) or (self.g_h == other.g_h)

    def __ge__(self, other):
        return (self.g_h > other.g_h) or (self.g_h == other.g_h)


def turn_dir(pos, new_pos):
    global orientation

    if (pos == new_pos):
        return orientation
    
    if (new_pos[0] == pos[0] - 1) and (new_pos[1] == pos[1]):
        dir = constants.UP
    elif (new_pos[0] == pos[0]) and (new_pos[1] == pos[1] - 1):
        dir = constants.LEFT
    elif (new_pos[0] == pos[0]) and (new_pos[1] == pos[1] + 1):
        dir = constants.RIGHT
    else:
        dir = constants.DOWN

    return dir


def turn_dir_weight(new_dir, man_dist):
    if (new_dir == orientation) :
        return - man_dist * 2 
    else :
        return  man_dist * 2


def manhattan_dist(pos, end):
    dist = (pos[0] - end[0])**2 + (pos[1] - end[1])**2
    return dist

def node_with_lowest_man_dist(visited):
    node = None
    lowest_g_h = float('inf')

    for key in visited: 
        if (visited[key].manhattan < lowest_g_h and visited[key].manhattan > 0) :
            node = visited[key]
            lowest_g_h = visited[key].manhattan

    return node

def get_neighbors(pos):
    global MAZE
    neighbors = []
    rowNbr = [-1,0,0,1]
    colNbr = [0,-1,1,0]

    for i in range(4):
        r = pos[0] + rowNbr[i]
        c = pos[1] + colNbr[i]

        if (r < MAZE.shape[0] and r >= 0):
            if (c < MAZE.shape[1] and c >= 0):
                if MAZE[r][c] == constants.FREE_SPACE:
                    neighbors.append((r, c))
    
    return neighbors

def astar(map, start_pos = (0, 0), end_pos = (30, 30)):
    print(start_pos)
    print(end_pos)
    global MAZE, orientation, old_pos
    path = []
    state = Node( start_pos, None, 0, 0, 0)
    list = []
    reached_goal = False
    
    old_pos = start_pos
    orientation = constants.RIGHT
    MAZE = map
    heapq.heappush(list, (state.g_h, state))
    visited = {state.pos : state}

    while (list):
        
        state = heapq.heappop(list)[1]
        orientation = turn_dir(old_pos, state.pos)
        old_pos = state.pos
        
        if (state.pos == end_pos):
            reached_goal = True
            break
        
        neighbors = get_neighbors(state.pos)
    
        for neighbor in neighbors:
                if (neighbor not in visited or visited[neighbor].p_cost > state.p_cost + 1 ):
                    neigh_dir = turn_dir(state.pos, neighbor)
                    man_dist = manhattan_dist(neighbor, end_pos)
                    dir_weight = turn_dir_weight(neigh_dir, man_dist)

                    neigh_state = Node(neighbor, state.pos, state.p_cost + 1, state.p_cost + 1 + man_dist + dir_weight , manhattan_dist(neighbor, end_pos))
                    heapq.heappush(list, (neigh_state.g_h, neigh_state))
                    visited[neigh_state.pos] =  neigh_state
    
    parent = None

    if reached_goal :
        path.insert(0, state.pos)
        parent = state.parent
    else :
        parent = node_with_lowest_man_dist(visited).pos
    
    while (parent != None):
        path.insert(0, visited[parent].pos)
        parent = visited[parent].parent

    return path
    