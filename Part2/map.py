import numpy as np
import math
import time

from constants import *
import mapping as mp

class Map:
    
    def __init__(self, x, y, start_position, orientation):
        self.map = np.zeros((x,y), dtype=int)
        self.current_position = start_position
        self.orientation = orientation
        self.height = x
        self.width = y
        self.scanning_map = False
        self.task_complete = False

    def insertMapSection(self, ultrasonicMap):
        ultrasonic_map_width = ultrasonicMap.shape[1]
        for i in range(ultrasonicMap.shape[0]):
            for j in range(ultrasonic_map_width):
                if (self.orientation == DOWN):
                    map_i_idx = self.current_position[0] + i
                    map_j_idx = self.current_position[1] + (j - math.floor(ultrasonic_map_width/2))
                elif (self.orientation == LEFT):
                    map_i_idx = self.current_position[0] + (j - math.floor(ultrasonic_map_width/2))
                    map_j_idx = self.current_position[1] - i
                elif (self.orientation == UP):
                    map_i_idx = self.current_position[0] - i
                    map_j_idx = self.current_position[1] - (j - math.floor(ultrasonic_map_width/2))
                elif (self.orientation == RIGHT):
                    map_i_idx = self.current_position[0] - (j - math.floor(ultrasonic_map_width/2))
                    map_j_idx = self.current_position[1] + i
                else:
                    print("ERROR: UNKNWON ORIENTATION IN MAP.PY:", self.orientation)
                    map_i_idx = 0
                    map_j_idx = 0
                if (self.isPointInBounds((map_j_idx, map_i_idx))):
                    self.setLabelAtPoint((map_i_idx, map_j_idx), ultrasonicMap[i][j])

    def printSelf(self):
        print(self.map)
        print(self.current_position)
        print(self.orientation)

    def getLabelAtPoint(self, point):
        return self.map[point[0]][point[1]]
    
    def setLabelAtPoint(self, point, label):
        self.map[point[0]][point[1]] = label

    def isPointInBounds(self, point):
        x, y = point
        if x < self.width and x >= 0 and y < self.height and y >= 0:
            return True
        return False

    def scanSurroundings(self):
        self.scanning_map = True

        map = mp.UltrasonicMap()
        relative_map = map.GetUltrasonicMapping()
        self.insertMapSection(relative_map)

        self.scanning_map = False

        time.sleep(10)
        m = mp.UltrasonicMap()
        m.print_map(self.map)
        