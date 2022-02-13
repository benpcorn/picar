import math
import numpy as np
from constants import *
from Ultrasonic import *
from servo import *
from PCA9685 import PCA9685
import RPi.GPIO as GPIO

us = Ultrasonic()
servo = Servo()

class UltrasonicMap:
    ANGLE_RANGE = 160
    STEP = 10
    us_step = STEP
    current_angle = 0
    max_angle = 160
    min_angle = 0
    prev_point = (0,0)
    map_height = 100
    map_width = 200
    main_map = np.full([map_height,map_width], UNKNOWN_SPACE, dtype=int)

    def plotLine(self, x0, y0, x1, y1, type=UNCLASSIFIED_OBJECT):
        hit_object = False
        dx =  abs(x1-x0)
        sx = 1 if x0<x1 else -1
        dy = -abs(y1-y0)
        sy = 1 if y0<y1 else -1
        err = dx+dy
        while (True):
            if (self.main_map[x0][y0] == UNCLASSIFIED_OBJECT):
                hit_object = True
            if ((x0-1) >= 0 and self.main_map[x0-1][y0] == UNCLASSIFIED_OBJECT) or ((y0-1) >= 0 and self.main_map[x0][y0-1] == UNCLASSIFIED_OBJECT or ((x0+1) < self.map_height and self.main_map[x0+1][y0] == UNCLASSIFIED_OBJECT) or ((y0+1) < self.map_width and self.main_map[x0][y0+1] == UNCLASSIFIED_OBJECT)):
                hit_object = True

            elif (type == UNCLASSIFIED_OBJECT or not hit_object):
                self.main_map[x0][y0] = type
            if (x0 == x1 and y0 == y1):
                break
            e2 = 2*err
            if (e2 >= dy):
                err += dy
                x0 += sx
            if (e2 <= dx):
                err += dx
                y0 += sy

    '''
    Performs ultrasonic scanning for one angle and updates the main map accordingly
    '''
    def scan_step_return_distances(self):
        # set the current angle
        self.current_angle += self.us_step
        if self.current_angle >= self.max_angle:
            self.current_angle = self.max_angle
            self.us_step = -self.STEP
        elif self.current_angle <= self.min_angle:
            self.current_angle = self.min_angle
            self.us_step = self.STEP
        
        # find the distance to the object and the x/y components
        servo.setServoPwm('0',self.current_angle)
        distance = us.get_distance()
        y_idx = round((math.cos(math.radians((self.current_angle)))*distance))
        x_idx = round((math.sin(math.radians((self.current_angle)))*distance))
        x_idx = x_idx + math.floor(self.map_width/2) # shift horizontal distances by 1/2 the size of the grid for indexing

        # save the objects coordinates and their distances/angles (reject ones that are far away or not detected)
        if distance != -2 and y_idx < self.map_height and x_idx >= 0 and x_idx < self.map_width:
            self.main_map[y_idx][x_idx] = UNCLASSIFIED_OBJECT
            
            # if the object we got is near another one we just scanned, draw a line between them to consider them one object
            if (self.prev_point[0] != 0 and self.prev_point[1] != 0):
                if (((self.prev_point[0] - y_idx)**2) + ((self.prev_point[1] - x_idx)**2) < 30**2):
                    # print("drawing line from ", prev_point, "to ", (y_idx,x_idx))
                    self.plotLine(self.prev_point[0], self.prev_point[1], y_idx, x_idx)
            self.prev_point = (y_idx,x_idx)
        else:
            self.prev_point = (0,0)

    def mark_free_space(self):
        for i in range(self.map_height):
            self.plotLine(0, int(self.map_width/2), i, 0, type=FREE_SPACE)
            self.plotLine(0, int(self.map_width/2), i, self.map_width-1, type=FREE_SPACE)
        for i in range(self.map_width):
            self.plotLine(0, int(self.map_width/2), self.map_height-1, i, type=FREE_SPACE)

    '''
    The condensed map stores one element for every 5x5 square in the main map.
    - If the corresponding 5x5 grid contains any objects, the element is marked as an 
        object, regardless of the other values in the 5x5 square.
    - If the corresponding 5x5 grid contains free space and no objects, the element is marked as free
    - If all 25 elements in the 5x5 grid are unknown space, the element is marked as unknown space
    '''
    @staticmethod
    def condense_map(map, scale_factor=GRANULARITY):
        condensed_height = math.floor(map.shape[0]/scale_factor)
        condensed_width = math.floor(map.shape[1]/scale_factor)
        condensed_map = np.full([condensed_height, condensed_width], UNKNOWN_SPACE, dtype=int)

        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                condensed_i = math.floor(i/scale_factor)
                condensed_j = math.floor(j/scale_factor)
                if (map[i][j] == UNCLASSIFIED_OBJECT):
                    condensed_map[condensed_i][condensed_j] = UNCLASSIFIED_OBJECT
                elif (map[i][j] == FREE_SPACE and condensed_map[condensed_i][condensed_j] != UNCLASSIFIED_OBJECT):
                    condensed_map[condensed_i][condensed_j] = FREE_SPACE
        return condensed_map

    @staticmethod
    def print_map(map):
        print('\n\n', end='')
        map_height = map.shape[0]
        map_width = map.shape[1]
        for row in range(map_height):
            for col in range(map_width):
                print(map[row][col], end=', ')
            print('\n', end='')

    def GetUltrasonicMapping(self):
        # clear the map
        self.main_map = np.full([self.map_height,self.map_width], UNKNOWN_SPACE, dtype=int)

        # scan a few times and fill the main map with objects
        for i in range(64):
            self.scan_step_return_distances()

        # mark free spaces
        self.mark_free_space()
        
        # fill the condensed version of the map using the main map
        condensed_map = self.condense_map(self.main_map)

        self.print_map(condensed_map)

        # face forward again and print the condensed map
        servo.setServoPwm('0', 0)

        return condensed_map
            
if __name__ == "__main__":
    scanner = UltrasonicMap()
    scanner.GetUltrasonicMapping()

        
