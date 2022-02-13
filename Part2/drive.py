import enum
import time

import map 
import mapping 
import constants 
import routing

from PCA9685 import PCA9685
from Ultrasonic import *
                     
us = Ultrasonic()             
motor = Motor()

class Drive:

    def __init__(self, map):
        self.map = map

    def emergency_stop(self):
        distance = us.get_distance() 
        print(distance)
        print("--")
        x1, y1 = self.map.current_position
        x2, y2 = self.map.current_position

        if self.map.orientation == constants.UP:
            x1 -= 1
            x2 -= 2
        if self.map.orientation == constants.RIGHT:
            y1 += 1
            y2 += 2
        if self.map.orientation == constants.DOWN:
            x1 += 1
            x2 += 2
        if self.map.orientation == constants.LEFT:
            y1 -= 1
            y2 -= 2

        while (distance < 20 and distance > 2) and ( self.map.isPointInBounds((x1, y1)) and self.map.map[x1][y1] == constants.FREE_SPACE) :    
            motor.setMotorModel(0,0,0,0)
            distance = us.get_distance() 


    def turning_dir(self, new_pos):
        dir = ''

        if (new_pos[0] == self.map.current_position[0] - 1) and (new_pos[1] == self.map.current_position[1]):
            dir = constants.UP
        elif (new_pos[0] == self.map.current_position[0]) and (new_pos[1] == self.map.current_position[1] - 1):
            dir = constants.LEFT
        elif (new_pos[0] == self.map.current_position[0]) and (new_pos[1] == self.map.current_position[1] + 1):
            dir = constants.RIGHT
        else:
            dir = constants.DOWN

        return dir

    def dir_from_val(val):
        if (val == 0):
            return constants.UP
        elif (val == 1):
            return constants.RIGHT
        elif (val == 2):
            return constants.DOWN
        elif (val == 3):
            return constants.LEFT 

    def update_pos(self):
        if (self.map.orientation == constants.UP):
            self.map.current_position = (self.map.current_position[0] - 1, self.map.current_position[1])
        elif (self.map.orientation == constants.RIGHT):
            self.map.current_position = (self.map.current_position[0], self.map.current_position[1] + 1)
        elif (self.map.orientation == constants.DOWN):
            self.map.current_position = (self.map.current_position[0] + 1, self.map.current_position[1])
        elif (self.map.orientation == constants.LEFT):
            self.map.current_position = (self.map.current_position[0], self.map.current_position[1] - 1)
  
    def turn(self, new_dir):
        dir_num = new_dir - self.map.orientation
        
        if (dir_num > 0):
            motor.setMotorModel(-2000,-2000,500,500) # turn right
            time.sleep(3)
            motor.setMotorModel(0,0,0,0)
            
        elif (dir_num < 0):
            motor.setMotorModel(500,500,-2000,-2000) # turn left
            time.sleep(3)
            motor.setMotorModel(0,0,0,0)

        self.map.orientation = new_dir   

    def translate(self): 
        
        motor.setMotorModel(-2000,-2000,-2000,-2000) # forward
        time.sleep(3)
        motor.setMotorModel(0,0,0,0)
        self.update_pos()

    def drive_step(self, new_pos):

        if (self.map.current_position == new_pos):
            return

        temp_orientation = self.turning_dir(new_pos)
        self.turn(temp_orientation)
        self.map.orientation = temp_orientation
        self.emergency_stop()
        self.translate()