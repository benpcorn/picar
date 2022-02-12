import time
import argparse
import cv2
import os
import time

from Motor import *
from advanced_mapping import Mapping
from path_finding import Path_finding_A_star

motor = Motor()

def forward(power):
    motor.setMotorModel(-power, -power, -power, -power)

def backward(power):
    motor.setMotorModel(power, power, power, power)

def turn_left(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(power)
    right_rear.set_power(power)
    motor.setMotorModel(-power, -power, power, power)

def turn_right(power):
    motor.setMotorModel(power, power, -power, -power)

def stop():
    motor.setMotorModel(0,0,0,0)

def main_drive():
    # find directions of the route
    directions = get_directions()

    # Read frames
    while True:
        forward(1000)
        current_dir = directions[0]
        for dir in directions:
            drive_dir(current_dir, dir)

def get_directions():
    # find route based on surroundings
    clearance = 10
    starting = (51, clearance + 1)
    destination = (80, 80)
    map_width = 101
    mapping = Mapping(map_width, clearance)
    map_grid = mapping.scan()
    path_finding = Path_finding_A_star(map_grid, starting, destination, clearance)
    path = path_finding.search()

    directions = []
    for i in range(len(path) - 1):
        x = path[i][0] - path[i + 1][0]
        y = path[i][1] - path[i + 1][1]
        directions.append((x, y))

    return directions

def drive_dir(curr_dir, new_dir):
    if(curr_dir == new_dir):
        forward(1000)
    elif(new_dir == (0,1)):
        turn_right(1000)
        curr_dir = new_dir
    elif(new_dir == (1,0)):
        turn_left(1000)
        curr_dir = new_dir

if __name__ == "__main__":
    main_drive()