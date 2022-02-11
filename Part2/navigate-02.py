import time
import random
from Motor import *
from Ultrasonic import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685
from matplotlib import pyplot as plt
import matplotlib
import time
import threading
import numpy as np

speed = 5
NUM_SAMPLES = 100
ANGLE_RANGE = 160
STEP = ANGLE_RANGE / NUM_SAMPLES
us_step = STEP
angle_distance = [0, 0]
max_angle = 160
min_angle = 0
current_angle = max_angle
scan_list = []

us = Ultrasonic()
motor = Motor()
servo = Servo()

def turn_right():
    print("Right")
    motor.setMotorModel(-1450,-1450,1450,1450)

def turn_left():
    print("Left")
    motor.setMotorModel(1450,1450,-1450,-1450)

def stop():
    print("Stop")
    motor.setMotorModel(0,0,0,0)

def forward():
    print("Forward")
    motor.setMotorModel(-600,-600,-600,-600)

def backward():
    print("Backward")
    motor.setMotorModel(600,600,600,600)

def get_distance_at(angle):
    global angle_distance
    servo.setServoPwm('0',angle)
    distance = us.get_distance()
    angle_distance = [angle, distance]
    return int(distance), int(angle)

def scan():
    for i in range (min_angle, max_angle, 5):
        scan_list.append(get_distance_at(i))
        time.sleep(0.2)
    servo.setServoPwm('0',0)

def draw_map(scan_list, size):
    env_map = np.zeros([size, size])
    mask = np.ones([10, 7])
    point_y, point_x = scan_list[0]
    for sample in scan_list:
        distance, angle = sample
        if distance > 0:
            x = math.floor((math.sin(math.radians(-angle))) * distance) + 50
            y = math.floor((math.cos(math.radians(angle))) * distance)
            if 100 > x > 0 and y < 100 and (abs(point_y - y) <= 1) \
                    and (abs(point_x - x) <= 1):
                env_map[x, y] = 1        
            point_x = x
            point_y = y
            
    env_map = binary_dilation(env_map, mask).astype(env_map.dtype)
    return np.rot90(env_map)

scan()
print(scan_list)
draw_map(scan_list, 100)