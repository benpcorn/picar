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


scan()
print(scan_list)