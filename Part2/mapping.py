from numpy import ndarray
import time
import random
import numpy as np
import math
from scipy.ndimage import binary_dilation
import get_path
from Motor import *
from Ultrasonic import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685
from matplotlib import pyplot as plt

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

ultrasonic = Ultrasonic()
us_servo = Servo()


def scan_step(ref):
    global scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP

    status = get_distance_at(current_angle)
    scan_list.append(status)

    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            scan_list.reverse()
        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False


def get_status_at(angle, ref1=35, ref2=10):
    dist = get_distance_at(angle)
    if dist > ref1 or dist == -2:
        return 0
    elif dist > ref2:
        return 1
    else:
        return 1


def get_distance_at(angle):
    global angle_distance
    us_servo.setServoPwm('0',angle)
    time.sleep(0.04)
    distance = ultrasonic.get_distance()
    angle_distance = [angle, distance]
    return int(distance), int(angle)


def scan():
    while True:
        scan_detection_list = scan_step(30)
        if not scan_detection_list or len(scan_detection_list) < NUM_SAMPLES - 1:
            continue
        break

    return scan_detection_list


def draw_map(detection_list, size):
    env_map = np.zeros([size, size])
    mask = np.ones([10, 7])
    point_y, point_x = detection_list[0]
    for sample in detection_list:
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
