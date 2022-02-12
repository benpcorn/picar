import time
import random
import math
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
from queue import Queue
from scipy.ndimage import binary_dilation

angle_distance = [0, 0]
max_angle = 160
min_angle = 0
current_angle = max_angle

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
    servo.setServoPwm('0',angle)
    distance = us.get_distance()
    return int(distance), int(angle) - 80

def scan():
    scan_list = []
    for i in np.linspace(start=max_angle, stop=min_angle, num=100):
        scan_list.append(get_distance_at(i))
        time.sleep(0.2)
    servo.setServoPwm('0',80)
    return scan_list

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

def BFS(env_map, start_node, target_node):
    target = target_node
    start = start_node
    visited = set()
    queue = Queue()
    path = []

    queue.put(target)
    visited.add(start)

    parent = dict()
    parent[start_node] = None

    path_found = False
    while not queue.empty():
        current_node = queue.get()
        if current_node == target_node:
            path_found = True
            break

        neighbors_set = []
        y, x = current_node

        if (y + 1) <= 99:
            if env_map[y + 1, x] == 0:
                neighbors_set.append((y + 1, x))
        if (y - 1) >= 0:
            if env_map[y - 1, x] == 0:
                neighbors_set.append((y - 1, x))
        if (x + 1) <= 99:
            if env_map[y, x + 1] == 0:
                neighbors_set.append((y, x + 1))
        if (x - 1) >= 0:
            if env_map[y, x - 1] == 0:
                neighbors_set.append((y, x - 1))

        for next_node in neighbors_set:
            if next_node not in visited:
                queue.put(next_node)
                parent[next_node] = current_node
                visited.add(next_node)

    if path_found:
        path.append(target_node)
        while parent.get(target_node) is not None:
            path.append(parent[target_node])
            print(target_node)
            print((parent))
            target_node = parent[target_node]
        path.reverse()

    return path

def moves_sequence(path, start_node, target_node):
    target = target_node
    start = start_node
    moveSequence = []
    step_counter = 0
    direction = 0
    last_step = 0

    for step in path:
        if step == start:
            direction = 0  # North
            change = 0  # No direction change
            last_step = step
            step_counter = 0
            continue
        if step == target:
            print("target node found")
            moveSequence.append((step_counter, 0, direction))
            if direction == 1:
                moveSequence.append((0, 2, 0))
            if direction == 2:
                moveSequence.append((0, 1, 0))
            continue

        y_current, x_current = step
        y_last, x_last = last_step

        if direction == 0 and x_current == x_last:
            step_counter += 1
            last_step = step
            continue
        if (direction == 1 or direction == 2) and y_current == y_last:
            step_counter += 1
            last_step = step
            continue
        if direction == 0 and x_current != x_last:
            moveSequence.append((step_counter, 0, direction))
            step_counter = 0
            if x_current > x_last:
                direction = 1
                moveSequence.append((step_counter, 1, direction))
                last_step = step
                continue
            else:
                direction = 2
                moveSequence.append((step_counter, 2, direction))
                last_step = step
                continue

        if (direction == 1 or direction == 2) and y_current != y_last:
            moveSequence.append((step_counter, 0, direction))
            step_counter = 0
            if direction == 1:
                direction = 0
                moveSequence.append((step_counter, 2, direction))
                last_step = step
                continue
            else:
                direction = 0
                moveSequence.append((step_counter, 1, direction))
                last_step = step
                continue

    item_num = len(moveSequence)
    step_counter = 0
    stop_index = 0

    if item_num > 2:
        for index in range(2, item_num):
            steps, change_direction, direction = moveSequence[index]
            if steps < 50:
                step_counter = step_counter + steps
                if change_direction == 0 and steps == 0:
                    step_counter = step_counter + 1
            if steps > 50:
                stop_index = index
                break
        steps_, change_, direction_ = moveSequence[2]
        moveSequence[2] = (step_counter, change_, direction_)
        del moveSequence[3:stop_index - 1]

    return moveSequence

def execute_sequence(move_sequence):
    stop_detect_delay = 0
    skip_detect_detection = False

    for step in move_sequence:
        distance, changeDirection, direction = step

        if changeDirection == 0:

            while distance > 0:
                current_time = time.time()
                forward()
                time.sleep(0.04)
                distance -= 1
            stop()
            continue

        if changeDirection == 1:
            turn_right()
            time.sleep(1.2)
            stop()
            time.sleep(0.5)
            continue
        if changeDirection == 2:
            turn_left()
            time.sleep(1.2)
            stop()
            time.sleep(0.5)
            continue
    stop()

origin = (99, 50)
target = (0, 50)

detection_list = scan()

env_map = draw_map(detection_list, 100)

print(env_map)

path = BFS(env_map, origin, target)
mov_sequence = moves_sequence(path, origin, target)

for element in path:
    env_map[element] = 3

plt.imshow(env_map)
plt.show()

execute_sequence(mov_sequence)

