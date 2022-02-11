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

class AdvancedRouting:
    def __init__(self):
        self.Ultrasonic = Ultrasonic()
        self.PWM = Motor()
        self.Servo = Servo()

    def run(self):
        print("Implement")

    def turn_right(self):
        print("Right")
        self.PWM.setMotorModel(-1450,-1450,1450,1450)

    def turn_left(self):
        print("Left")
        self.PWM.setMotorModel(1450,1450,-1450,-1450)

    def stop(self):
        print("Stop")
        self.PWM.setMotorModel(0,0,0,0)

    def forward(self):
        print("Forward")
        self.PWM.setMotorModel(-600,-600,-600,-600)

    def backward(self):
        print("Backward")
        self.PWM.setMotorModel(600,600,600,600)

    def get_distance_at(angle, self):
        global angle_distance
        self.pwm_S.setServoPwm('0',angle)
        distance = self.Ultrasonic.get_distance()
        angle_distance = [angle, distance]
        return int(distance), int(angle)

    def scan_step(ref, self):
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

    def scan(self):
        while True:
            scan_detection_list = scan_step(30)
            if not scan_detection_list or len(scan_detection_list) < NUM_SAMPLES - 1:
                continue
            break

        return scan_detection_list

    def draw_map(detection_list, size, self):
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

car = AdvancedRouting()

# Main program logic follows:
if __name__ == '__main__':
    print ('Starting up the Advanced Routing car ... ')
    try:
        origin = (99, 50)
        target = (0, 50)
        detection_list = car.scan()
        env_map = car.draw_map(detection_list, 100)
    except KeyboardInterrupt:
        car.PWM.setMotorModel(0,0,0,0)