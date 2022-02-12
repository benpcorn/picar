import detect
import time
from Motor import *

speed = 5

motor = Motor()

def execute_sequence(move_sequence):
    stop_detect_delay = 0
    skip_detect_detection = False

    for step in move_sequence:
        distance, changeDirection, direction = step

        if changeDirection == 0:

            while distance > 0:
                motor.setMotorModel(-1000,-1000,-1000,-1000)
                time.sleep(0.04)
                distance -= 1
            motor.setMotorModel(0,0,0,0)
            continue

        if changeDirection == 1:
            motor.setMotorModel(-1450,-1450,1450,1450)
            time.sleep(1.2)
            motor.setMotorModel(0,0,0,0)
            time.sleep(0.5)
            continue
        if changeDirection == 2:
            motor.setMotorModel(1450,1450,-1450,-1450)
            time.sleep(1.2)
            motor.setMotorModel(0,0,0,0)
            time.sleep(0.5)
            continue
    motor.setMotorModel(0,0,0,0)
