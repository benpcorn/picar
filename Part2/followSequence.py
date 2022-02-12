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
                current_time = time.time()
                if (current_time - stop_detect_delay) < 3:
                    skip_stop_detection = True
                else:
                    skip_stop_detection = False

                if detect.person_detected:
                    motor.setMotorModel(0,0,0,0)
                    print("Person detected, waiting ....")
                    time.sleep(2.5)
                    continue
                if detect.stop_sign_detected and not skip_stop_detection:
                    motor.setMotorModel(0,0,0,0)
                    print("Stop sign detected, stopping for 8 seconds...")
                    time.sleep(8)
                    stop_detect_delay = time.time()
                    continue
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
