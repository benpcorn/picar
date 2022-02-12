import time
import random
from Motor import *
from Ultrasonic import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685

class SelfDrivingElon:
    def __init__(self):
        self.Ultrasonic = Ultrasonic()
        self.PWM = Motor()

    def run(self):
        while True:
            dist = self.Ultrasonic.get_distance()
            if dist <= 10:
                self.stop()
                self.backward()
                time.sleep(1)
                self.stop()
                direction = random.randint(0,1)

                if direction == 0:
                    self.turn_left()
                    time.sleep(1.5)
                    self.stop()
                else:
                    self.turn_right()
                    time.sleep(1.5)
                    self.stop()
            else:
                self.forward()

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

        
dummy = SelfDrivingElon()

# Main program logic follows:
if __name__ == '__main__':
    print ('Starting up the self driving Elon car ... ')
    try:
        threshold = 10
        start_position = (51, threshold + 1)
        destination = (100,100)
        space_width = 110
        mapping = Mapping(space_width, threshold)
        map_grid = mapping.scan()
        
        path_finding = Path_finding_A_star(map_grid, start_position, destination, threshold)
        path = path_finding.search()

        

        dummy.run()
    except KeyboardInterrupt:
        dummy.PWM.setMotorModel(0,0,0,0)