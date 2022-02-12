from Motor import *

motor = Motor()

# Turn Right
motor.setMotorModel(-2000,-2000,1500,1500) 
time.sleep(1)
motor.setMotorModel(0,0,0,0)
time.sleep(1)

# Turn Left
motor.setMotorModel(1500,1500,-2000,-2000) 
time.sleep(1)
motor.setMotorModel(0,0,0,0)
