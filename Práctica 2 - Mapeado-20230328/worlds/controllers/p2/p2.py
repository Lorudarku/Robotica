from controller import Robot, Motor, DistanceSensor
import os
import sys
os.environ["WEBOTS_HOME"] = "C:\\Program Files\\Webots"
os.environ["PYTHONPATH"] = "add ${WEBOTS_HOME}\\lib\\controller\\python"
os.environ["PYTHONIOENCODING"] = "UTF-8"

# Constants
CRUISE_SPEED = 25
MAX_SPEED = 47.6

robot = Robot()

map = [[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0]]
TIME_STEP = int(robot.getBasicTimeStep())

leftWheel = robot.getDevice("left wheel motor")
rightWheel = robot.getDevice("right wheel motor")

encoderL = robot.getDevice("left wheel sensor")
encoderL.enable(TIME_STEP)
encoderR = robot.getDevice("right wheel sensor")
encoderR.enable(TIME_STEP)

leftWheel.setPosition(0)
rightWheel.setPosition(0)
leftWheel.setVelocity(20)
rightWheel.setVelocity(20)

posL = encoderL.getValue()
posR = encoderR.getValue()

rightWheel.setVelocity(CRUISE_SPEED)

while(robot.step(TIME_STEP) != -1):
    leftWheel.setPosition(encoderL.getValue() + 250/21)
    rightWheel.setPosition(encoderL.getValue() + 250/21)
    print(posL)
    print(posR)
    print(map)
