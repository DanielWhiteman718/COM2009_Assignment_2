"""free_wander controller."""
from controller import Robot, DistanceSensor, Motor, Lidar
import numpy as np

# calculates the force for turn from distances in front
def wanderer(rangeImage):
    ranges = np.array(rangeImage)[:, 2] * -1
    force = np.sum(angles * ranges * 10)
    
    if force > 120:
        motorLeft.setVelocity(10)
        motorRight.setVelocity(2)
        
    # turn left
    elif force < -120:
        motorLeft.setVelocity(3)
        motorRight.setVelocity(10)
        
    # go straight
    else:
        motorLeft.setVelocity(10)
        motorRight.setVelocity(10)
        
        
def match_color(rgb):
    distance = np.sum(np.square(rgb - base_color))
    distance = np.sqrt(distance)
    return distance < 100

                 

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# sensors and motors
motorLeft = robot.getMotor('wheel_left')
motorRight = robot.getMotor('wheel_right')

motorLeft.setPosition(float('inf'))
motorRight.setPosition(float('inf'))
motorLeft.setVelocity(0.0)
motorRight.setVelocity(0.0)

ds = robot.getLidar('lidar')
ds.enable(timestep)
cam = robot.getCamera('camera')
cam.enable(timestep)

cam_back = robot.getCamera('camera_back')
cam_back.enable(timestep)

# program start time
start = True
start_time = robot.getTime()

# cosine values for wanderer
angles = np.cos(np.deg2rad(np.linspace(0, 179, 180)))
base_color = [0, 0, 0]

mode = 'free'
def hug_beacon(rangeImage):
    if rangeImage[90][2] < 0.2:
        motorLeft.setVelocity(0)
        motorRight.setVelocity(0)



while robot.step(timestep) != -1:
    if robot.getTime() - 1 < start_time:
        continue
           
    if start:
        base_color = np.array(cam_back.getImageArray())[32][32]
        start = False
        
    # lidar data
    rangeImage = ds.getRangeImageArray()
    # camera data
    camImage = np.array(cam.getImageArray())
    
    if match_color(camImage[32][32]):
        motorLeft.setVelocity(10)
        motorRight.setVelocity(10)
        mode = 'hug'
    
    if mode == 'hug':
        hug_beacon(rangeImage)
    elif mode == 'free':
        wanderer(rangeImage)
        
    pass
