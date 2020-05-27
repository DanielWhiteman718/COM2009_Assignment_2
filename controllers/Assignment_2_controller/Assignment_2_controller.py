"""Assignment controller."""
from controller import Robot, DistanceSensor, Motor, Lidar
import math
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
        
        


def stopMotors():
    motorLeft.setVelocity(0)
    motorRight.setVelocity(0)
    
def turn_90(direction):
    motorRight.setPosition(1)
    motorLeft.setPosition(-1)

    #motorRight.setVelocity(-1)
    #motorLeft.setVelocity(1)



robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#setup motors
motorLeft = robot.getMotor('wheel_left')
motorRight = robot.getMotor('wheel_right')

motorLeft.setPosition(float('inf'))
motorRight.setPosition(float('inf'))
motorLeft.setVelocity(0.0)
motorRight.setVelocity(0.0)

#setup sensors
ds = robot.getLidar('lidar')
ds.enable(timestep)
cam = robot.getCamera('camera')
cam.enable(timestep)

cam_back = robot.getCamera('camera_back')
cam_back.enable(timestep)

start = True
mode = 'straight'
mode_start = robot.getTime()
start_time = robot.getTime()

mode = 'free'

angles = np.cos(np.deg2rad(np.linspace(0, 179, 180)))
base_color = [0, 0, 0]
# Main loop:

colours = [[128,0,0],[255,0,0],[128,128,0],[255,255,0],[0,128,0],[0,255,0],
        [0,128,128],[0,255,255],[0,0,128],[0,0,255],[128,0,128],[255,0,255]]

def Maze(start, mode, mode_start):
    while robot.step(timestep) != -1:
        if robot.getTime() - 0.5 < start_time:
            continue
        
        if start:
            #run motors
            motorLeft.setVelocity(10)
            motorRight.setVelocity(10)
            start = False
            
        # lidar data
        rangeImage = ds.getRangeImageArray()
        
        # lidar debuging
        #front = rangeImage[89][2]
        #left = rangeImage[0][2]
        #right = rangeImage[179][2]
        #front = "{:.2f}".format(front)
        #left = "{:.2f}".format(left)
        #right = "{:.2f}".format(right)
        #print(front + " | " + left + " | " + right + "      Front | Left | Right")
        
        lidar_0 = rangeImage[179][2]   # distance to right 
        lidar_30 = rangeImage[149][2]  # distance to right minus 30 degrees
        front = rangeImage[90][2]
        
        ratio = round(lidar_30 / lidar_0, 2)
        
        camImage = np.array(cam.getImageArray())
        print(camImage[32][32])
        colour = np.array(camImage[32][32])
        colourCatch = False
        
        
        if colour[0] == 128:
            if (colour[1] < 100) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] == 128) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] < 100) and (colour[2] == 128):
                colourCatch = True
        elif colour[0] == 255:
            if (colour[1] < 100) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] == 255) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] < 100) and (colour[2] == 255):
                colourCatch = True
        elif colour[0] < 100:
            if (colour[1] == 128) and (colour[2] < 100):
                colourCatch = True
            if (colour[1] == 255) and (colour[2] < 100):
                colourCatch = True
            if (colour[1] == 128) and (colour[2] == 128):
                colourCatch = True
            if (colour[1] == 255) and (colour[2] == 255):
                colourCatch = True
            if (colour[1] < 100) and (colour[2] == 128):
                colourCatch = True
            if (colour[1] < 100) and (colour[2] == 255):
                colourCatch = True
            
            
        

        
        """for i in range(len(colours)):
            if colours[i][0] == colour[0]:
                if colours[i][1] == colour[1]:
                    if colours[i][2] == colour[2]:
                        colourCatch = True"""
        if colourCatch:
            print("CAUGHT")
            break


        # turn left if wall dead ahead
        if front < 0.24 and mode != 'wall_ahead':
            mode = 'wall_ahead'    
            motorLeft.setVelocity(1.1)
            motorRight.setVelocity(10)
            continue
        
        # robot is too close to wall AND is going into the wall 
        if lidar_0 < 0.08 and mode != 'collision_avoid' and ratio <= 1.15:
            mode = 'collision_avoid'
            motorRight.setVelocity(10)
            motorLeft.setVelocity(3)
            continue
        
        # robot was too close to the wall and is now turning left, go straight once it's parallel
        if mode == 'collision_avoid' and ratio > 1.15:
            motorRight.setVelocity(10)
            motorLeft.setVelocity(10)
            mode = 'straight'
            continue
        
        # going parallel to the wall (lidar distances and wall make ~(30 60 90) triangle)
        if ratio >= 1.12 and ratio <= 1.17 and mode!= 'straight':
            mode = 'straight'   
            motorLeft.setVelocity(10)
            motorRight.setVelocity(10)
            continue
        
        # need to turn right (30* distance too short)
        if ratio > 1.18 and mode != 'wall_ahead' and mode != 'right':
            mode = 'right'
            motorLeft.setVelocity(10)
            motorRight.setVelocity(3)
            mode_start = robot.getTime()
            continue
        
        # need to turn left (30* distance too long)
        if ratio < 1.12 and mode != 'wall_ahead' and mode !='left' and robot.getTime() - 0.3 > mode_start:
            mode = 'left'
            motorLeft.setVelocity(3)
            motorRight.setVelocity(10)
            mode_start = robot.getTime()
            continue




def hug_beacon(rangeImage):
    if rangeImage[90][2] < 0.2:
        motorLeft.setVelocity(0)
        motorRight.setVelocity(0)

def match_color(rgb):
    distance = np.sum(np.square(rgb - base_color))
    distance = np.sqrt(distance)
    return distance < 100


def Beaconing(start, mode):
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

Maze(start, mode, mode_start)
Beaconing(start, mode)