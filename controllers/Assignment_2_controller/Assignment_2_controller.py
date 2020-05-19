"""Assignment controller."""
from controller import Robot, DistanceSensor, Motor, Lidar

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

#run motors
motorLeft.setVelocity(2)
motorRight.setVelocity(2)

# Main loop:
while robot.step(timestep) != -1:
    front = ds.getRangeImageArray()[255][2]
    left = ds.getRangeImageArray()[0][2]
    right = ds.getRangeImageArray()[511][2]
    
    front = "{:.2f}".format(front)
    left = "{:.2f}".format(left)
    right = "{:.2f}".format(right)
    
    print(front + " | " + left + " | " + right + "      Front | Left | Right")