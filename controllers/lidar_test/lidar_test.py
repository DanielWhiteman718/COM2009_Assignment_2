"""lidar_test controller."""

from controller import Robot, DistanceSensor, Motor, Lidar

robot = Robot()


timestep = int(robot.getBasicTimeStep())

lidar = robot.getLidar('lidar')
lidar.enable(timestep)

# Main loop:
while robot.step(timestep) != -1:
    print(lidar.getRangeImage())
    pass
