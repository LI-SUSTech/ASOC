"""asoc_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
yaw_encoder_3=robot.getDevice('yaw_encoder_3')
yaw_encoder_3.enable(timestep)

motor_4_l=robot.getDevice('motor_4_l')
motor_4_r=robot.getDevice('motor_4_r')
motor_4_l.setPosition(float('inf'))
motor_4_r.setPosition(float('inf'))
motor_4_l.setVelocity(5.0)
motor_4_r.setVelocity(5.0)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    print(motor_4_l.getVelocity())
    print(yaw_encoder_3.getValue())
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
