# ROS Client
import rospy

# Messages and Services
from std_msgs.msg import Int16
from vnh5019_serial_controller.msg import MixedCommand

# Standard Libraries
from time import sleep

# Dynamic Reconfigure Libraries
from dynamic_reconfigure.server import Server
from obstacle_avoiding_robot.cfg import ParametersConfig

DISTANCE = 0
THRESHOLD = 0
TURN_DELAY = 0


def parameter_callback(config, level):
    global THRESHOLD, TURN_DELAY
    THRESHOLD = config["obstacle_threshold"]
    TURN_DELAY = config["turn_delay"]
    return config


def get_distance(data):
    global DISTANCE 
    DISTANCE = data.data


def robot_control():
    global DISTANCE, THRESHOLD, TURN_DELAY 
    # Initialize Publishers And Subscribers
    motors = rospy.Publisher('vnh5019_motor_controller', MixedCommand, queue_size=10) 
    rospy.Subscriber('ultrasonic_sensor', Int16, get_distance)

    # Initialize Node
    rospy.init_node("robot_control")

    # Initialize Parameter Server
    srv = Server(ParametersConfig, parameter_callback)

    # Main Loop
    rate = 10
    while not rospy.is_shutdown():
        motor_command = MixedCommand()
        if DISTANCE > THRESHOLD:
            motor_command.speed = 100
            motor_command.turn = 0
            delay = 1 / rate
        else:
            motor_command.speed = 0
            motor_command.turn = 0
            delay = TURN_DELAY
        motors.publish(motor_command)
        sleep(delay)

