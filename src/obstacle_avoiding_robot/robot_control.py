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

# Read-Only Globals (May Be Altered By Callbacks)
DISTANCE = 0
THRESHOLD = 0
TURN_DELAY = 0

# Publisher For Controlling Motors
motors = rospy.Publisher('vnh5019_motor_controller', MixedCommand, queue_size=10) 


def parameter_callback(config, level):
    """
    Dynamic Reconfigure Callback.
    
    config (dictionary): Parameter Configuration Message.
    level (int): A Bit-Mask Denoting Which Parameters Have Changed.
    Returns: A config object containing any changes we may have made.
    """
    global THRESHOLD, TURN_DELAY
    THRESHOLD = config["obstacle_threshold"]
    TURN_DELAY = config["turn_delay"]
    return config


def get_distance(data):
    """
    A Subscriber Callback For Reading Distance From The Ultrasonic Sensors.

    data (std_msgs.Int16): A distance reading from the sensors.
    Returns: None.
    """
    global DISTANCE 
    DISTANCE = data.data


def stop():
    """Stop The Robot"""
    global motors
    motors.publish(MixedCommand(speed=0, turn=0))
    sleep(1.5)


def turn_left():
    """Turn The Robot Left"""
    global motors
    motors.publish(MixedCommand(speed=0, turn=-50))


def turn_right():
    """Turn The Robot Right"""
    global motors
    motors.publish(MixedCommand(speed=0, turn=50))


def forward():
    """Drive The Robot Forward"""
    global motors
    motors.publish(MixedCommand(speed=100, turn=0))


def reverse():
    """Drive The Robot In Reverse"""
    global motors
    motors.publish(MixedCommand(speed=-100, turn=0))


def setup():
    """Setup Routine"""
    # Initialize Subscribers
    rospy.Subscriber('ultrasonic_sensor', Int16, get_distance)

    # Initialize Node
    rospy.init_node("robot_control")

    # Initialize Parameter Server
    srv = Server(ParametersConfig, parameter_callback)


def loop():
    """Main Program Loop"""
    global DISTANCE, THRESHOLD, TURN_DELAY 
    while not rospy.is_shutdown():
        while DISTANCE > THRESHOLD:
            forward()
            sleep(0.1)
        stop()
        while DISTANCE <= THRESHOLD:
            turn_left()
        sleep(TURN_DELAY)
        stop()


def robot_control():
    """ Executable """
    setup()
    loop()

