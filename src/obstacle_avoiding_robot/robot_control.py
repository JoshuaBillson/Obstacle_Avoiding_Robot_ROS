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


def generate_message(speed, turn):
    """
    Generate a message to be published to the motor controller.

    speed (int): The speed value.
    turn (int): The turn value.
    returns: A MixedCommand message.
    """
    message = MixedCommand()
    message.speed = speed
    message.turn = turn
    return message


def stop():
    global motors
    motors.publish(generate_message(0, 0))


def turn_left():
    global motors
    motors.publish(generate_message(0, -50))


def turn_right():
    global motors
    motors.publish(generate_message(0, 50))


def forward():
    global motors
    motors.publish(generate_message(100, 0))


def reverse():
    global motors
    motors.publish(generate_message(-100, 0))


def setup():
    """ Setup Routine """
    # Initialize Subscribers
    rospy.Subscriber('ultrasonic_sensor', Int16, get_distance)

    # Initialize Node
    rospy.init_node("robot_control")

    # Initialize Parameter Server
    srv = Server(ParametersConfig, parameter_callback)


def loop():
    """ Main Program Loop """
    global DISTANCE, THRESHOLD, TURN_DELAY 
    RATE = 10
    while not rospy.is_shutdown():
        if DISTANCE > THRESHOLD:
            forward()
        else:
            stop()
            sleep(2)
            while DISTANCE <= THRESHOLD:
                turn_left()
            sleep(TURN_DELAY)
            stop()
            sleep(2)
        sleep(1 / RATE)


def robot_control():
    """ Executable """
    setup()
    loop()

