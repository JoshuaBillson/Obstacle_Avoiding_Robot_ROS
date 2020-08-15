import rospy
from std_msgs.msg import Int16
from vnh5019_serial_controller.msg import MixedCommand
from time import sleep

distance = 0

def read_distance(data):
    global distance
    distance = data.data


def setup():
    pass

def robot_control():
    global distance
    motors = rospy.Publisher('vnh5019_motor_controller', MixedCommand, queue_size=10) 
    rospy.Subscriber('ultrasonic_sensor', Int16, read_distance)
    rospy.init_node("robot_control")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        motor_command = MixedCommand()
        if distance > 15:
            motor_command.speed = 100
            motor_command.turn = 0
        else:
            motor_command.speed = 0
            motor_command.turn = 0
        motors.publish(motor_command)
        rate.sleep()

