#!/usr/bin/env python
from sensor_msgs.msg import Joy
import math
import rospy
from std_msgs.msg import String
import serial
import os
import getpass
from geometry_msgs.msg import Vector3


# Author: parker (the bodaciousjedi)
# VectorCalc_node is subscribed to the 'joy' topic from the joy node: http://wiki.ros.org/joy
# and publishes a list of vectors [speed, velocity_vector, theta_dot]
# to the 'vectors' topic

returnVectors = [0, 0]#just the logitech inputs. append dial data from arduino later
horn = 0 #whether or not horn button is depressed. set in callback
ser = serial.Serial('/dev/ttyACM0',250000)


def callback(data):
    """
    runs when joys_node updates. Funnels joy_node inputs, and reads data stored in the dials serial buffer.
    :param data: return object from joy_node
    :return: nothing
    """
    global returnVectors
    global horn
    SPEED_THROW = 2.5 #M/s
    translation = data.axes[0]*450
    speed = calc_speed(data.axes[1], SPEED_THROW)
    returnVectors = [speed,translation]
    horn = data.buttons[19]
    #print horn

def calc_speed(pedal,speedCap):
    """
    takes raw joystick data in and determines what speed to write. Manages preset speedcap
    :param speed: input from 'gas' pedal
    :return: speed. bender speed in M/s
    """
    if pedal < 0:#first half
        pedal *=-.5
        pedal = .5-pedal
    elif pedal == 0.0:
        pedal = .5
    else:#second half
        pedal *=.5
        pedal +=.5
    pedal *=speedCap
    speed = pedal
    if speed > speedCap:
        speed = speedCap
    return speed

def get_dial_pos(last_theta_dot,THETA_DOT_THROW,THETA_DOT_DEADBAND,THETA_CAP):
    """
    parses the data in the serial buffer sent back from the Arduino running the dial. Arduino should update 2x as fast
    as the rospy.Rate() refresh rate. it takes the 0-1023 range the arduino AnalogRead() defaults to and converts it to
    the desired theta_dot range
    :param last_theta_dot: int. The last theta dot that was calculated. Incase refreshrate is faster than arduino
    :return: theta_dot: int
    """
    try: #need to catch when we try to read an empty buffer if Joy updates faster than arduino
        while ser.inWaiting() > 0:
            dialIn = int(ser.readline())
        multiplier = float(THETA_DOT_THROW / 1023) * 2
        theta_dot = int(dialIn * multiplier)
        if theta_dot == THETA_DOT_THROW:
            theta_dot = 0
        else:
            theta_dot = THETA_DOT_THROW - theta_dot
    except UnboundLocalError:#when wheel updates faster than arduino
        theta_dot = last_theta_dot
    except ValueError:#boot up noise
        theta_dot = 0
    if theta_dot < THETA_DOT_DEADBAND and theta_dot > -THETA_DOT_DEADBAND:#deadband
        theta_dot = 0
    if theta_dot > THETA_CAP:
        theta_dot=THETA_CAP
    if theta_dot < -THETA_CAP:
        theta_dot = -THETA_CAP
    return theta_dot

def configure_steering_wheel():
    """
    prompts user for admin password and executes command to logitech steeringwheel range to max 900 degrees
    :return:nadathing
    """
    p = getpass.getpass()
    os.popen("sudo -S sudo ltwheelconf --wheel DFGT --nativemode --range 900", 'w').write(p)

def start():
    """
    main method. where the ROS publishers and subscribers live.
    :return: nothing
    """
    last_theta_dot = 0
    THETA_DOT_THROW = 250.0  # max influence of that_dot in degrees. This is +-THETA_DOT_THROW for a total of THETA_DOT_THROW*2
    THETA_CAP = 200.0
    THETA_DOT_DEADBAND =25 #deadband for theta_dot
    pub = rospy.Publisher('vectors', String, queue_size=1)
    #pub = rospy.Publisher("vectors",Vector3,queue_size=1)
    rospy.init_node('SteeringWheel_node')
    r = rospy.Rate(30)
    rospy.Subscriber("joy", Joy, callback)  # use this if joystick
    while not rospy.is_shutdown():
        theta_dot = get_dial_pos(last_theta_dot, THETA_DOT_THROW,THETA_DOT_DEADBAND,THETA_CAP)
        last_theta_dot = theta_dot
        #vectorMsg = Vector3(x=returnVectors[0]*100,y=returnVectors[1]*100,z=theta_dot)
        returnString = str(str(int(returnVectors[0] * 100)) + ',' + str(int(returnVectors[1])) + ',' + str(int(theta_dot)) + ',' + str(int(horn)))
        #print vectorMsg
        print returnString
        #pub.publish(vectorMsg)
        pub.publish(returnString)
        r.sleep()
    rospy.spin()

if __name__ == '__main__':
    #configure_steering_wheel()
    start()
