#!/usr/bin/env python
from sensor_msgs.msg import Joy
import math
import rospy
from std_msgs.msg import String

# Author: parker (the bodaciousjedi)
# VectorCalc_node is subscribed to the 'joy' topic from the joy node: http://wiki.ros.org/joy
# and publishes a list of vectors [speed, velocity_vector, theta_dot]
# to the 'vectors' topic

returnVectors = [0, 0, 0]


def callback(data):
    """
    runs when joys_node updates. Funnels joy_node inputs.
    :param data: return object from joy_node
    :return: nothing
    """
    global returnVectors
    speedCap = 1.2  # M/s
    deadBand = .05  # 5%
    turnCap = 45  #degrees for theta_dot
    slow = False
    stop = False
    yAxis = data.axes[1] * speedCap
    xAxis = data.axes[0] * speedCap
    zAxis = data.axes[2] * turnCap * -1
    if data.buttons[7] == 1:
        slow = True
    if data.buttons[1] == 1:
        stop = True

    speed = calc_speed(xAxis, yAxis, speedCap, slow, stop)
    velocity_vector = calc_bearing(xAxis, yAxis)
    theta_dot = zAxis
    returnVectors = calc_dead_band(speed, velocity_vector, theta_dot, speedCap, deadBand, turnCap)


def calc_dead_band(speed, velocity_vector, theta_dot, speedCap, deadBand, turnCap):
    """
    determines the 3 values (1 vector + 2 angles) that will be published to the 'vectors' topic
    :param speed: bender speed in M/s
    :param velocity_vector: translational angle in degrees
    :param theta_dot: rotational vector in degrees/s
    :param speedCap: max allowed speed in M/s
    :param deadBand: % of deadband in the controls
    :param turnCap: caps theta_dot
    :return: returnVectors. this is what gets published from this node
    """
    if speed < speedCap * deadBand:
        speed = 0
    if theta_dot < turnCap * deadBand and theta_dot > -turnCap * deadBand:
        theta_dot = 0
    returnVectors = [speed, velocity_vector, theta_dot]
    return returnVectors


def calc_speed(xAxis, yAxis, speedCap, slow, stop):
    """
    takes raw joystick data in and determines what speed to write
    :param xAxis: float 0-1 that represents joystick location on the xAxis
    :param yAxis: float 0-1 that represents joystick location on the yAxis
    :param speedCap: max allowed speed in M/s
    :param slow: bool for if slow button is pressed
    :param stop: bool for if stop button is pressed
    :return: speed. bender speed in M/s
    """
    speed = math.fabs(xAxis * xAxis + yAxis * yAxis)
    if stop:
        speed = 0
    if speed > speedCap:
        speed = speedCap
    if slow:
        speed *= .5
    return speed


def calc_bearing(xAxis, yAxis):
    """
    this method is just sorting out angles we decided on a scale that puts 0to180 degrees inside quadrants 1 and 4, and
    0to-179 degrees inside quadrants 2 and 3.
    :param xAxis: float 0-1 that represents joystick location on the xAxis
    :param yAxis: float 0-1 that represents joystick location on the yAxis
    :return: velocity_vector: translational angle in degrees
    """
    try:
        velocity_vector = math.atan(xAxis / yAxis)  # we do adj/opp here since we want zero degrees to be across the yAxis
        velocity_vector = velocity_vector * 57.2958  # convert radians to degrees
        velocity_vector = math.fabs(velocity_vector)  # convert to pos
        if yAxis < 0:
            if xAxis < 0:
                velocity_vector = (velocity_vector - 180) * -1
                velocity_vector = velocity_vector * -1
            else:
                velocity_vector = 180 - velocity_vector
        else:
            if xAxis < 0:
                velocity_vector = velocity_vector * -1
    except ZeroDivisionError:
        # determine if on x or y
        if xAxis == 0:
            velocity_vector = 0
        else:
            if xAxis > .01:
                velocity_vector = 90
            else:
                velocity_vector = -90
    return velocity_vector


def start():
    """
    main method. where the ROS publishers and subscribers live.
    :return: nothing
    """
    pub = rospy.Publisher('vectors', String, queue_size=1)
    rospy.init_node('VectorQueue_node')
    r = rospy.Rate(10)
    rospy.Subscriber("joy", Joy, callback)  # use this if joystick
    while not rospy.is_shutdown():
        returnString = str(str(int(returnVectors[0] * 100)) + ',' + str(int(returnVectors[1])) + ',' + str(int(returnVectors[2])))
        print returnString
        pub.publish(returnString)
        r.sleep()
    rospy.spin()


if __name__ == '__main__':
    start()





