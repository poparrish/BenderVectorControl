#!/usr/bin/env python
import math
import serial
import rospy
from std_msgs.msg import String

# Author: parker (the bodaciousjedi)
#teensyWrite_node is subscribed to the 'vectors' topic from VectorCalc_node
#It outputs wheel commands to the Teensy after processing desired vectors from the 'vectors' topic
#All IMU and GPS data should plug in here


ser = serial.Serial('/dev/ttyACM0', 250000)
Hz = 10
LENGTH = .66675
WIDTH = .6223
MAX_TURNS = 1
RADIUS = math.sqrt(LENGTH/2*LENGTH/2+WIDTH/2*WIDTH/2)
psi0 = 43
psi1 = 133
psi2 = -133
psi3 = -43
vectors = [0,0,0]
current_angle = 0
desired_total_angle = 0
last_desired = 0
total_angle = 0
desired_angle = 0
current_angle = 0
reset = False

def calcWheel(speed, velocity_vector, theta_dot, wheel_psi):
    """
    this is the method that actually does the calculations. this takes desired vectors and outputs data for the
    individual wheels. nearly identical to the writeup that Dr. Swanson produced.
    :param speed: desired speed at center of Bender
    :param velocity_vector: desired translational bearing
    :param theta_dot: desired rotational vector
    :param wheel_psi: location on the+-180 scale of each wheel
    :return: wheelString. a string representing desired bearing and speed for the current wheel
    """

    theta_dot_rad = theta_dot * 0.0174533
    rot_speed = math.fabs(theta_dot_rad) * RADIUS
    if math.fabs(speed) < 0.05:
        if math.fabs(rot_speed) > .001:
            wheelString = str('B0' + 'S0')
            return wheelString
        else:
            wheel_theta = velocity_vector
            wheelString = str('B' + str(wheel_theta) + 'S0')
            return wheelString
    else:
        if math.fabs(rot_speed) < .001:  # just translation
            wheel_theta = velocity_vector
            wheel_speed = speed / 0.0085922
            wheelString = str('B' + str(wheel_theta) + 'S' + str(wheel_speed))
            return wheelString
        else:
            if (theta_dot < 0):
                angle_vv_rot = wheel_psi - velocity_vector - 90
            else:
                angle_vv_rot = wheel_psi - velocity_vector + 90
            sup_angle = 180 - angle_vv_rot
            if (sup_angle > 180):
                sup_angle = sup_angle - 360
            if (sup_angle < -180):
                sup_angle = sup_angle + 360

            # now convert everything to rads and meters
            sup_angle_rad = sup_angle * 0.0174533
            velocity_vector_rad = velocity_vector * 0.0174533
            wheel_speed = math.sqrt(speed * speed + rot_speed * rot_speed - 2 * speed * rot_speed * math.cos(sup_angle_rad))
            delta_theta = math.asin((math.sin(sup_angle_rad) * rot_speed) / wheel_speed)
            wheel_theta = velocity_vector_rad + delta_theta
            wheel_speed = wheel_speed / 0.0085922  # for no orange tread
            wheel_theta = wheel_theta * 57.2958
            wheelString = str('B' + str(int(wheel_theta)) + 'S' + str(int(wheel_speed)))
            #print plan_angle
            return wheelString


def getLatestData():
    """
    parses the data in the serial buffer sent back from the teensy. Currently I believe we are sending the total
    distance traveled in meters, the current angle of each planetary motor, and the current speed of each hub motor.
    :return:
    """
    while ser.inWaiting() > 0:
        toParse = ser.readline()
        parsed = toParse.split(",")
        data = []
        for num in range(0, 9):
            try:
                data.append(int(parsed[num]))
            except ValueError:
                data.append(float(parsed[num]))
        return data

def checkDir(current_angle, desired_angle):
    """
    determines if shortest distance to desired angle is tracing right or left and then returns the differnce along with
    a number representing whether the difference should be added or subtracted to the total_angle
    :param current_angle: current angle represented as a range of 0to360 going clockwise
    :param desired_angle: desired angle reported from joystick data on the same range as current_angle
    :return: int,difference.
    """
    difference = 0
    current_angle = current_angle%360
    if current_angle <0:
        current_angle=current_angle*-1
    if current_angle == desired_angle:
        return 0,difference
    elif (current_angle == 0 and desired_angle <= 180):
        difference = desired_angle-current_angle
        return 1,difference
    elif (current_angle == 0 and desired_angle > 180):
        difference = 360 - desired_angle
        return -1,difference
    elif desired_angle == 0 and current_angle <= 180:
        difference = current_angle
        return -1,difference
    elif desired_angle == 0 and current_angle > 180:
        difference = 360-current_angle
        return 1,difference
    elif current_angle > desired_angle and current_angle-desired_angle > 0 and current_angle-desired_angle<180:
        if desired_angle < current_angle:
            difference = current_angle-desired_angle
        else:
            difference = 360 - desired_angle+(current_angle)
        return -1,difference
    else:
        if desired_angle > current_angle:
            if(desired_angle-current_angle>180):
                difference = 360-desired_angle+current_angle
                difference = difference*-1
            else:
                difference = desired_angle-current_angle
        else:
            difference = 360-current_angle+desired_angle
        return 1,difference

def callback(data):
    """
    this runs everytime the ROS subscriber is updated. belive its every 10hz. This writes data to the teensy, and tracks
    for over wrapping on the wheels to avoid twisted wires.
    :param data: list of 3 vectors recieved from subscriber
    :return: nothing
    """
    global desired_total_angle
    global last_desired
    global total_angle
    global desired_angle
    global current_angle
    global reset
    toParse = data.data.split(",")
    vectors = []
    for num in range(0, 3):
        try:
            vectors.append(int(toParse[num]))
        except ValueError:
            vectors.append(float(toParse[num]))

    # get serial data feedback from arduino
    serial = getLatestData()

    # shift floats back to the correct decimal place. Shifted over prior to sending in arduino to make python deserializing simpler
    metersTraveled = serial[0]
    metersTraveled = float(metersTraveled) / 100
    serial[0] = metersTraveled
    desired_angle = vectors[1]

    #make +360 and check/add angles joystick is tracking
    if(desired_angle <0):
        desired_angle+=360
    joyUpdate = checkDir(current_angle,desired_angle)
    if(joyUpdate[0] == 1):#going right
        desired_total_angle += joyUpdate[1]
    if(joyUpdate[0] == -1):#going left
        desired_total_angle -= joyUpdate[1]
    current_angle = desired_angle

    #intelligent zeroing if wires wind up too much & writes data to the teensy
    if ((serial[1] or serial[2] or serial[3] or serial[4]) > (360*MAX_TURNS)) or ((serial[1] or serial[2] or serial[3] or serial[4]) < (-360 * MAX_TURNS)):
        reset = True
    if serial:
        if(reset):
            desired_total_angle = 0
            velocity_vector = desired_total_angle
            theta_dot = 0
            speed = vectors[0] * .01
            ser.write(str('W0' + calcWheel(speed, velocity_vector, theta_dot, psi0) + '\n'))
            ser.write(str('W1' + calcWheel(speed, velocity_vector, theta_dot, psi1) + '\n'))
            ser.write(str('W2' + calcWheel(speed, velocity_vector, theta_dot, psi2) + '\n'))
            ser.write(str('W3' + calcWheel(speed, velocity_vector, theta_dot, psi3) + '\n'))
            ser.reset_input_buffer()
            if(serial[1] == 0 and serial[2] == 0 and serial[3] == 0 and serial[4] == 0):
                reset = False
        else:
            velocity_vector = desired_total_angle
            theta_dot = 0
            speed = vectors[0] * .01
            ser.write(str('W0' + calcWheel(speed, velocity_vector, theta_dot, psi0) + '\n'))
            ser.write(str('W1' + calcWheel(speed, velocity_vector, theta_dot, psi1) + '\n'))
            ser.write(str('W2' + calcWheel(speed, velocity_vector, theta_dot, psi2) + '\n'))
            ser.write(str('W3' + calcWheel(speed, velocity_vector, theta_dot, psi3) + '\n'))
            ser.reset_input_buffer()


def start():
    """
    main method where callback is triggered from ROS subscriber
    :return: nothing
    """
    rospy.Subscriber("vectors", String, callback)
    rospy.init_node('TeensyWrite_node')
    rospy.spin()


if __name__ == '__main__':
    start()