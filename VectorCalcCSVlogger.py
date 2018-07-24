#!/usr/bin/env python
import math
import time
import serial
from collections import deque
import collections
import csv
import sys

ser = serial.Serial('/dev/ttyACM0', 250000)


Hz = 10
LENGTH = .66675
WIDTH = .6223
RADIUS = math.sqrt(LENGTH/2*LENGTH/2+WIDTH/2*WIDTH/2)
psi0 = 43
psi1 = 133
psi2 = -133
psi3 = -43
matrix = []


def calcWheel(speed, velocity_vector, theta_dot, wheel_psi, plan_angle):

    theta_dot_rad = theta_dot * 0.0174533
    rot_speed = math.fabs(theta_dot_rad) * RADIUS
    # START calcWheel first determine what combination of trans + rot
    if math.fabs(speed) < 0.05:
        if math.fabs(rot_speed) > .001:
            print 'first if'
    # todo point turn
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
    while ser.inWaiting() > 0:
        print ser.inWaiting()
        toParse = ser.readline()
        #toParse = "8.02,5,2,-19,90"
        parsed = toParse.split(",")
        data = []
        for num in range(0,9):
            try:
                data.append(int(parsed[num]))
            except ValueError:
                data.append(float(parsed[num]))
        return data


def start():
    global count
    global matrix
    start = time.time()
    print count
    if count < 40 :
        speed = .4
        #speed = .5
    elif (count < 80) and (count >= 40) :
        speed = .9
    else:
        speed = 1.4
    #speed = 1
    theta_dot = 0
    print speed
    data = getLatestData()
    #data.append(int(speed))
    if data:

        #shift floats back to the correct decimal place. Shifted over prior to sending in arduino
        # to make python deserializing simpler
        metersTraveled = data[0]
        metersTraveled = float(metersTraveled) / 100
        data[0] = metersTraveled
        speedRPM = speed / 0.0085922
        data.append(speedRPM)

        #write changes to wheels
        velocity_vector = metersTraveled * theta_dot * -1
        ser.write(str('W0'+calcWheel(speed, velocity_vector, theta_dot, psi0, data[1])+'\n'))
        ser.write(str('W1'+calcWheel(speed, velocity_vector, theta_dot, psi1, data[2])+'\n'))
        ser.write(str('W2'+calcWheel(speed, velocity_vector, theta_dot, psi2, data[3])+'\n'))
        ser.write(str('W3'+calcWheel(speed, velocity_vector, theta_dot, psi3, data[4])+'\n'))
        ser.reset_input_buffer()

        #log changes to .csv file
        writer = csv.writer(open('test11.csv', "wb"))
        matrix.append(data)

        if count > 120:
            for entries in matrix:
                print matrix
                cols = zip(entries)
                writer.writerow(cols)
            sys.exit()


    while True:
        current_time = time.time()
        if current_time -start >= 1.0/Hz:
            count = count+1
            break



if __name__ == '__main__':
    count = 1
    while True:
        start()