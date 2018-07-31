#!/usr/bin/env python
import math


# Author: parker (the bodaciousjedi)
#CalculationTest script is just used to verify that the numbers produced by the calcWheel method make sense. Enter in the
#numbers you want to test in the main method and then the results will be printed to the terminal for each wheel.

Hz = 10
LENGTH = .66675
WIDTH = .6223
RADIUS = math.sqrt(LENGTH/2*LENGTH/2+WIDTH/2*WIDTH/2)
psi0 = 43
psi1 = 133
psi2 = -133
psi3 = -43



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

    theta_dot_rad = theta_dot*0.0174533
    rot_speed = math.fabs(theta_dot_rad) * RADIUS
    rot_speed_deg = math.fabs(theta_dot) * RADIUS
    # START calcWheel first determine what combination of trans + rot
    if math.fabs(speed) < 0.05:
        if math.fabs(rot_speed) > .001:
            print 'first if'
    # todo point turn
    else:
        if math.fabs(rot_speed) < .001: #just translation
            wheel_theta = velocity_vector
            wheel_speed = speed
            wheelString =  str('  B:' + str(wheel_theta) + '  S:' + str(wheel_speed))
            return wheelString
        else:
            if(theta_dot < 0):
                angle_vv_rot=wheel_psi-velocity_vector-90
            else:
                angle_vv_rot=wheel_psi-velocity_vector+90
            sup_angle = 180-angle_vv_rot
            if(sup_angle>180):
                sup_angle=sup_angle-360
            if(sup_angle<-180):
                sup_angle= sup_angle+360

            #now convert everything to rads and meters
            sup_angle_rad = sup_angle*0.0174533
            velocity_vector_rad=velocity_vector*0.0174533
            wheel_speed = math.sqrt(speed*speed+rot_speed*rot_speed-2*speed*rot_speed*math.cos(sup_angle_rad))
            wheel_speed_deg = math.sqrt(speed*speed+rot_speed_deg*rot_speed_deg-2*speed*rot_speed_deg*math.cos(sup_angle_rad))
            delta_theta = math.asin((math.sin(sup_angle_rad)*rot_speed)/wheel_speed)
            #delta_theta = math.asin((math.sin(sup_angle)*rot_speed)/wheel_speed)
            wheel_theta = velocity_vector_rad+delta_theta
            print wheel_speed
            wheel_speed = wheel_speed / 0.0085922  # for no orange tread
            wheel_theta = wheel_theta * 57.2958

            wheelString = str('  B:' + str(int(wheel_theta)) + '  S:' + str(int(wheel_speed)))

            return wheelString

def start():
    """
    Main method. enter in values for the 3 variables and then they will be passed through to 4 instances of the calcWheel
    method with the results printed to the terminal as strings. \n strings are the same data type written to the Teensy
    :return: nothing
    """
    #NOTE -velocity_vector cannot be translation theta...need new logic to do trans+rot
    speed = 1
    theta_dot = 40
    velocity_vector = -50
    print str('W0' + calcWheel(speed, velocity_vector, theta_dot, psi0) + '\n')
    print str('W1' + calcWheel(speed, velocity_vector, theta_dot, psi1) + '\n')
    print str('W2' + calcWheel(speed, velocity_vector, theta_dot, psi2) + '\n')
    print str('W3' + calcWheel(speed, velocity_vector, theta_dot, psi3) + '\n')


if __name__ == '__main__':
    start()