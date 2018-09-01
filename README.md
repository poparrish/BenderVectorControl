# BenderVectorControl
New manual control law for bender using vector addition



#ROS folder: All the files needed to run the ROS graph for manual control are located in this folder. The graph is designed to have 3 running nodes. 
The first is the joy_node. This is essentially a driver for all general logitech/xbox/playstation controllers: http://wiki.ros.org/joy. 
The second can be one of three: VectorCalcSingleStick_node or VectorCalcDualStick_node or VectorCalcTriStick_node. They are just for different joystick types. The most stable & reccomended is the VecorCalcDualStick_node, this one runs the logitech handheld controller. This just parses the relevant data we get from joy_node and publishes a 3 tuple of [speed,translation_vector,rotation_vector]. 
The third is TeensyWrite_node. This is the node where all the math from the Vector Steering Writeup is implemented. Once the calculations are performed serial data is written as a newline string to each of the wheel assemblies containting the desired bearing and speed. 


#TeensyWrite folder: This folder contains all the c++ code running on the Teensy for manual control
This code is mostly identical to what runs during autonomous navigation. The only difference is in the PID gains and the data that the Teensy echos back to the TeensyWrite_node for wheel wraparound control.


#VectorControlCSVlogging: this is used for control law debugging by logging recorded data to csv files
1. Open the main TeensyJoy.ino class in the Arduino IDE and specify what returnVariables should be reported back to the laptop. To do this see line 600. Each value separated by a comma will be parsed as an int or float to be logged in the csv. Note the total number of return variables. 
2. Open VectorControlCSVlogging.py and see the getLatestData() method. Set the range(0,__) so that it matches the number of return variables on the Teensy. ser.write(str(params)) is where the laptop writes data to the Teensy, so for the test to work each of the params must have a value (speed, velocity_vector, theta_dot). A few test cases are given and commented out.
3. Plug in the teensy cable first, and the relay cable second(yellow). 
4. Upload teensy code from Arduino IDE (ensure that the "port" option under "tools" matches the Teensy USB address)
4a. If you haven't already get the Teensyduino addon software https://www.pjrc.com/teensy/td_download.html
5. Finally run the python script from a terminal, and THEN power on bender. You must do it in this order. If the hub motors are not being written a PWM when the MCs are powered on the hub motors may not all turn.
