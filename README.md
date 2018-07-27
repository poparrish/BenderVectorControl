# BenderVectorControl
New manual control law for bender using vector addition


#VectorControlCSVlogging this is used for control law debugging by logging recorded data to csv files

Configure code:
1. Open the main TeensyJoy.ino class in the Arduino IDE and specify what returnVariables should be reported back to the laptop. To do this see line 600. Each value separated by a comma will be parsed as an int or float to be logged in the csv. Note the total number of return variables. 
2. Open VectorControlCSVlogging.py and see the getLatestData() method. Set the range(0,__) so that it matches the number of return variables on the Teensy. ser.write(str(params)) is where the laptop writes data to the Teensy, so for the test to work each of the params must have a value (speed, velocity_vector, theta_dot). A few test cases are given and commented out.

Running code:
3. Plug in the teensy cable first, and the relay cable second(yellow). 
4. Upload teensy code from Arduino IDE (ensure that the "port" option under "tools" matches the Teensy USB address)
4a. If you haven't already get the Teensyduino addon software https://www.pjrc.com/teensy/td_download.html
5. Finally run the python script from a terminal, and THEN power on bender. You must do it in this order. If the hub motors are not being written a PWM when the MCs are powered on the hub motors may not all turn.
