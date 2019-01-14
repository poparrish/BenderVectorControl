int potPin = A2;    // select the input pin for the potentiometer
int val = 0;       // variable to store the value coming from the sensor
int delayms = 10;
int retVal = 0;
int CALIBRATED_MIDPOINT = 476; //where we want to calibrate the middle of the dial to

void setup() {
  Serial.begin(9600);
  pinMode(potPin, INPUT);  // declare the ledPin as an OUTPUT
}

void loop() {
  val = analogRead(potPin);    // read the value from the sensor
  delay(delayms);                  // stop the program for some time

  if (val == CALIBRATED_MIDPOINT){
    retVal = 512;
  }else if( val < CALIBRATED_MIDPOINT){
    retVal = val*1.0745;
  }else if(val > CALIBRATED_MIDPOINT){
    retVal = val*.935;
  }else{
    retVal = val;
  }

  retVal = val;

  Serial.println(retVal);
}
