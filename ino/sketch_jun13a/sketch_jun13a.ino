// This code is reading data from an ultrasonic sensor
// and sending it through the serial port. The objective
// is to send these data to Matlab


// ultrasonic pins
int trig = 8;     // trigger
int echo = 9;     // echo
float distance;   // distance (cm)

// reading ultrasonic data
long readUltrasonicDistance(int triggerPin, int echoPin)
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(echoPin, HIGH);
}

// time variables (used to calculate the elapsed time)
long prevT = 0;   // previous time
long currT = 0;   // current time

void setup() {
  // ultrasonic pin modes
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // start the serial communication
  Serial.begin(9600);
}

void loop() {
  currT = micros(); // current time
  float deltaT = ((float) (currT - prevT)/1.0e6);   // time variation
  prevT = currT;    // previous time
  
  distance = 0.01723 * readUltrasonicDistance(8, 9);    // distance (cm)

  // Serial communication
  Serial.print(currT/1.0e6);  // elapsed time (s)
  Serial.print(" ");      
  Serial.println(distance);   // distance (cm)
  
  delay(100);
}
