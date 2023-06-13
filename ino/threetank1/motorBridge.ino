int enable = 3;
int input1 = 2;
int input2 = 4;
int trig = 8;		// trigger pin
int echo = 9;		// echo pin

float distance = 0;	// distance from HC SR04 to the water (cm)
float height = 0;	// water level (cm) 

float setpoint = 10;		// system's setpoint (cm)
float error_integral = 0;	// error integral

// controller parameters
float kC = 3;		// kC
float tauI = 60;	// tauI

// time variables
long prevT = 0;		// previous time


long input = 0;		// L293D input (bit)


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

void setup() {
  // put your setup code here, to run once:
  pinMode(enable, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(input1, 1);
  digitalWrite(input2, 0);

  long currT = micros();
  float deltaT = ((float) (currT - prevT)/1.0e6);
  prevT = currT;

  distance = 0.01723 * readUltrasonicDistance(8, 9);
  height = 24.5 - distance;

  float error = setpoint - height;
  error_integral = error_integral + error*deltaT;
  
  float P = kC*error;
  float I = (kC/tauI) * error_integral;

  float u = P + I;

  if(u < 0.0){
    u = 0.0;
    error_integral = error_integral - error*deltaT;
  }
  
  if (u > 27.8) {
  	u = 27.8;
    error_integral = error_integral - error*deltaT;
  }

  input = ((u/27.8)*255);
  if (input > 255){
    input = 255;
  }  

  analogWrite(enable, input);

  float ll = 0;
  float ul = 20;

  Serial.print(ul);
  Serial.print(", ");
  Serial.print(ll);  
  Serial.print(", ");
  Serial.print(height);
  Serial.print(" cm, ");
  Serial.print(setpoint);
  Serial.print(" cm, ");
  Serial.print(u);
  Serial.println(" cm3/s, ");
  // Serial.print(input);
  // Serial.println(" bit");
  delay(10);
}
