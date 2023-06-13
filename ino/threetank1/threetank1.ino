// C++ code
//
int enable1 = 10;
int input1 = 6;
int input2 = 12;

int enable2 = 3;
int input3 = 2;
int input4 = 4;

int trig1 = 5;
int echo1 = 7;

int trig2 = 8;
int echo2 = 9;

float setpoint = 25;
float distance;
float error_integral = 0;

float kC;
float tauI;

long prevT = 0;
int pwm = 0;

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


void setup()
{
  Serial.begin(9600);
  
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  
  digitalWrite(input3, 1);
  digitalWrite(input4, 0);
}

void loop()
{
  kC = 3.66e-2;
  tauI = 177.7;
  
  long currT = micros();
  float deltaT = ((float) (currT - prevT)/1.0e6);
  prevT = currT;
  
  distance = 0.01723 * readUltrasonicDistance(trig2, echo2);
  
  float error = setpoint - distance;
  error_integral = error_integral + error*deltaT;
  
  float P = kC*error;
  float I = (kC/tauI) * error_integral;
  
  float u = P + I;
  
  if(u < 0.0){
    u = 0.0;
    error_integral = error_integral - error*deltaT;
  }
  
  if (u > 25) {
  	u = 25;
    error_integral = error_integral - error*deltaT;
  }
  
  pwm = ((u/1)*255);
  if (pwm > 255){
    pwm = 255;
  }
  
  analogWrite(enable2, pwm);

  Serial.print(deltaT);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print("cm, ");
  Serial.print(error);
  Serial.print("cm, ");
  Serial.print(u);
  Serial.print("cm3/s, ");
  Serial.println(pwm);
  delay(500); // Wait for 1000 millisecond(s)
}