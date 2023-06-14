int enable = 3;
int input1 = 2;
int input2 = 4;

int trig = 8;
int echo = 9;
float distance = 0;
float height = 0;

float setpoint = 15;
float error_integral = 0;
float kC = 3;
float tauI = 60;

long prevT = 0;
long input = 0;

const int k = 10;
float buffer[k];
int bufferIndex = 0;
float sum = 0;

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
  height = 25 - distance;

  sum = sum - buffer[bufferIndex] + height;
  buffer[bufferIndex] = height;
  bufferIndex = (bufferIndex + 1) % k;

  height = sum / k;

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

   if (currT/1.0e6 < 120) {
     u = 15;
  } 
  // else if ((currT/1.0e6 < 320)) {
  //   u = 20;
  // } 

  input = ((u/27.8)*255);
  if (input > 255){
    input = 255;
  }

  analogWrite(enable, 0);

  float ll = 5;
  float ul = 15;

  Serial.print(currT/1.0e6);
  Serial.print(" ");
  // Serial.print(ul);
  // Serial.print(", ");
  // Serial.print(ll);  
  //Serial.print(", ");
  //Serial.print(distance);
  // Serial.print(", ");
  Serial.print(height);
  Serial.print(" ");
  // Serial.print(setpoint);
  // Serial.print(", ");
  Serial.println(u);
  // Serial.println(" cm3/s");
  delay(200);
}
