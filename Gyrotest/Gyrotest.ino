/*
*This is a Test if tracking the relative position of the MPU6050 is feasible.
*/


#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 accGyro(Wire);

//global variable to remember the time when the last accelration value was calculated
unsigned long pastIterationTime;

//same for the velocity and position the device is thought to be in
float velo[3] = {0, 0, 0};
float pos[3] = {0, 0, 0};

float xAccOff, yAccOff, zAccOff;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  accGyro.begin();
  accGyro.calcGyroOffsets();
  xAccOff = accGyro.getAccX();
  yAccOff = accGyro.getAccY();
  zAccOff = accGyro.getAccZ();
}

/*
*will crash after ~70min for 1 iteration because the internal microsecond timer overflows
*/
void loop() {
  accGyro.update();
  unsigned long iterationTime = millis();
  unsigned long deltaTime = iterationTime - pastIterationTime;
  pastIterationTime = iterationTime;
  float accel[3] = {accGyro.getAccX(), accGyro.getAccY(), accGyro.getAccZ()};// Unit= 9.81 m/(s^2)
  int i;
  accGyro.update();
  for(i = 0; i < 3; i++) {
    velo[i] += accel[i];// * deltaTime;
    pos[i] += velo[i];// * deltaTime;
  }
  accGyro.update();
  Serial.print("Beschl: ");
  Serial.print(accel[0] - xAccOff);
  Serial.print(" ");
  Serial.print(accel[1] - yAccOff);
  Serial.print(" ");
  Serial.println(accel[2] - zAccOff);
  accGyro.update();
  /*Serial.print("Velocity: ");
  Serial.print(velo[0]);
  Serial.print(" - ");
  Serial.print(velo[1]);
  Serial.print(" - ");
  Serial.println(velo[2]);
  accGyro.update();
  /*Serial.print("Position: ");
  Serial.print(pos[0]);
  Serial.print(" - ");
  Serial.print(pos[1]);
  Serial.print(" - ");
  Serial.println(pos[2]);*/
  //delay(1);
}
