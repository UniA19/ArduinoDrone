/*
*This is a Test if tracking the relative position of the MPU6050 is feasable.
*/


#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 accGyro(Wire);

//global Variable to remember the time the last accelration Value was calculated
unsigned long pastIterationTime;

//same for the Velocity and Position the device is thought to be in
float velo[3] = {0, 0, 0};
float pos[3] = {0, 0, 0};


void setup() {
  Serial.begin(9600);
  Wire.begin();
  accGyro.begin();
  accGyro.calcGyroOffsets();
}

/*
*Will Crash after ~70min for 1 iteration because the internal microsecond timer
*overflows
*/
void loop() {
  accGyro.update();
  unsigned long iterationTime = millis();
  unsigned long deltaTime = iterationTime - pastIterationTime;
  pastIterationTime = iterationTime;
  float accel[3] = {accGyro.getAccX(), accGyro.getAccY(), accGyro.getAccZ()};// Unit= 9.81 m/(s^2);
  int i;
  for(i = 0; i < 3; i++) {
    velo[i] += accel[i] * deltaTime;
    pos[i] += velo[i] * deltaTime;
  }
  Serial.print("Position: ");
  Serial.print(pos[0]);
  Serial.print(" - ");
  Serial.print(pos[1]);
  Serial.print(" - ");
  Serial.print(pos[2]);
}
