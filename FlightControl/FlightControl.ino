/*
 * This Program runs on the Arduino Nano to steer the Drone
 * It recieves Orders through Wifi as a String like such:
 * 
 * non-commands:
 * <#Yaw|Z|X|Y>
 * 
 * These are the desired speeds. They are represented in percent ranging from -100 to 100 where 100 is the max speed the drone can handle.
 * They are always signed and have 3 digits. (thus 0 is +000, 25 is +025 and -60 is -060
 * 
 * Yaw is Angular Speed. Positive means clockwise motion when viewed from below.
 * 
 * Examples:
 * Stay where you are: (if possible)
 * Transmitted String: <#+000|+000|+000|+000>
 * Fly up fast while turning slowly:
 * Transmitted String: <#+005|+070|+000|+000>
 *
 * Commands:
 * commands are passed wiht <*command> 
 * 
 * cal            Calibrate the drone (only call when drone is resting)
 * 
 */

#define INTERRUPT_PIN 2 //this is the Pin the INT from the mpu is connected to. It is used to interrupt the program flow when new data is ready. (on Arduino UNO Pin 2 or 3)

#define GYRO_OFFSET_X 51        //these values are taken from an example and quite the magic numbers TODO: find out how to calculate them and in general demystifying them
#define GYRO_OFFSET_Y 8         //if im not mistaken these are the starting values for the functions in the library used to calibrate the gyro
#define GYRO_OFFSET_Z 21
#define ACCEL_OFFSET_X 1150
#define ACCEL_OFFSET_Y -50
#define ACCEL_OFFSET_Z 1060

#define MAX_VELOCITY_X 30       //TODO: Change the random Values here to something reasonable
#define MAX_VELOCITY_Y 30
#define MAX_VELOCITY_Z 30
#define MAX_ANGLE_VELOCITY_YAW 30


#define WLAN_SSID "Drone"         //Name of the drone visible for devices
#define WLAN_PASSWORD "dronepw"   //WLAN Password
#define WLAN_CHANNEL "5"          //WLAN Channel
#define WLAN_TIMEOUT "2500"       //time in microseconds with no transmission that is still considered an active connection TODO: write function that lands the drone after lost connection

#include <I2Cdev.h>                           //Library for the MPU-6050 (Gyro and Accel)
#include <MPU6050_6Axis_MotionApps_V6_12.h>   //Firmware for the built in Processor of the Gyroscope (D igital M otion P rocessor) (DMP)
#include <Wire.h>                             //Arduino Library for IC2 Protocol (in this case to talk to the MPU-6050 on Pins A4(SDA) and A5(SCL))


MPU6050 mpu;

//gloabl variables for...
//keeping track of the last transmission via WLAN so the drone can shut off after set time when connection is lost
unsigned long timeOfLastTransmission;

//motion storage:
Quaternion q;                                 //Orientation as a Quaternion
VectorInt16 gyro;                             //Gyro Data
VectorFloat gravity;                          //Gravity
float ypr[3];                                 //Yaw Pitch Roll

//used to control the mpu:
uint16_t packetSize;                          //DMP Packet size TODO: find out if actually needed
uint8_t fifoBuffer[64];

//headers:
void startupWireless();
void startupDMP();

//desired drone state:                        //this is the state the drone steers towards, it is used to maneuver the Drone
float desiredAngleX;                          //Angles for accelration in the x and y directions in Rad
float desiredAngleY;

float desiredVelocityX;                        //Velocities that are updated by the WLAN-module
float desiredVelocityY;                        //in m/s
float desiredVelocityZ;
float desiredYawSpeed;                         //Angular Yaw speed in Rad/s



//standard routine:
void setup()
{
  startupWireless();
  startupDMP();
  timeOfLastTransmission = micros();
}

void loop()     //pushes the drone towards the desired state from the actual state, those two are updated through interrupts
{

}


//interrupt functions:
void SerialEvent()                //Refreshes the global Variables that keep track of the desired drone state and updates the timer of when the last transmission took place.
{
  if (Serial.read() != '<') return;      //checks if its a desired input from us or junk from the WLAN-module
  timeOfLastTransmission = micros();
  while(!Serial.available());			 //wait for input.
  if (Serial.read() == '#') {                         //checks if its a command
    desiredYawSpeed = parseSerialPercentage() * (MAX_ANGLE_VELOCITY_YAW / 100);
    while(!Serial.available());			 //wait for input.
    Serial.read(); //remove the seperator "|"
    desiredVelocityZ = parseSerialPercentage() * (MAX_VELOCITY_Z / 100.0);
    while(!Serial.available());			 //wait for input.
    Serial.read(); //remove the seperator "|"
    desiredVelocityX = parseSerialPercentage() * (MAX_VELOCITY_X / 100.0);
    while(!Serial.available());			 //wait for input.
    Serial.read(); //remove the seperator "|"
    desiredVelocityY = parseSerialPercentage() * (MAX_VELOCITY_Y / 100.0);
  } else {
    
  }
}

void mpuInterrupt()               //Refreshes the global variables that tell the Drone its state. Gets executed everytime the mpu sends INT, thus everytime new data is ready.
{
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

//"normal" functions

void startupDMP()
{
  Wire.begin();                         //I2C startup
  Wire.setClock(400000);
  mpu.initialize();

  mpu.dmpInitialize();                  //loads firmware for the DMP if it seems to fail it can be checked through return value but for now I assume it wont fail. 0 means success

  mpu.setXGyroOffset(GYRO_OFFSET_X);    //set initial Offsets see comment at the constants
  mpu.setYGyroOffset(GYRO_OFFSET_Y);
  mpu.setZGyroOffset(GYRO_OFFSET_Z);
  mpu.setXAccelOffset(ACCEL_OFFSET_X);
  mpu.setYAccelOffset(ACCEL_OFFSET_Y);
  mpu.setZAccelOffset(ACCEL_OFFSET_Z);

  mpu.CalibrateAccel(6);                //TODO: find out what these two 6es do and demistify them
  mpu.CalibrateGyro(6);
  //mpu.printActiveOffsets();           //I leave this here for later to shed some light on the offset magic

  mpu.setDMPEnabled(true);

  pinMode(INTERRUPT_PIN, INPUT);        //sets the Pin INTERRUPT_PIN to INPUT mode
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), mpuInterrupt, RISING);  //enables the INT pin of the mpu to interrupt the Arduino to execute the function mpuInterrupt
  //mpu.getIntStatus(); //if you somehow need to find out the interrupt status of the mpu

  packetSize = mpu.dmpGetFIFOPacketSize();    //sets the Packet size Variable to later know how big the packages are

}

void startupWireless()
{
  Serial.begin(115200); //The WLAN module is connected via Serial
  Serial.println("AT+UART_CUR=9600,8,1,0,0"); //Tell it to Slow down
  Serial.begin(9600);   //Slow down yourself
  Serial.println("ATE0"); //switches echo (1: on 0: off)
  Serial.println("AT+CWMODE=3"); //set SoftAP + Station mode
  Serial.println("AT+CIPMUX=1");  //multiple connections needed for tcp server!!
  Serial.println("AT+CIPSERVER=1,100"); //start TCP server using port 100
  Serial.println("AT+CWSAP_CUR=\"" + (String) WLAN_SSID + "\",\"" + (String) WLAN_PASSWORD + "\"," + (String) WLAN_CHANNEL + ",3,1"); //setting SSID, Password, Channel, Encryption (3=WPA2_PSK), Only one user
}

char parseSerialPercentage()                //Parses the next 4 Characters in Serial Buffer based on the Rules listed at the beginning of this Program
{
  char percent = 0;
  while (Serial.available < 4);
  if (Serial.read() == '+') {
    percent += (Serial.read() - '0') * 100;
    percent += (Serial.read() - '0') * 10;
    percent += (Serial.read() - '0') * 1;
  } else {
    percent -= (Serial.read() - '0') * 100;
    percent -= (Serial.read() - '0') * 10;
    percent -= (Serial.read() - '0') * 1;
  }
  return percent;
}

void send(String string)
{
  Serial.println("AT+CIPSENDBUF=0," + String(string.length()));
  delay(2);//is needed for proper sending
  Serial.println(string);
}
