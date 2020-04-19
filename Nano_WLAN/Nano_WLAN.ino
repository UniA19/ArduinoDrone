#include <Wire.h>
#include <math.h>
#include <SoftwareSerial.h>

String getWLAN();
void getGyro();
void calibrateGyro();
void printGyro();

SoftwareSerial ESP8266(11, 10);  //RX,TX
const int MPU=0x68; 
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXOff, AcYOff, AcZOff, TmpOff, GyXOff, GyYOff, GyZOff;

void setup()
{
  Serial.begin(9600);
  
  //Setup WLAN
  ESP8266.begin(115200);
  while(!ESP8266);
  /*ESP8266.println("AT+RESTORE");//restore all defaults
  delay(1000);
  getWLAN();*/
  
  ESP8266.println("AT+UART_CUR=9600,8,1,0,0");//change serial connection
  delay(100);
  getWLAN();
  
  ESP8266.begin(9600);
  while(!ESP8266);
  ESP8266.println("AT+GMR"); //checks version information
  delay(100);
  getWLAN();
  
  ESP8266.println("ATE0"); //switches echo (1: on 0: off)
  delay(100);
  getWLAN();
  
  ESP8266.println("AT+CWMODE=3"); //set SoftAP + Station mode
  delay(100);
  getWLAN();
  
  ESP8266.println("AT+CIPMUX=1"); //multiple connections needed for tcp server!!
  delay(100);
  getWLAN();
  
  ESP8266.println("AT+CIPSERVER=1,100"); //start tcp server, port
  delay(100);
  getWLAN();
  
  ESP8266.println("AT+CWSAP_CUR=\"drone\",\"1234567890\",5,3,1"); //set ssid, password, channel, encryption (3: WPA2_PSK)
  delay(100);
  getWLAN();
  
  //Setup Gyro
  /*Wire.begin(); //Needs SCL -> A5  SDA -> A4
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  calibrateGyro();*/
}

void loop()
{
  getWLAN();
  delay(10);
}

String getWLAN()
{
  String output = "";
  while(ESP8266.available()) {
    char c = (char)ESP8266.read();
    while (c == '<') {
      while(!ESP8266.available()) delay(1);
      char d = (char)ESP8266.read();
      if (d == '>') {
        send(output);
        Serial.println(output);
        break;
      }
      output.concat(d);
    }
  }
  return output;
}

void send(String string)
{
  ESP8266.println("AT+CIPSENDBUF=0," + String(string.length()));
  delay(2);//is needed for proper sending
  ESP8266.println(string);
}

void getGyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcX = (Wire.read() << 8 | Wire.read()) - AcXOff;
  AcY = (Wire.read() << 8 | Wire.read()) - AcYOff;
  AcZ = (Wire.read() << 8 | Wire.read()) - AcZOff;
  Tmp = (Wire.read() << 8 | Wire.read()) - TmpOff;
  GyX = (Wire.read() << 8 | Wire.read()) - GyXOff;
  GyY = (Wire.read() << 8 | Wire.read()) - GyYOff;
  GyZ = (Wire.read() << 8 | Wire.read()) - GyZOff;

  Serial.println();
}

void calibrateGyro()
{
  getGyro();
  AcXOff += AcX;
  AcYOff += AcY;
  AcZOff += AcZ;
  TmpOff += Tmp;
  GyXOff += GyX;
  GyYOff += GyY;
  GyZOff += GyZ;

  Serial.println("Calibrated Gyro");
}

void printGyro()
{
  getGyro();
  
  Serial.print("AcX: "); Serial.println(AcX);
  Serial.print("AcY: "); Serial.println(AcY);
  Serial.print("AcZ: "); Serial.println(AcZ);
  Serial.print("Tmp: "); Serial.println(Tmp);
  Serial.print("GyX: "); Serial.println(GyX);
  Serial.print("GyY: "); Serial.println(GyY);
  Serial.print("GyZ: "); Serial.println(GyZ);
}
