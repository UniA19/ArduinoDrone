#include <SoftwareSerial.h>

String getOutput();
int i;

SoftwareSerial ESP8266(11, 10);  //RX,TX

void setup()
{
  Serial.begin(9600);
  ESP8266.begin(115200);
  while(!ESP8266);
  ESP8266.println("AT+UART_CUR=9600,8,1,0,0");
  getOutput();
  
  ESP8266.begin(9600);
  while(!ESP8266);
  ESP8266.println("AT+GMR");
  getOutput();
  
  ESP8266.println("ATE1");
  getOutput();
  
  ESP8266.println("AT+CWMODE=3");
  getOutput();

  ESP8266.println("AT+CIPMUX=1"); //max 1 connection
  getOutput();

  ESP8266.println("AT+CIPSERVER=1,100"); //start tcp server
  getOutput();

  ESP8266.println("AT+CWSAP_CUR=\"drone\",\"1234567890\",5,3");
  getOutput();
}

void loop()
{
  getOutput();
}

String getOutput()
{
  String output = "";
  delay(100);
  int i = 0;
  while(ESP8266.available()) {
    i = 1;
    char c = (char)ESP8266.read();
    output.concat(c);
    if (c == '>') break;
  }
  if (i == 1) {
    Serial.println(output);
  }
  return output;
}
