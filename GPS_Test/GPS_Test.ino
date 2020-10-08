#include <SoftwareSerial.h>

SoftwareSerial NEO6M(7, 6);  //RX,TX

unsigned char CK_A; //Checksum A
unsigned char CK_B; //Checksum B

void sendData(unsigned char CLASS, unsigned char ID, unsigned short LENGTH, unsigned long PAYLOAD);
void calculateChecksum(unsigned char Buffer[], int Size);
String getGPS();

void setup() {
  NEO6M.begin(9600);
  Serial.begin(9600);
  while (!NEO6M.available()) {
    delay(10);
  }
  Serial.println(getGPS());
}

void loop() {
  
  delay(10);
  while (!NEO6M.available()) {
    delay(10);
  }
}

void sendData(unsigned char CLASS, unsigned char ID, unsigned short LENGTH, unsigned long PAYLOAD) {
  int Size = 1 + 1 + 2 + LENGTH; //CLASS + ID + LENGTH + PAYLOAD
  unsigned char Buffer[Size];
  Buffer[0] = CLASS;
  Buffer[1] = ID;
  Buffer[2] = (LENGTH << 8) >> 8;
  Buffer[3] = LENGTH >> 8;

  for (int i = 0; i < LENGTH; ++i) {
    Buffer[i + 4] = (PAYLOAD << (i * 8)) >> (LENGTH * 8);
  }

  calculateChecksum(Buffer, Size);
  
  NEO6M.write(0xB5); //Always this first Number 
  NEO6M.write(0x62); //Always this second Number 

  for (int i = 0; i < Size; ++i) {
    NEO6M.write(Buffer[i]);
  }

  NEO6M.write(CK_A);
  NEO6M.write(CK_B);

}

void calculateChecksum(unsigned char Buffer[], int Size) {
  CK_A = CK_B = 0;
  for (int i = 0; i < Size; ++i) {
    CK_A = CK_A + Buffer[i];
    CK_B = CK_B + CK_A;
  }
}

String getGPS() {
  String output = "";
  while(NEO6M.available()) {
    char c = NEO6M.read();
    output.concat(c);
  }
  return output;
}
