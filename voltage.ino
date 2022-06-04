#include <Arduino.h>
#include <SensorModbusMaster.h>

//Modbus
byte modbusAddress = 0x01;
long modbusBaudRate = 9600;

const int adapterPwrPin = 4;
const int DEREPin = 2;

#if defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
const int SSRxPin = 19;
const int SSTxPin = 18;
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);
#else
HardwareSerial* modbusSerial = &Serial2;
#endif

modbusMaster modbus;

//Koneksi
//#define Firebase_Host "monitoring-daya-ppns-default-rtdb.asia-southeast1.firebasedatabase.app"
//#define Firebase_Auth "QpcYZuWZUjfsoReNe8zU31BZOdg26fSEPQObsbns"

//Delay
unsigned long delayStart = 0;
bool delayRunning = false;

void setup() {
  
  if(DEREPin >= 0){
    pinMode(DEREPin, OUTPUT);
  }
  if(adapterPwrPin >= 0){
    pinMode(adapterPwrPin, OUTPUT);
    digitalWrite(adapterPwrPin, HIGH);
  }

  Serial.begin(57600);

  #if defined(ARDUINO_AVR_UNO)
  modbusSerial.begin(modbusBaudRate);
  #else
  Serial2.begin(modbusBaudRate, SERIAL_8E1);
  #endif

  modbus.begin(modbusAddress, modbusSerial, DEREPin);
}

void loop() {
  int16_t voltage = 0;
  int32_t power = 0;
  int16_t arus = 0;
      
  voltage = modbus.uint16FromRegister(0x03, 0x00, bigEndian); //tegangan
  power = modbus.uint16FromRegister(0x03, 0x1D, bigEndian); //daya
  arus = modbus.uint16FromRegister(0x03, 0x03, bigEndian); //arus
      
  Serial.print("Volt ");
  Serial.println((double)voltage*0.1,1);
  Serial.print("Power ");
  Serial.println((double)power*0.01,2);
  Serial.print("Current ");
  Serial.println((double)arus*0.001,3);

  delay(10UL * 60UL * 1000UL);
}
