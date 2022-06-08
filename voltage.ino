#include <Arduino.h>
#include <SensorModbusMaster.h>
#include <FirebaseESP32.h>
#include <WiFiMulti.h>

// define available SSID & PASS
#define SSID1 "R13"
#define PASS1 "enerpe31"
#define SSID2 "add"
#define PASS2 "11223344"
#define SSID3 "Ular Sanca"
#define PASS3 "12093487"
#define SSID4 "AncolKentang"
#define PASS4 "ppqooqiiq123"

#define Firebase_Host "monitoring-daya-ppns-default-rtdb.asia-southeast1.firebasedatabase.app"
#define Firebase_Auth "QpcYZuWZUjfsoReNe8zU31BZOdg26fSEPQObsbns"
FirebaseData fbdo;
FirebaseJson fjson;
//initialize variabel and etc
WiFiMulti AP;
String path = "/ruangan2";
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
  setupWiFi();
  while(AP.run() != WL_CONNECTED) {
    Serial.print('.');
    
    delay(500);
  }
  Serial.println();
  Firebase.begin(Firebase_Host, Firebase_Auth);
  Serial.println("Firebase init");
  Firebase.reconnectWiFi(true);
}

void loop() {
  int16_t tegR = 0;
  int16_t tegS = 0;
  int16_t tegT = 0;
  int16_t arusR = 0;
  int16_t arusS= 0;
  int16_t arusT = 0;
  int16_t daya = 0;
  int32_t kwh = 0;
      
  tegR = modbus.uint16FromRegister(0x03, 0x00, bigEndian); //tegangan R
  tegS = modbus.uint16FromRegister(0x03, 0x01, bigEndian); //tegangan S
  tegT = modbus.uint16FromRegister(0x03, 0x02, bigEndian); //tegangan T
  arusR = modbus.uint16FromRegister(0x03, 0x03, bigEndian); //arus R
  arusS = modbus.uint16FromRegister(0x03, 0x04, bigEndian); //arus S
  arusT = modbus.uint16FromRegister(0x03, 0x05, bigEndian); //arus T
  daya = modbus.uint16FromRegister(0x03, 0x07, bigEndian); //daya
  kwh = modbus.uint32FromRegister(0x03, 0x1D, bigEndian); //kwh
  
  fjson.set("Timestamp/.sv", "timestamp");
  fjson.set("daya", daya);
  fjson.set("kwh", kwh);
  fjson.set("tegR", tegR);
  fjson.set("tegS", tegS);
  fjson.set("tegT", tegT);
  fjson.set("arusR", arusR);
  fjson.set("arusS", arusS);
  fjson.set("arusT", arusT);
  if(!Firebase.pushJSON(fbdo, path, fjson)) {
    Serial.printf("Error: %s\n", fbdo.errorReason().c_str());
  } else {
    Serial.println("Sukses");
  }
  
  Serial.print("Volt R-");
  Serial.print((double)tegR*0.1,1);
  Serial.print(" S-");
  Serial.print((double)tegS*0.1,1);
  Serial.print(" T-");
  Serial.println((double)tegT*0.1,1);
  Serial.print("Current R-");
  Serial.print((double)arusR*0.001,3);
  Serial.print(" S-");
  Serial.print((double)arusS*0.001,3);
  Serial.print(" T-");
  Serial.println((double)arusT*0.001,3);
  Serial.print("Daya ");
  Serial.println((double)daya,0);
  Serial.print("kWh ");
  Serial.println((double)kwh*0.01,2);

  delay(10UL * 60UL * 1000UL); //delay 10 menit
//  delay(100);
}

void setupWiFi(){
  AP.addAP(SSID1, PASS1);
  AP.addAP(SSID2, PASS2);
  AP.addAP(SSID3, PASS3);
  AP.addAP(SSID4, PASS4);
}
