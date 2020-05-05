#include "BluetoothSerial.h"



#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// WiFi network name and password:
const char * networkName = "greenio";
const char * networkPswd = "green7650";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "greenio.farm";
const int udpPort = 9956;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;


const String version = "RayongGWUDP120v2";
String rssi = "RSSI --";
String packSize = "--";
String packet;

unsigned int counter = 0;

bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.

long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends

// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI0     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    920E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true

uint16_t crcdata = 0;
uint16_t recCRCData = 0;

 
 
int count = 0;

String data = "";
String clientId = "";
 

SSD1306 display(0x3c, 4, 15);
BluetoothSerial SerialBT;

 
//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}


void logo(){
  display.clear();
  display.drawXbm(0,5,logo_width,logo_height,(const unsigned char *)logo_bits);
  display.display();
}
 
void setup()
{
	pinMode(25,OUTPUT);

	pinMode(16,OUTPUT);
	digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
	delay(50);
	digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

	display.init();
	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_16);
	logo();
	delay(300);
	display.clear();
Serial.begin(115200);

SerialBT.begin("ESP32test"); 
  while (!Serial); //test this program,you must connect this board to a computer
  Serial.println("ver");
  Serial.println(version);
  Serial.println("LoRa Receiver");
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
 
}

void onReceive(int packetSize)//LoRa receiver interrupt service
{
	//if (packetSize == 0) return;

	packet = "";
    packSize = String(packetSize,DEC);

    while (LoRa.available())
    {
    		packet += (char) LoRa.read();
    }

    Serial.println(packet);
    rssi = "RSSI: " + String(LoRa.packetRssi(), DEC);

    receiveflag = true;
}


uint16_t calcByte(uint16_t crc, uint8_t b)
{
  uint32_t i;
  crc = crc ^ (uint32_t)b << 8;

  for ( i = 0; i < 8; i++)
  {
    if ((crc & 0x8000) == 0x8000)
      crc = crc << 1 ^ 0x1021;
    else
      crc = crc << 1;
  }
  return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer, uint32_t length)
{
  uint16_t wCRC16 = 0;
  uint32_t i;
  if (( pBuffer == 0 ) || ( length == 0 ))
  {
    return 0;
  }
  for ( i = 0; i < length; i++)
  {
    wCRC16 = calcByte(wCRC16, pBuffer[i]);
  }
  return wCRC16;
}

uint16_t recdata( unsigned char* recbuf, int Length)
{
  crcdata = CRC16(recbuf, Length - 2); //Get CRC code
  recCRCData = recbuf[Length - 1]; //Calculate CRC Data
  recCRCData = recCRCData << 8; //
  recCRCData |= recbuf[Length - 2];
}
 

void loop()
{
  String data = "";
  int packetSize = LoRa.parsePacket();
  
  uint8_t newData[packetSize] = {}; //Store Sensor Data here

float tempInside = (temprature_sens_read() - 32) / 1.8;
  if (packetSize) {
    // received a packet
    Serial.println("Received packet '");
    // read packet
    uint8_t buf[packetSize];//receive data buffer
    int i = 0;
    while (LoRa.available()) {
      buf[i] = (byte)LoRa.read();
      i++;
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println(clientId);
    Serial.println(" ");
      recdata( buf, packetSize);
      Serial.print("Start CRC check "); Serial.println(packetSize);
      Serial.println(crcdata);
      Serial.println(recCRCData);
        Serial.println("end CRC check");  
  
        clientId = buf[4];

        for (int i = 0; i < packetSize; i++)
        {
          newData[i] = buf[i+4];
       
          Serial.print(newData[i]);
          Serial.print(" ");
        }
        //    }
        Serial.print("' with RSSI ");
    
        Serial.println(LoRa.packetRssi());
        if(buf[4] == 100 || buf[4] == 101 || buf[4] == 102 || buf[4] == 103 || buf[4] == 104 || buf[4] == 105 || buf[4] == 106 || buf[4] == 107 || buf[4] == 108 || buf[4] == 109 || buf[4] == 110 || buf[4] == 111 || buf[4] == 112 || buf[4] == 113 || buf[4] == 114 || buf[4] == 115 || buf[4] == 116 || buf[4] == 117 || buf[4] == 118 || buf[4] == 119 || buf[4] == 120){

          digitalWrite(25, HIGH);
          display.clear();
          display.drawString(0, 10, "Packet " + (String)(LoRa.packetRssi()) + "  done");
          display.drawString(0, 40, "no. " + (String) count++);
          display.drawString(0, 90, "id. " + buf[4]);
          display.drawString(50, 40, "wifi " + (String) WiFi.RSSI());
          Serial.print("WIFI.RSSI");
          Serial.println(WiFi.RSSI());
          SerialBT.print("WIFI.RSSI:");
          SerialBT.println(WiFi.RSSI());
          SerialBT.print("LoRa.packetRssi:");
          SerialBT.println(LoRa.packetRssi());
          display.display();
       //only send data when connected
          if(connected){
            //Send a packet
            data.concat("{\"Tn\":\"smf@ry");
            data.concat(clientId);
            data.concat("L\",");
            data.concat("\"aT\":");
            data.concat(newData[3]);
            data.concat(".");
            data.concat(newData[4]);
            data.concat(",\"aH\":");
            data.concat(newData[5]);
            data.concat(".");
            data.concat(newData[6]);
            data.concat(",\"sT\":");
            data.concat(newData[7]);
            data.concat(".");
            data.concat(newData[8]);
            data.concat(",\"sH\":");
            data.concat(newData[9]);
            data.concat(".");
            data.concat(newData[10]);
            data.concat(",\"l\":");            
            data.concat(word(newData[11],newData[12]));
            data.concat(",\"p\":");  
            data.concat(newData[13]);
            data.concat(newData[14]);
            data.concat(".");
            data.concat(newData[15]);
            data.concat(",\"v\":");            
            data.concat(newData[16]);
            data.concat(".");
            data.concat(newData[17]);            
            data.concat(",\"s\":");   
            data.concat(LoRa.packetRssi());  
            data.concat(",\"t\":");
            data.concat(tempInside);
            data.concat(",\"w\":");
            data.concat(WiFi.RSSI());           
            data.concat("}");
             
            SerialBT.println(data.c_str());
            Serial.println("sending");            
            Serial.println(data);
            
            udp.beginPacket(udpAddress,udpPort);
            //udp.printf("{\"Tn\":\"smf@ry103L\",\"aT\":1.2,\"aH\":3.4,\"sT\":5.6,\"sH\":7.8,\"l\":2314,\"p\":1112.13,\"v\":4.98,\"s\":-70}", millis()/1000);
            udp.printf(data.c_str(), millis()/1000);

            udp.endPacket();
          }
    }

    if(WiFi.RSSI() ==0)
      reboot();
  }
 
 
   
}

void reboot(){
  Serial.println("Restarting in 10 seconds");
 
  delay(10000);
 
  ESP.restart();  
  }
 
