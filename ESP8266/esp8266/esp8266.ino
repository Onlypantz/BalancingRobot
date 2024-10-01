#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
//#include "DHT.h"
//#define DHTPIN 2 // what digital pin we're 
//#define DHTTYPE DHT11 // DHT 11
//DHT dht(DHTPIN, DHTTYPE);

//These are the avariables you can change for your project. For example, change the SSID (network ID), change the ports on both machines.
//The most useful one is the LOOP_DELAY which changes how quicly it refreshes. set to 100ms be default.
//Sets up the wifi components.
String nd="00",doam="00";
//int t_tam=0,h_tam=0,h=0,t=0;
char ssid[] = "ROBOT";      //your network SSID (name)
char password[] = "";  //Set the AP's password
unsigned int localPort = 2222;        // local port to listen on
char remoteIp[] = "192.168.4.1";      //IP address of computer end - the AP is always this by default
int remoteUDPPort =  2223;            //Port number on computer end
#define LOOP_DELAY 10                //how fast we check for new data
#define ARRAYSIZE 255                 //size of message array (255bytes)


//global variables
char packetBuffer[ARRAYSIZE]; //buffer to hold incoming packet
int arraySize = ARRAYSIZE;
char inData[ARRAYSIZE]; // Allocate some space for the string
char inChar; // Where to store the character read
byte aindex = 0; // Index into array; where to store the character
boolean dataToSend = false;
WiFiUDP Udp;
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
 
  Serial.println(myIP);
  Udp.begin(localPort);
//  dht.begin();
}



void loop()
{
  //check for incoming data via UDP
  int packetSize = Udp.parsePacket();
  if (Udp.available()>0)
  {    
    //Serial.println(packetSize);
    //int len = Udp.read();  //read in the packet
    for(int i=0;i<packetSize;i++)Serial.write(Udp.read());//print
    Serial.println();
    //Serial.write(packetBuffer);

//        WiFiClient::stopAll();
//        WiFiUDP::stopAll();
   
  }

 /* h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
 // Read temperature as Fahrenheit (isFahrenheit = true)
   if (isnan(h) || isnan(t)) {
       nd=(String) t_tam;
       doam=(String) h_tam;
   }
   else{
       nd=(String) t;
       doam=(String) h;
       if(t != t_tam|| h != h_tam){
           t_tam=t;
           h_tam=h;
           Serial.print(packetBuffer+nd+doam);
       }
   }*/
  // send UDP Data - this sends data to the device  at the top of the code.
 /*Udp.beginPacket(remoteIp, remoteUDPPort);
 Udp.write(msg_count++); //add an incrementing message number for sync
 if (msg_count > 100){msg_count = 0;} 
 Udp.write("Hi!");
 Udp.endPacket(); */
}
