
/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using the CC3200 launchpad
 When a packet is received an Acknowledge packet is sent to the client on port remotePort


 created 30 December 2012
 by dlf (Metodo2 srl)
 
 modified 1 July 2014
 by Noah Luskey

 */

#ifndef __CC3200R1M1RGC__
// Do not include SPI for CC3200 LaunchPad
#include <SPI.h>
#endif
#include <WiFi.h>
#include <BMA222.h>
#include <Wire.h>


// your network name also called SSID
char ssid[] = "eittest";
// your network password
char password[] = "12345678";
IPAddress ip;
long rssi;

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "yesss it should be workin";       // a string to send back
BMA222 mySensor;
WiFiUDP Udp;

void setup() {
    mySensor.begin();
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.print("Starting network...");
  WiFi.beginNetwork(ssid, password);
  Serial.println("done.");
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:

  
  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nIP Address obtained");
  printWifiStatus();

  Serial.println("\nWaiting for a connection from a client...");
  Udp.begin(localPort);
}

void loop() {

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  //Udp.beginPacket("192.168.1.2", 42679);
  //  Udp.write(ReplyBuffer);
  //  Udp.endPacket();
  int8_t acclX = mySensor.readXData();
  int8_t acclY = mySensor.readYData();
  int8_t acclZ = mySensor.readZData();
    
  Serial.print("X: ");
  Serial.print(acclX);

  Serial.print(" Y: ");
  Serial.print(acclY);

  Serial.print(" Z: ");
  Serial.println(acclZ);
  while (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received

    int8_t acclX = mySensor.readXData();
    int8_t acclY = mySensor.readYData();
    int8_t acclZ = mySensor.readZData();
    
    Serial.print("After connection X: ");
    Serial.print(acclX);
    Serial.print(" Y: ");
    Serial.print(acclY);
    Serial.print(" Z: ");
    Serial.println(acclZ);
    
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.print("{ rssi: '");
    Udp.print(rssi);
    Udp.print("', ip: '");
    Udp.print(ip);
    Udp.print("', accl_x: '");
    Udp.print(acclX);
    Udp.print("', accl_y: '");
    Udp.print(acclY);
    Udp.print("', accl_z: '");
    Udp.print(acclZ);
    Udp.print("' }");
    Udp.endPacket();
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}




