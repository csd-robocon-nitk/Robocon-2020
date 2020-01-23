///
//  Author : ibrahim - Original from2019 CSD Robocon
//  File for ESP8266 connected to SKPS and acting as
//  client connected to AP on Robot.
//  Reads PS2 states and sends joysticks values if R1
// is pressed.
//
// https://docs.google.com/document/d/1F2stWnGFJfb7ArR2sYMqamdiFjWAy4YZVkM9en5Gbx0/edit
///

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "ps2skps.h"

//  WiFi variables
const char *ssid = "controller";
const char *pass = "csdrobocon";
unsigned int localPort = 2000; // local port to listen for UDP packets
IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);

// UDP variables
WiFiUDP udp;
uint8_t packetBuffer[9];  //Where we get the UDP data

// SKPS variables
unsigned char *buffer;

//======================================================================
//                Setup
//======================================================================
void setup()
{
    WiFi.begin(ssid, pass);   //Connect to access point
    while (WiFi.status() != WL_CONNECTED);  // Wait for connection
    udp.begin(localPort); //Start UDP

    buffer = init_skps();
    skps_vibrate(p_motor2, 255);
    _delay_ms(1000);
    skps_vibrate(p_motor2, 0);
}

//======================================================================
//                MAIN LOOP
//======================================================================
void loop()
{
  skps(p_all, buffer);

  int cb = udp.parsePacket();
  if (!cb)
  {
    if (!((*(buffer + 1) >> 3) & 0x01)){  // Check bit 3 of 2nd byte in buffer
                                          // for R1 button. Refer SKSP docs
      udp.beginPacket(ServerIP, 2000);  //Send Data to Robot
                                        //Send UDP requests are to port 2000
      udp.write(buffer + 2, 4); //Send four bytes to Robot
      static uint8_t Checksum = *(buffer + 2) + *(buffer + 3) - *(buffer + 4)
                                - *(buffer + 5);
      udp.write(Checksum);
      udp.endPacket();
    }
  }
  else {
    // We've received a UDP packet
    udp.read(packetBuffer, 6); // read the packet into the buffer,
                               // we are reading 4 byte
  }
}
//=======================================================================
