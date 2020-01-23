//

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid = "controller";
const char *pass = "csdrobocon";
unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress ServerIP(192, 168, 4, 1);
IPAddress ClientIP(192, 168, 4, 2);

// UDP variables
WiFiUDP udp;
char packetBuffer[6];

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.softAP(ssid, pass); //Create Access point
  if (udp.begin(localPort)){ // Start UDP
    digitalWrite(LED_BUILTIN, HIGH);
    _delay_ms(1000);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void loop()
{
  static int cb = udp.parsePacket();
  if (!cb)
  {
    //if there is no data to read
  }
  else {
    //if UDP packet is received
    udp.read(packetBuffer, 5); // read the packet into the buffer
    Serial.write(packetBuffer, 5);
  }
}
//======================================================================
