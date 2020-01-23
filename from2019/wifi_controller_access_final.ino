#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define rx_pin 4
#define tx_pin 5
#define LEDPIN  2

unsigned long time_out = 1000;
SoftwareSerial controller(rx_pin, tx_pin);

uint8_t ps_data[6];
const char *ssid = "controller";
const char *pass = "csdrobocon";
unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress ServerIP(192, 168, 4, 1);
IPAddress ClientIP(192, 168, 4, 2);

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

char packetBuffer[9];




void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  controller.begin(9600);
//  Serial.println();
WiFi.softAP(ssid, pass);    //Create Access point
//
// Start UDP
Serial.println("PROGRAM: starting UDP");
  if (udp.begin(localPort) == 1)
  {
    Serial.println("PROGRAM: UDP started");
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
  }
 else
  {
    Serial.println("PROGRAM: UDP not started");
  }

}

void loop()
{
  send_command(31);
  bool k = readAllData();
  //print_data();
  int cb = udp.parsePacket();

  if (!cb)
  {
    //if there is no data to read
    udp.beginPacket(ClientIP, 2000);
    //Send UDP requests to port 2000
    udp.write(ps_data, 6); //Send six bytes to ESP8266
    udp.endPacket();
  }
  else {
    //if any UDP packet is received then display that on serial monitor,not really needed for the controller task
    udp.read(packetBuffer, 1); // read the packet into the buffer, we are reading only one byte
    Serial.print(packetBuffer);
  }
  delay(100);
}
//======================================================================

void print_data()
{
  for(int i = 0; i < 6; i++)
  {
    Serial.print(ps_data[i]);
    Serial.write('\t');
  }
  Serial.println(" ");
}

void send_command(uint8_t cmd)
{
  while (controller.available() > 0)
  {
    controller.read();
  }
  controller.write(cmd);
}

bool readAllData(void)
{
  unsigned long prev_ms = millis();
  while (controller.available() < 6)
  {
    if((millis() - prev_ms) > time_out)
    {
      return false;
    }
  }
  for (int i = 0; i < 6; i++)
  {
    ps_data[i] = controller.read();
  }
  return true;
}
