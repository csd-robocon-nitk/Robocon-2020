#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid = "controller";
const char *pass = "csdrobocon"; 

unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

uint8_t packetBuffer[9];//Where we get the UDP data
//======================================================================
//                Setup
//======================================================================
void setup()
{
    Serial.begin(9600);
    Serial.println();

    WiFi.begin(ssid, pass);   //Connect to access point
  
    Serial.println("");

  // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    //Start UDP
    Serial.println("PROGRAM: starting UDP");
    if(udp.begin(localPort) == 1)
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
//======================================================================
//                MAIN LOOP
//======================================================================
void loop()
{
    int cb = udp.parsePacket();
    if (!cb) 
    {
      //If serial data is recived send it to UDP
      if(Serial.available()>0)
        {
        udp.beginPacket(ServerIP, 2000);  //Send Data to Master unit
        //Send UDP requests are to port 2000
        
        char a[1];
        a[0]=char(Serial.read()); //Serial Byte Read
        udp.write(a,1); //Send one byte to ESP8266 
        udp.endPacket();
        }
    }
    else {
      // We've received a UDP packet, send it to serial
      udp.read(packetBuffer, 6); // read the packet into the buffer, we are reading 4 byte
      print_data();
      delay(100);
    }
}
void print_data()
{
  for(int i=0;i<6;i++)
  {
    Serial.print(packetBuffer[i]);
    Serial.print(" ");
   }
   Serial.println();
      
}
//=======================================================================
