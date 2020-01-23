Two NodeMCU are used to act as WiFi bridge from wired PS2 to Arduino on Bot.
ps2NodeMCU.ino -> Code for NodeMCU connected to SKPS module. Uses header file ps2skps.h . Acts as WiFi client, connects to AP on Bot and sends both data from the PS2 about the two joysticks.
botNodeMCU.ino -> Code for NodeMCU on the Bot connected to the Arduino. Acts as Access Point (AP) and receives data from the PS2. Sends this data to Arduino over Serial.
arduinoBotManual.ino -> Code for Arduino on the Bot. Uses wheelsDriver.h for controlling the omni wheels. Connected to botNodeMCU using Serial.

Wiring :  Arduino 5v to NodeMCU Vin
              Arduino Gnd to NodeMCU Gnd
              Arduino Rx to NodeMCU Tx (default pins only)
(Note: Arduino Tx need NOT be connected to NodeMCU Rx, as data is not transmitted back from Bot to PS2.)