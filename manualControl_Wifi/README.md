#### Two NodeMCU are used to act as WiFi bridge from wired PS2 to Arduino on Bot.  

__ps2NodeMCU.ino__ -> Code for NodeMCU connected to SKPS module. Uses header file ps2skps.h . Acts as WiFi client, connects to AP on Bot and sends data from the PS2 about the two joysticks.  
__botNodeMCU.ino__ -> Code for NodeMCU on the Bot connected to the Arduino. Acts as Access Point (AP) and receives data from the PS2. Sends this data to Arduino over Serial.  
__arduinoBotManual.ino__ -> Code for Arduino on the Bot. Uses wheelsDriver.h for controlling the omni wheels. Connected to botNodeMCU using Serial.  

__Wiring :__  
Arduino 5v to NodeMCU Vin  
          Arduino Gnd to NodeMCU Gnd  
          Arduino Rx to NodeMCU Tx (default pins only)  
(__Note:__ Arduino Tx need NOT be connected to NodeMCU Rx, as data is not transmitted back from Bot to PS2.)
