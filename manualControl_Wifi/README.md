#### Two NodeMCU are used to act as WiFi bridge from wired PS2 to the Arduino MEGA on Bot.  

__ps2NodeMCU.ino__ -> Code for NodeMCU connected to SKPS module. Uses header file ps2skps.h . Acts as WiFi client, connects to AP on Bot and sends data from the PS2 about the two joysticks, if R1 is pressed.  
__botNodeMCU.ino__ -> Code for NodeMCU on the Bot connected to the Arduino. Acts as Access Point (AP) and receives data from the PS2. Sends this data to Arduino over Serial.  
__arduinoBotManual.ino__ -> Code for Arduino on the Bot. Uses wheelsDriver.h for controlling the omni wheels. Connected to botNodeMCU using Serial. The onboard LED should toggle to signal the receive of a message from the PS2.  

__Wiring :__  
Using Arduino MEGA with NodeMCU on Bot  
Arduino Gnd to NodeMCU Gnd  
Arduino Rx1(pin 19) to NodeMCU Tx(pin labelled TX on board)  
(__Note:__ Arduino Tx1 need NOT be connected to NodeMCU Rx, as data is not transmitted back from Bot to PS2.)
