///
//
//
///

#define BUFFER_SIZE 6

#include "wheelsDriver.h"

uint8_t f_toggle = 0;
uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t) * BUFFER_SIZE)
memset(buffer, 127, BUFFER_SIZE);

void setup(){
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, (f_toggle++)%2);

  init_motors();
}

void loop(){
  //static int forwardV = ( 255 - buffer[0] ) - 127;
  //static int rightV = buffer[1] - 127;
  //static int rotClkwise = buffer[3] - 127;

  //motors(forwardV, rightV, rotClkwise);
  motors(( 255 - (int)buffer[0] ) - 127, (int)buffer[1] - 127, (int)buffer[3] - 127);

  if (Serial.available() >= 5){
    Serial.read(buffer, 5);
    //Checksum for security
    if (buffer[4] != (buffer[0] + buffer[1] - buffer[2] - buffer[3])){
      memset(buffer, 127, BUFFER_SIZE); //Reset the buffer
    }else{
      digitalWrite(LED_BUILTIN, (f_toggle++)%2);
    }
  }
  delay(500);
}
