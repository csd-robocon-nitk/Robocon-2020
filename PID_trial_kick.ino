#include "PinChangeInterrupt.h"
#define BITS 10
#define LSB 
#define max_speed 30

int pins_MSB_2_LSB[] = {53,51,41,39,37,35,49,47,45,43};
int arr[]={1,2,4,8,16,32,64,128,256,512};

uint8_t motor_pin = 2;
uint8_t dir_pin = 22;

float Kp = 0.1, Ki = 0, Kd = 10;
float P = 0, I = 0, D = 0, PID =0;
float error =0;
float last_error = 0, set_position = 512;

int encoderValue = 0;

void setup(){
  int i;
  for (i = 0;i<BITS;i++){
    pinMode(pins_MSB_2_LSB[i], INPUT_PULLUP);
  }
  
  pinMode(motor_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  Serial.begin(9600);
  Serial.print("ready\n");
}

void loop(){
  encoder_val();
  cal_PID();
  set_motor(PID);
}

void greytobinary(int *a, int count)
{
  int i;  
  for(i = count - 2; i >= 0; i--)
  {
      a[i] = a[i]^a[i+1];
  }
}

int binarytodecimal(int *a, int count)
{
  int sum=0,i;
  for(i= 0;i < BITS;i++)
  sum = sum + (arr[i]*a[i]);
        return sum;
}

void encoder_val(){
  int rawValues[BITS];
  int i, j;

  j = BITS - 1;

  for (i = 0;i<BITS;i++){ 
    int readVal;
    readVal = !digitalRead(pins_MSB_2_LSB[i]);
    rawValues[j] = readVal;
    j--;
  }
  //for(j = BITS -1; j >=0; j--){
    //Serial.print(rawValues[j]); 
  //}
  
  greytobinary(rawValues, BITS);
  // Serial.print("\t");
  //for(j = BITS-1; j>=0; j--){
    //Serial.print(rawValues[j]); 
  //}
   
  encoderValue = binarytodecimal(rawValues, BITS);

  
  Serial.println(encoderValue);
  Serial.print("\t");
  delay(10);
}

void set_motor(int motor_speed) {
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  else if (motor_speed >= 0) {
    //Clockwise direction
    digitalWrite(dir_pin, HIGH);
    analogWrite(motor_pin, motor_speed);
  }
  else if (motor_speed < 0) {
    //Anti Clockwise direction
    digitalWrite(dir_pin, LOW);
    analogWrite(motor_pin, -1 * motor_speed);
  }
}


int sign(int i) {
  if (i >= 0)
    return 1;
  else
    return -1;
}

void cal_PID(){
      error = set_position - encoderValue;
      //error = error - correction;
      P = error;
      I += error;
      D = (-1)*(error - last_error);
      if(abs(P) < 30)
        P = 0;
      if(abs(D) < 20)
        D = 0;            
      PID = (-1)*((Kp * P) + (Ki * I) + (Kd * D));
      last_error = error;
 
      if (abs(PID) > max_speed)
        PID = sign(PID) * max_speed;
      
      Serial.println(PID);
}
