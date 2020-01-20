
// Start - 850 , Kick - 205, End/Max - 140
#define BITS 10
#define LSB 
#define maxSpeed 255

int pins_MSB_2_LSB[] = {53,51,41,39,37,35,49,47,45,43};
int arr[]={1,2,4,8,16,32,64,128,256,512};
#define motorPWM 2
#define motorDir 22 
int result,state = 0,setpoint = 850;
int error = 0,lastError = 0,errorSum = 0;
float kp = 100,ki = 0 ,kd = 20;

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

void setup(){
  int i;
  for (i = 0;i<BITS;i++){
    pinMode(pins_MSB_2_LSB[i], INPUT_PULLUP);
  }
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  Serial.begin(9600);
  Serial.print("ready\n");
  calcPID(0);
}

void loop(){
  int rawValues[BITS];
  

   int j = BITS - 1;

  for (int i = 0;i<BITS;i++){ 
    int readVal;
    readVal = !digitalRead(pins_MSB_2_LSB[i]);
    rawValues[j] = readVal;
    j--;
  }
//  for(j = BITS -1; j >=0; j--){
//    Serial.print(rawValues[j]); 
//  }
//  
  greytobinary(rawValues, BITS);
//   Serial.print("\t");
//  for(j = BITS-1; j>=0; j--){
//    Serial.print(rawValues[j]); 
//  }
   
  result = binarytodecimal(rawValues, BITS);

  Serial.print("\nencoder: ");
  Serial.print(result);
  //delay(100);

    digitalWrite(motorDir, HIGH);
  if ((result < 520) && (result >300)){
    analogWrite(motorPWM, 0);
    while(1)
      Serial.print("\n Reached.");
  } 
  else{
   analogWrite(motorPWM, 255);
    
  }
  //calcPID(state);
}


int sign(int i) {
  if (i >= 0)
    return 1;
  else
    return -1;
}

void calcPID(int state)
{
  uint8_t Speed = 0;
  int dir = 0;
  
 // setpoint = (state==0) ?190 :(state == 1?370:300);
  error = (setpoint-result);
  Speed = error*kp/3550 + (lastError - error)*kd/200 + ki*errorSum/1000;
  errorSum += error;
  errorSum = (errorSum >= 1024)? 0:errorSum; 
  lastError = error;
  Serial.print("\t error: ");
  Serial.print(error);
  
  if(abs(Speed) > maxSpeed)
    Speed = sign(Speed) * maxSpeed;
    
  set_motor((-1)*Speed);
  Serial.print("\t speed: ");
  Serial.print(Speed);
}
void set_motor(int motor_speed) {
  if (abs(motor_speed) < 10)
    motor_speed = 0;
  else if (motor_speed >= 0) {
    //Clockwise direction
    digitalWrite(motorDir, HIGH);
    analogWrite(motorPWM, motor_speed);
  }
  else if (motor_speed < 0) {
    //Anti Clockwise direction
    digitalWrite(motorDir, LOW);
    analogWrite(motorPWM, -1 * motor_speed); 
  }
}
