// LSA08 USER MANUAL: https://drive.google.com/file/d/0BzFWfMiqqjyqXzFaMTNHMy1pWUE/view

#define DELAY 50

#define max_speed 1.2

#define comp_speed 0.5 //ultra slow compenstaion speed to bring bot to halt in throwing gray area
#define low_speed 0.6
#define medium_speed 1.0
#define high_speed  1.1
#define ultra_speed 1.5

#define control_speed 1.5

uint8_t led_pin = 25;
uint8_t buzzer_pin = 27;

boolean line = 1;
boolean wheel = 0;
boolean junction_flag = 0;

/*  'dir' values:
   0  - Stop
   1  - Front
  -1  - Back
   2  - Left
  -2  - Right
*/

int dir_cmd[14] = { -1, -2, -1, 2, -1, -2, -1, 2, -1, -1, -2, -2, 2, 1}; //defining the directions corresponding to each corner index
//int dir_cmd[14] = {1, 2, 1, -2, 1, 2, 1, -2, 1, 1, 2, 2, -2, -1};  //defining the directions corresponding to each corner index
int corner_index = 0, prev_corner_index = 0;

//int correction[15] =   {35, 35, 35, 35, 35, 35, 35, 35, 35, 55, 35, 55, 35, 55, 35};

int noline[4] = {0, 0, 0, 0};  // 0 -> Line detected, 1 -> No Line detected
int dir = 0;
int jun_dir = 1;
unsigned long int pres_ms = 0, prev_ms = 0, jun_time = 250, present_ms = 0, previous_ms = 0, last_ms = 0, delta_t = 0;

float base_speed = 0.6;
float J[4][3] = {{13.1579, -13.1579, -8.4211}, {13.1579, 13.1579, 8.4211}, {13.1579, 13.1579, -8.4211}, {13.1579, -13.1579, 8.4211}};
/*
  Conversion Matrix (J)
  13.1579 -13.1579 -8.4211
  13.1579  13.1579  8.4211
  13.1579  13.1579 -8.4211
  13.1579 -13.1579  8.4211
*/
float Vsp[3] = {0, 0, 0};
//Vmax = 3.725
//Vx,Vy,W
float max_div = 255.0, max_rpm = 468.0;
float conv_factor[4] = {0.1334, 0.3667, 0.8, 2.6};
float pi = 3.1416;
float w[4] = {0, 0, 0, 0};
/*
  0 - FL
  1 - FR
  2 - BL
  3 - BR
*/

////////////////////////////////////////////////////////////////////
////Sensor Pins
//uint8_t ser_enable[4] = {22, 30, 38, 46};   //F B L R
//uint8_t Jpulse[4] = {24, 32, 40, 48};       //F B L R
//
////Motor Pins
//uint8_t motor_pin[4] = {6, 3, 12, 9};
//uint8_t dir_pin[4] = {5, 2, 11, 8};
/////////////////////////////////////////////////////////////////////

//Sensor Pins
uint8_t ser_enable[4] = {22, 30, 38, 46};   //F B L R
uint8_t Jpulse[4] = {24, 32, 40, 48};       //F B L R

//Motor Pins
uint8_t motor_pin[4] = {3, 6, 9, 12};    // FL FR BL BR
uint8_t dir_pin[4] = {2, 5, 8, 11};
/*
  Pin mappings to motors
  0 - front left motor
  1 - front right motor
  2 - rear left motor
  3 - rear right motor
*/

//Sensor Data
char address = 0x00;
char sensor_addr[4] = {0x01, 0x02, 0x03, 0x04};
int sensor_data[4] = {0, 0, 0, 0};
boolean jun_data[4] = {0, 0, 0, 0};
int jun_sum = 0;
//Sensor Settings
char contrast = 100; //0 - 255
char brightness = 0x04; //0 to 10
char junction_width = 0x04; //0x02 to 0x08
char threshold = 0x03; //0 to 7
char line_mode = 0x00; //Light on Dark Background
char UART_Mode = 0x02; //byte of analog Value

/*
  Left Line Following
  Kp and Kd 0.045
*/
//PID Variables
//float Kp[3] = {0.0065, 0.0065, 0.01}, Kd[3] = {0.012, 0.012, 0.012}, Ki[3] = {0};
float Kp[3] = {0.015, 0.015, 0.02}, Kd[3] = {0.03, 0.03, 0.03}, Ki[3] = {0};
float P[3] = {0, 0}, I[3] = {0, 0}, D[3] = {0, 0};
float PID[3] = {0, 0};
float error[3] = {0, 0};
float last_error[3] = {0, 0};
float set_position = 35;

/*
  Index
  0     -     Linear Velocity PID
  1     -     Angular Velocity PID
*/

//Wheel PID Variables

float max_vel = 400;
//float wheel_Kp[4] = {9, 9, 9, 9}, wheel_Kd[4] = {100, 100, 100, 100};
////float wheel_Kp[4] = {12, 12, 12, 12}, wheel_Kd[4] = {150, 150, 150, 150};
//float wheel_Ki[4] = {0, 0, 0, 0};
//float wheel_P[4] = {0, 0, 0, 0}, wheel_I[4] = {0, 0, 0, 0}, wheel_D[4] = {0, 0, 0, 0};

//Controller Variables

float data[6] = {132, 132, 132, 132, 132, 132};
String response = " ";
void setup()
{
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();
  Serial2.begin(115200);
  Serial1.begin(9600);

  init_indicators();
  init_motors();
  init_sensors();
  //calibrate();    //uncomment to calibrate the LSA08 sensors
  for (int i = 0; i < 20; i++)  // Loop of 20 iterations to neglect the initial zero values of the LSA08 readings
  {
    read_sensors();
  }
}

void loop()
{

  //  while(corner_index == 12)
  //  {
  //    control();
  //    Vsp[1] = (data[2] - 132) * control_speed / 128;
  //    Vsp[0] = (data[3] - 132) * control_speed / 128;
  //    Vsp[2] = (data[4] - 132) * control_speed / 128;
  //
  //    matrix_mult();
  //    motors(w);
  //  }
  base_speed =  assign_base_speed(corner_index); //Assigning different base speeds for different portions of the arena
  //  if (corner_index == 12 || corner_index == 13)
  // base_speed = low_speed;


  if (corner_index >= 13)    //When the bot reaches the T junction corresponding to the throwing gray area
  {
    dir = 1; //move the bot backwards
    int jun = 0;  //to count the number of junctions passed after the T junction
    base_speed = medium_speed;
    while (1)
    {
//      Serial.print(dir);
//      Serial.print(" ");
//      Serial.println("Full Speed");
      
      if (jun_data[1] == 1)
      {
        jun++;  //increment junction counter when BACK LSA08 detects a junction
        if(jun == 1)
        {
          base_speed = ultra_speed;
        }
        while (jun_data[1] != 0)
        {
          read_sensors();
          cal_error();
          cal_PID();
          set_Vsp();
          matrix_mult();
          motors(w);
        }
      }

      if (jun_data[0] == 1 && jun == 4)
        base_speed = comp_speed;         //slowing down the bot when FRONT LSA08 detects 3rd junction

      if (jun == 4)
      {
        //tone(buzzer_pin, 200, 1000);   //buzzer plays when last junction is reached
        jun_PID();      // Junction PID at the last junction
      }
      read_sensors();
      cal_error();
      cal_PID();
      set_Vsp();
      matrix_mult();
      motors(w);
    }
  }

  dir = dir_cmd[corner_index];  //obtaining 'dir' value of the bot based on the corner index mapping
  prev_corner_index = corner_index;
  Serial.print(corner_index);
  Serial.print(" ");
  Serial.println(dir);
  while (1)
  {
    read_sensors();
    if (jun_data[det_sens_dir(dir)])
    {
      while (jun_data[det_sens_dir(dir)])
      {
        read_sensors();
        sensor_data[det_sens_dir(dir_cmd[corner_index])] = 35;
        cal_error();
        cal_PID();
        set_Vsp();
        matrix_mult();
        motors(w);
      }
      corner_index++;
      if ((corner_index != 9) &&  (corner_index != 11))
        base_speed = low_speed;   //slowing down the bot until its centre is aligned with the junction
      else
        base_speed = medium_speed;
    }
    cal_error();
    cal_PID();
    set_Vsp();
    matrix_mult();
    motors(w);

    // 'Blinding' the LSA08 opposite to the primary LSA08 to make it think it is on the line
    sensor_data[det_opp_sens(det_sens_dir(dir_cmd[corner_index]))] = 35;

    while (corner_index != prev_corner_index)
    {
      read_sensors();
      if (corner_index == 10)
      {
        base_speed = 0.2;
      }
      else if (corner_index == 1)
      {
        base_speed = 0.3;
      }
      else if (corner_index < 7 || corner_index > 11 || corner_index == 13)
      {
        base_speed = 0.15;
      }
      else if (corner_index == 7)
      {
        base_speed = 0.5;
      }
      else
      {
        base_speed = 0.9;
      }
      if (!noline[det_sens_dir(dir_cmd[corner_index] )])
      {
        //Breaks from while loop if 'det_sens_dir(dir_cmd[corner_index]' LSA08 detects line
        break;
      }
      // 'Blinding' the primary LSA08 of prev corner_index dir to make it think it is on the line
      sensor_data[det_sens_dir(dir_cmd[corner_index - 1])] = 35;
      cal_error();
      cal_PID();
      set_Vsp();
      matrix_mult();
      motors(w);
    }

    if (corner_index != prev_corner_index && !(corner_index == 9 || corner_index == 11))
    {
      last_ms = millis();
      while ((millis() - last_ms) < jun_time)
      { //stopping the bot at the junction for 'jun_time' milliseconds (except for corners 9 and 11)
        read_sensors();
        Vsp[0] = 0;
        Vsp[1] = 0;
        Vsp[2] = 0;
        matrix_mult();
        motors(w);
      }
      break;
    }
  }
}

void control()
{
  if (check_serial())
  {
    parse_response();
  }
}

void parse_response()
{
  int l = response.length(), k = 0;
  int limits[200];
  for (int i = 0; i < l; i++)
  {
    if (response[i] == '#')
    {
      limits[k] = i + 1;
      k++;
    }
  }
  for (int i = 0; i < 6; i++)
  {
    String temp = response.substring(limits[i], limits[i + 1] - 1);
    data[i] = temp.toInt();
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

boolean check_serial()
{
  if (Serial2.available() > 0)
  {
    response = Serial2.readStringUntil('\n');
    if (response[0] == '#')
    {
      return true;
    }
  }
  return false;
}

float assign_base_speed(int corner_index)
{
  //Note: 0.7  is the max reliable base speed but overshooting may occur on certain trials
  if (corner_index < 8)
  {
    if (corner_index == 0)
      return medium_speed;
    else if ((corner_index % 2) == 0)
      return medium_speed;   // return high_speed;
    else if (corner_index == 7)
      return comp_speed;
    else
      return medium_speed;
  }
  else
  {
    if (corner_index >= 13)
      return ultra_speed;
    else
      return medium_speed;  //return high_speed;
  }
}

int det_opp_sens(int front)
{ //determining LSA08 number opposite to that of the 'dir'
  /*
    1(front) -> 1
    -1(back)  -> 0
    2(left)  -> 3
    -2(right) -> 2
  */
  switch (front)
  {
    case 0:
      {
        return 1;
      }
      break;
    case 1:
      {
        return 0;
      }
      break;
    case 2:
      {
        return 3;
      }
      break;
    case 3:
      {
        return 2;
      }
      break;
  }
}

int det_sens_dir(int dir)
{ //determining primary LSA08 number along that of the 'dir'
  /*
    1(front) -> 0
    -1(back)  -> 1
    2(left)  -> 2
    -2(right) -> 3
  */
  switch (dir)
  {
    case 1:
      {
        return 0;
      }
      break;
    case -1:
      {
        return 1;
      }
      break;
    case 2:
      {
        return 2;
      }
      break;
    case -2:
      {
        return 3;
      }
      break;
  }
}

void set_Vsp()
{
  switch (abs(dir))
  {
    case 1:
      if (dir > 0)
      {
        Vsp[0] = base_speed;
        Vsp[1] = -1 * PID[0];
        Vsp[2] = -1 * PID[2];
      }
      else
      {
        Vsp[0] = -1 * base_speed;
        Vsp[1] = -1 * PID[0];
        Vsp[2] = -1 * PID[2];
      }
      break;
    case 2:
      if (dir > 0)
      {
        Vsp[0] = PID[1];
        Vsp[1] =  base_speed;
        Vsp[2] = -1 * PID[2];
      }
      else
      {
        Vsp[0] = PID[1];
        Vsp[1] = -1 * base_speed;
        Vsp[2] = -1 * PID[2];
      }
      break;
    default:
      Vsp[0] = 0;
      Vsp[1] = 0;
      Vsp[2] = 0;
  }
  if (dir == 0)
  {
    base_speed = 0;
    Vsp[0] = PID[1];
    Vsp[1] = -1 * PID[0];
    Vsp[2] = -1 * PID[2];
  }
  else {
    //base_speed = follow_speed;
  }
}

void cal_error()
{
  float temp = 0;
  temp = (sensor_data[0] - set_position) - (sensor_data[1] - set_position);
  error[0] = temp / 2;
  temp = (sensor_data[2] - set_position) - (sensor_data[3] - set_position);
  error[1] = temp / 2;
  if (abs(dir) == 1 || abs(dir) == 0)
  {
    temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
    error[2] = temp / 2;
  }
  if (abs(dir) == 2)
  {
    temp = (sensor_data[2] - set_position) + (sensor_data[3] - set_position);
    error[2] = temp / 2;
  }
}

void cal_PID()
{
  int i;
  for (i = 0; i < 3; i++)
  {
    P[i] = error[i];
    D[i] = error[i] - last_error[i];

    PID[i] = (Kp[i] * P[i]) + (Kd[i] * D[i]) + (Ki[i] * I[i]);
    last_error[i] = error[i];
    if (abs(PID[i]) > max_speed)
    {
      if (PID[i] > 0)
        PID[i] = max_speed;
      else if (PID[i] < 0)
      {
        PID[i] = -1 * max_speed;
      }
    }
  }
}

void motors(float motor_speed[4])
{
  int i;
  for (i = 0; i < 4; i++)
  {
    set_motor(i, motor_speed[i]);
  }
}

void matrix_mult()
{
  float sum = 0;
  int i, j;
  for (i = 0; i < 4; i++)
  {
    sum = 0;
    for (j = 0; j < 3; j++)
    {
      sum += (J[i][j] * Vsp[j]);
    }
    w[i] = sum;
    w[i] = w[i] * 60 / (2 * pi); //rps to RpM
    if (abs(w[i]) > max_vel)
    {
      if (w[i] > 0)
      {
        w[i] = max_vel;
      }
      else
      {
        w[i] = -1 * max_vel;
      }
    }

    if (abs(w[i]) > 255)
    {
      if (w[i] > 0)
      {
        w[i] = 255;
      }
      else
      {
        w[i] = -255;
      }
    }
  }
}



void reinit_sensor(int dir) {
  int junction_pull = 25; // 0 to 35
  switch (dir) {
    case 1:
      sensor_data[2] = 70;//+junction_pull;//70
      sensor_data[3] = 0;//-junction_pull;//0
      break;
    case 2:
      sensor_data[0] = 0;// - junction_pull;
      sensor_data[1] = 70;// + junction_pull;
      break;
    case -2:
      sensor_data[0] = 70;// + junction_pull;
      sensor_data[1] = 0;// - junction_pull;
      break;
    case -1:
      sensor_data[2] = 0;//-junction_pull;
      sensor_data[3] = 70;//+junction_pull;
      break;

  }
}

void read_sensors()
{
  int temp = 0, i;
  jun_sum = 0;
  for (i = 0; i < 4; i++)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    temp = Serial3.read();
    if (temp <= 70)
    {
      sensor_data[i] = temp;
      noline[i] = 0;
    }
    else
    {
      noline[i] = 1;
    }
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for (i = 0; i < 4; i++)  {
    jun_data[i] = digitalRead(Jpulse[i]);
    jun_sum += jun_data[i];
  }
}

void calibrate()
{
  int i;
  for (i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    send_command('C', 0x00);
    delay(DELAY);
  }
}

void set_sensor_settings()
{
  // Clear internal junction count of LSA08
  send_command('X', 0x00);
  delay(DELAY);
  // Setting LCD contrast
  send_command('S', contrast);
  delay(DELAY);
  // Setting LCD backlight
  send_command('B', brightness);
  delay(DELAY);
  // Setting junction width
  send_command('J', junction_width);
  delay(DELAY);
  //  send_command('T', threshold);
  //  delay(DELAY);
  //   Setting line mode
  send_command('L', line_mode);
  delay(DELAY);
  // Setting UART ouput Mode
  send_command('D', UART_Mode);
  delay(DELAY);
}

void init_sensors()
{
  Serial3.begin(9600); // Start Serial3 communication
  Serial3.flush();   // Clear Serial3 buffer
  int i;
  for (i = 0; i < 4; i++)
  {
    // pinMode(Jpulse[i], INPUT);
    pinMode(ser_enable[i], OUTPUT);
    digitalWrite(ser_enable[i], HIGH);
  }
  delay(DELAY);
  for (i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    set_sensor_settings();
  }
}

void send_command(char command, char data) {

  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);

}
void set_motor(uint8_t index, int motor_speed)
{
  if (index % 2 == 0)
    motor_speed = -1 * motor_speed;
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  if (motor_speed >= 0)
  {
    //clock wise direction
    digitalWrite(dir_pin[index], HIGH);
    analogWrite(motor_pin[index], motor_speed);
  }
  else if (motor_speed < 0)
  {
    //anti clock wise direction
    digitalWrite(dir_pin[index], LOW);
    analogWrite(motor_pin[index], -1 * motor_speed);
  }

}

void jun_PID()
{
  reinit_sensor(dir);
  dir = 0;
  int temp = 0;
  prev_ms = millis();
  while (1)
  {
    read_sensors();
    cal_error();
    cal_PID();
    set_Vsp();
    matrix_mult();
    motors(w);
  }
}

void init_indicators()
{
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);
}

void init_motors()
{
  int i;
  for (i = 0; i < 4; i++)
  {
    pinMode(motor_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
  }
}
