#define max_speed 150

uint8_t motor_pin = 2;
uint8_t dir_pin = 22;


void setup() {
  // put your setup code here, to run once:
   pinMode(motor_pin, OUTPUT);
   pinMode(dir_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  set_motor(255);
  delay(100);
}


void set_motor(int motor_speed) {
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  if (motor_speed >= 0) {
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
