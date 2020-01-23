//
////Motor Pins
//uint8_t motor_pin[4] = {6, 3, 12, 9};
//uint8_t dir_pin[4] = {5, 2, 11, 8};
/////////////////////////////////////////////////////////////////////

#ifndef WHEELS_DRIVER_H
#define WHEELS_DRIVER_H

#define NUM_MOTORS 4

static inline uint8_t sign(int val) {
 if (val < 0) return 0;
 return 1;
}

//Motor Pins
uint8_t motor_pin[4] = {3, 6, 9, 12};    // FL FR BR BL
uint8_t dir_pin[4]   = {2, 5, 8, 11};
/*
  Pin mappings to motors
  0 - front left motor
  1 - front right motor
  2 - rear right motor
  3 - rear left motor
*/

void init_motors()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(motor_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
  }
}

// Accepts input as 0 - 1023
// downconverts by a factor of 4 to comply with PWM output
void set_motor(int index, int motor_speed)
{
  if (index > NUM_MOTORS || index < 0) return;
  if (abs(motor_speed) < 5) motor_speed = 0;
  if (abs(motor_speed) > 1023) motor_speed = 1023;

  digitalWrite(dir_pin[index], sign(motor_speed));
  analogWrite(motor_pin[index], abs((int)(motor_speed/4)));
}


// forwardV   - (+ + + +) all positive
// rightV     - (+ - + -)
// rotClkwise - (+ - - +)
const int JmulMat[4][] = { {+3, +3, +2},
                           {+3, -3, -2},
                           {+3, +3, -2},
                           {+3, -3, +2}  };

void motors(int forwardV, int rightV, int rotClkwise)
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    static int motor_speed = 0;
    motor_speed += JmulMat[i][0] * forwardV;
    motor_speed += JmulMat[i][1] * rightV;
    motor_speed += JmulMat[i][2] * rotClkwise;
    set_motor(i, motor_speed);
  }
}

#endif  // WHEELS_DRIVER_H
