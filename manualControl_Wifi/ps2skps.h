//
//	Author				: ibrahim - Orignal by CYTRON
//	Project				: PR23 + SKPS
//	Project description	: Function calls and defines for PS2 with SKPS

//	define
//=======================================================================

#ifndef PS2SKPS_H
#define PS2SKPS_H

#define BUFFER_LENGTH 6

#define rx_pin 4
#define tx_pin 5

//skps protocol
#define	p_select		0
#define p_joyl			1
#define p_joyr			2
#define p_start			3
#define p_up			4
#define p_right			5
#define p_down			6
#define p_left			7
#define	p_l2			8
#define	p_r2			9
#define p_l1			10
#define p_r1			11
#define p_triangle		12
#define p_circle		13
#define p_cross			14
#define	p_square		15
#define p_joy_lx		16
#define	p_joy_ly		17
#define p_joy_rx		18
#define p_joy_ry		19
#define p_joy_lu		20
#define p_joy_ld		21
#define p_joy_ll		22
#define p_joy_lr		23
#define p_joy_ru		24
#define p_joy_rd		25
#define p_joy_rl		26
#define p_joy_rr		27

#define	p_con_status	28
#define p_motor1		29
#define p_motor2		30

#define p_all       31

// Includes
#include <SoftwareSerial.h>

// Global variables
//=======================================================================
unsigned long time_out = 100;
SoftwareSerial controller(rx_pin, tx_pin);

//	function prototype
//=======================================================================
unsigned char* init_skps(void);

void uart_send(unsigned char data);
unsigned char uart_rec(void);
unsigned char skps(unsigned char data);
void skps_vibrate(unsigned char motor, unsigned char value);

// initailization
//=======================================================================
unsigned char* init_skps()
{
	controller.begin(9600);

	return (unsigned char *)malloc(sizeof(unsigned char) * BUFFER_LENGTH);
}

// uart function
//=======================================================================

//function to send out a byte via uart
void uart_send(unsigned char data)
{
	while (controller.available() > 0)
  {
    controller.read();
  }
  controller.write(data);
}

//function to wait for a byte receive from uart
int uart_rec(unsigned char *buffer)
{
	unsigned long prev_ms = millis();
	int numBytes = 0;

	while ((millis() - prev_ms) < time_out){
		if (controller.available()){
			buffer[numBytes] = controller.read();
			numBytes++;
		}

		if (numBytes >= BUFFER_LENGTH){
			break;
		}
	}

	return numBytes;			//return the number of bytes received
}

// skps function
//=======================================================================

//function to read button and joystick
//information on ps controller
int skps(unsigned char data, unsigned char *buffer)
{
	uart_send(data);
	return uart_rec(buffer);
}

//function to control the vibrator motor
//on ps controller
void skps_vibrate(unsigned char motor, unsigned char value)
{
	uart_send(motor);
	uart_send(value);
}

#endif	 // PS2SKPS_H
