#include <stdio.h>

#include "inc/lm4f120h5qr.h"

#include "inc/hw_types.h"
#include "driverlib/sysctl.h"

#include "jaguar.h"

#define WAIT_1MS ((SysCtlClockGet()/3)/1000)

#define LED_RED 0x2
#define LED_BLUE 0x4
#define LED_GREEN 0x8

int main(void)
{
	// enable PORT F GPIO peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	// set LED PORT F pins as outputs
	GPIO_PORTF_DIR_R = LED_RED|LED_BLUE|LED_GREEN;

	// enable digital for LED PORT F pins
	GPIO_PORTF_DEN_R = LED_RED|LED_BLUE|LED_GREEN;

	// clear all PORT F pins
	GPIO_PORTF_DATA_R = 0;

	init_jaguar();

	SysCtlDelay(10* WAIT_1MS);

	jaguar_motor_config( 1, ENCODER_PPR, 121*4 ); //141 );

	// 16.16 fixed point constants
#define P	0x00010000
#define I	0x00000000
#define D	0x00000000

	jaguar_control_mode_enable( SPEED_CONTROL, 1, 1 );
	jaguar_control_mode_setPID( SPEED_CONTROL, 1, P, I, D );
	jaguar_control_mode_reference( SPEED_CONTROL, 1, QUAD_ENCODER );

	jaguar_control_mode_set( SPEED_CONTROL, 1, 80<<16 );

//	jaguar_voltage_control_enable( 0, 1 );
//	jaguar_voltage_control_set( 0, 15000 );

	// loop forever
	for(;;)
	{
		// clear all PORT F pins
		GPIO_PORTF_DATA_R = 0;

		SysCtlDelay(500* WAIT_1MS);

		// set LED PORT F pins high
		GPIO_PORTF_DATA_R |= LED_RED|LED_BLUE|LED_GREEN;

		SysCtlDelay(100* WAIT_1MS);
	}

	for(;;);
	return 0;
}
