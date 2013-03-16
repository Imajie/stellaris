/*
 * File:	jaguar.c
 * Author:	James Letendre
 *
 * Interface for UART -> Jaguar motor controller
 */
#include <stdlib.h>

#include "jaguar.h"

#include "inc/lm4f120h5qr.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"

#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

/*
 * create a packet
 */
void jaguar_make_packet( jaguar_packet_t *packet, api_class_t api_class, uint8_t api_idx, uint8_t dev_id, uint8_t size,
		uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3,
		uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7 )
{
	if( size > 8 ) size = 8;

	packet->size = size + 4;
	packet->header.rsv = 0x0;

	if( api_class == SYSTEM_CONTROL )
	{
		packet->header.devType = 0;
		packet->header.manufact = 0;
		packet->header.api_class = 0;
	}
	else
	{
		packet->header.devType = MOTOR_CONTROLLER;
		packet->header.manufact = TI;
		packet->header.api_class = api_class;
	}
	packet->header.api_idx = api_idx;
	packet->header.dev_id = dev_id;

	packet->data[0] = data0;
	packet->data[1] = data1;
	packet->data[2] = data2;
	packet->data[3] = data3;
	packet->data[4] = data4;
	packet->data[5] = data5;
	packet->data[6] = data6;
	packet->data[7] = data7;

}

/*
 * send a packet
 */
void jaguar_send_packet( jaguar_packet_t *packet )
{
	//GPIOB_PTOR |= 1<<19;
	// send start of frame and size
	UARTCharPut( JAG_UART, 0xFF );

	if( packet->size < 4 ) packet->size = 4;
	if( packet->size > 12) packet->size = 12;
	UARTCharPut( JAG_UART, packet->size );

	// put header in little endian 
	uint32_t *t_ptr = (uint32_t*)&(packet->header);
	uint32_t header = *t_ptr;

	UARTCharPut( JAG_UART, (header>>0 ) & 0xFF );
	UARTCharPut( JAG_UART, (header>>8 ) & 0xFF );
	UARTCharPut( JAG_UART, (header>>16) & 0xFF );
	UARTCharPut( JAG_UART, (header>>24) & 0xFF );

	// put data
	int i;
	for( i = 0; i < packet->size-4; i++ )
	{
		if( packet->data[i] == 0xff )
		{
			UARTCharPut( JAG_UART, 0xfe );
			UARTCharPut( JAG_UART, 0xfe );
		}
		else if( packet->data[i] == 0xfe )
		{
			UARTCharPut( JAG_UART, 0xfe );
			UARTCharPut( JAG_UART, 0xfd );
		}
		else
		{
			UARTCharPut( JAG_UART, packet->data[i] );
		}
	}
}

/*
 * Read packet
 */
int jaguar_read_packet( )
{
	return -1;
}

void heartbeat_int(void)
{
	uint32_t status = TimerIntStatus( TIMER0_BASE, 0 );
	TimerIntClear( TIMER0_BASE, status );
	
	jaguar_heartbeat();
}

void init_jaguar(void)
{
	// because macros
    const unsigned long srcClock = SysCtlClockGet();
    const unsigned long baud = 115200;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	// setup timer for heartbeat
	TimerDisable( TIMER0_BASE, TIMER_BOTH );
	TimerConfigure( TIMER0_BASE, TIMER_CFG_PERIODIC );
	
	// 75 ms period
	uint32_t period_val = ((srcClock / 1000)*75);
	TimerLoadSet( TIMER0_BASE, TIMER_A, period_val );

	// setup interrupts
	TimerIntRegister( TIMER0_BASE, TIMER_A, heartbeat_int );
	TimerIntEnable( TIMER0_BASE, TIMER_TIMA_TIMEOUT );

	// go
	TimerEnable( TIMER0_BASE, TIMER_A );

	// Setup Jaguar UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	GPIOPinConfigure( GPIO_PC4_U1RX );
	GPIOPinConfigure( GPIO_PC5_U1TX );

	// map UART1 to PC4/PC5
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART1_BASE, srcClock, baud,
                (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                 UART_CONFIG_WLEN_8));
    UARTEnable(UART1_BASE);
}

/*
 * API functions
 */

jaguar_packet_data jaguar_device_query( uint8_t dev_id )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, SYSTEM_CONTROL, DEVICE_QUERY, dev_id, 0,
			0,0,0,0, 0,0,0,0 );

	jaguar_send_packet( &packet );

	return NULL;
}

void jaguar_status( uint8_t dev_id, status_api id )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, STATUS, id, dev_id, 0,
			0,0,0,0, 0,0,0,0 );

	jaguar_send_packet( &packet );
	
	return;
}

void jaguar_voltage_control_enable( uint8_t dev_id, int on )
{
	jaguar_packet_t packet;
	
	jaguar_make_packet( &packet, VOLTAGE_CONTROL, (on ? VOLTAGE_MODE_ENABLE : VOLTAGE_MODE_DISABLE), dev_id, 0,
			0,0,0,0, 0,0,0,0 );

	jaguar_send_packet( &packet );

	return;
}

void jaguar_voltage_control_set( uint8_t dev_id, int16_t voltage )
{
	jaguar_packet_t packet;
	
	jaguar_make_packet( &packet, VOLTAGE_CONTROL, VOLTAGE_SET, dev_id, 2,
			voltage & 0xFF, (voltage>>8) & 0xFF,0,0, 
			0,0,0,0 );

	jaguar_send_packet( &packet );

	return;
}

void jaguar_control_mode_enable( api_class_t api, uint8_t dev_id, int on )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, api, (on ? MODE_ENABLE : MODE_DISABLE), dev_id, 0,
			0,0,0,0, 0,0,0,0 );

	jaguar_send_packet( &packet );

	return;
}

void jaguar_control_mode_setPID( api_class_t api, uint8_t dev_id, uint32_t p, uint32_t i, uint32_t d )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, api, SET_P, dev_id, 4, 
			p & 0xFF, (p>>8) & 0xFF, (p>>16) & 0xFF, (p>>24) & 0xFF,
			0,0,0,0 );
	jaguar_send_packet( &packet );

	jaguar_make_packet( &packet, api, SET_I, dev_id, 4, 
			i & 0xFF, (i>>8) & 0xFF, (i>>16) & 0xFF, (i>>24) & 0xFF,
			0,0,0,0 );
	jaguar_send_packet( &packet );

	jaguar_make_packet( &packet, api, SET_D, dev_id, 4, 
			d & 0xFF, (d>>8) & 0xFF, (d>>16) & 0xFF, (d>>24) & 0xFF,
			0,0,0,0 );
	jaguar_send_packet( &packet );

	return;
}

void jaguar_control_mode_set   ( api_class_t api, uint8_t dev_id, uint32_t val )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, api, SET, dev_id, 4, 
			val & 0xFF, (val>>8) & 0xFF, (val>>16) & 0xFF, (val>>24) & 0xFF,
			0,0,0,0 );
	jaguar_send_packet( &packet );

	return;
}

void jaguar_control_mode_reference( api_class_t api, uint8_t dev_id, control_mode_ref_t ref )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, api, REFERENCE, dev_id, 1, 
			ref, 0,0,0,
			0,0,0,0 );
	jaguar_send_packet( &packet );

	return;
}

void jaguar_motor_config( uint8_t dev_id, motor_control_config_api config, uint32_t val )
{
	jaguar_packet_t packet;

	uint8_t size = 1;
	
	switch( config )
	{
		case NUM_BRUSHES: case BREAK_COAST: case LIMIT_MODE: 
			size = 1;
			break;
		case ENCODER_PPR: case NUM_POT_TURNS: case MAX_OUTPUT_VOLTAGE: case FAULT_TIME:
			size = 2;
			break;
		default:
			break;
	}

	jaguar_make_packet( &packet, CONFIGURATION, config, dev_id, size,
			val & 0xFF, (val>>8) & 0xFF, (val>>16) & 0xFF, (val>>24) & 0xFF,
			0, 0, 0, 0 );

	jaguar_send_packet( &packet );

	return;
}

void jaguar_heartbeat( )
{
	jaguar_packet_t packet;

	jaguar_make_packet( &packet, SYSTEM_CONTROL, HEARTBEAT, 0, 0,
			0,0,0,0, 0,0,0,0 );

	jaguar_send_packet( &packet );

	return;
}

