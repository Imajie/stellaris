/*
 * File:	cpu_interface.c
 * Author:	James Letendre
 *
 * Interface from Stellaris to the CPU
 */
#include "cpu_interface.h"
#include "inc/lm4f120h5qr.h"

#include "driverlib/uart.h"

/*
 * Handle a received CPU message 
 */
void handle_message( cpu_message_t *message );
void handle_control_msg( cpu_message_t *message );
void handle_closed_loop_msg( cpu_message_t *message );
void handle_open_loop_msg( cpu_message_t *message );
void handle_monitor_msg( cpu_message_t *message );

/*
 * create a cpu message
 *
 * returns 0 if valid message was created
 */
int cpu_message_create( cpu_message_t *message, cpu_msg_class msg_class, 
		uint8_t msg_idx, uint8_t size, uint8_t data[] )
{
	if( size > 8 || ( size > 0 && data == NULL ) ||
			message == NULL) 
		return -1;

	// set new message info
	message->message_info.size	= size;
	message->message_info.index	= msg_idx;
	message->message_info.class	= msg_class;

	for( int i = 0; i < size; i++ )
	{
		message->data[i] = data[i];
	}

	return 0;
}

/* 
 * send a cpu message 
 */
void cpu_message_send( cpu_message_t *message )
{
	// valid message ptr?
	if( !message ) return;

	uint8_t size = message->message_info.size;

	// SOF
	UARTCharPut( UART0_BASE, 0xFF );

	// Message info
	if( message->message_info.all == 0xFF )
	{
		UARTCharPut( UART0_BASE, 0xFE );
		UARTCharPut( UART0_BASE, 0xFE );
	}
	else if( message->message_info.all == 0xFE )
	{
		UARTCharPut( UART0_BASE, 0xFE );
		UARTCharPut( UART0_BASE, 0xFD );
	}
	else
	{
		UARTCharPut( UART0_BASE, message->message_info.all );
	}

	// data bytes
	for( int i = 0; i < size; i++ )
	{
		if( message->data[i] == 0xFF )
		{
			UARTCharPut( UART0_BASE, 0xFE );
			UARTCharPut( UART0_BASE, 0xFE );
		}
		else if( message->data[i] == 0xFE )
		{
			UARTCharPut( UART0_BASE, 0xFE );
			UARTCharPut( UART0_BASE, 0xFD );
		}
		else
		{
			UARTCharPut( UART0_BASE, message->data[i] );
		}
	}
}

/*
 * Buffer space to build a message object into
 */
uint8_t message_buffer[9];
int8_t message_buffer_index = -1;
int8_t message_escape_received = 0;	// received the 0xFE byte

/* 
 * process a cpu message 
 *
 * returns: 1 if no message processed
 */
int cpu_message_process( void )
{
	if( UARTCharsAvail( UART0_BASE ) )
	{
		uint8_t byte = UARTCharGet( UART0_BASE );
		if( message_buffer_index == -1 && byte == 0xFF )
		{
			// start of message found
			message_buffer_index = 0;
		}
		else if( message_buffer_index >= 0 )
		{
			// Next byte of data
			if( byte == 0xFE && message_escape_received == 0 ) 
			{
				// escape character byte
				message_escape_received = 1;

				return;
			}
			else if( message_escape_received == 1 )
			{
				// was escaped before
				message_escapse_received = 0;

				if( byte == 0xFE )		byte = 0xFF;
				else if( byte == 0xFD ) byte = 0xFD;
				else
				{
					// Badly formated message, restart
					message_buffer_index = -1;
					return;
				}
			}
			// else -> normal byte

			message_buffer[message_buffer_index++] = byte;

			// check if the message is done being received
			cpu_message_info_t info = (cpu_message_info_t)message_buffer[0];
			if( info.size == message_buffer_index - 1 )
			{
				// have whole message, create object
				cpu_message_t msg;
				cpu_message_create( &msg, info.class, info.index, info.size, &message_buffer[1] );

				// handle this message
				handle_message( &msg );
			}
		}
	}
}

void handle_message( cpu_message_t *message )
{
	// First, what message type is this
	switch( message->message_info.class )
	{
		case CPU_MSG_CONTROL:
			handle_control_msg( message );
			break;
		case CPU_MSG_CLOSED_LOOP:
			handle_closed_loop_msg( message );
			break;
		case CPU_MSG_OPEN_LOOP:
			handle_open_loop_msg( message );
			break;
		case CPU_MSG_MONITOR:
			handle_monitor_msg( message );
			break;
	}
}

void handle_control_msg( cpu_message_t *message )
{
	// handle control messages
	switch( message->message_info.index )
	{
		case CPU_CONTROL_HEARTBEAT:
			
			break;
		case CPU_CONTROL_RESET:

			break;
		case CPU_CONTROL_STOP:

			break;
		case CPU_CONTROL_CLOSED_LOOP:

			break;
		case CPU_CONTROL_OPEN_LOOP:

			break;
		default:
			// Invalid
			break;
	}
}

void handle_closed_loop_msg( cpu_message_t *message );
void handle_open_loop_msg( cpu_message_t *message );
void handle_monitor_msg( cpu_message_t *message );

#endif
