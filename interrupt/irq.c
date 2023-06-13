#include "mb.h"
/*---   Interrupt   ---*/
static void s_handlerSIU00INT( void )
{
uart_modbus();
}


void uart_modbus( void )
{
	unsigned char u0io;
	unsigned char u0ful;
	unsigned char u0en;

	/*---	Transmission mode	---*/
	u0io = read_reg8((unsigned char)(UA00MOD)&0x1);
	if( u0io == 0 )
	{
		/*---	Any Message data left?	---*/
		u0ful = (unsigned char)( read_reg8(UA00STAT) & 0x8 );
		/*---	Is there data in the message buffer?	---*/
		if( u0ful == 0 )
		{		
			pxMBFrameCBTransmitterEmpty();
		}	
		
	}
	/*---	Reception mode	---*/
	else
	{
		/*---	Any reception data left?	---*/
		uart0f_clrErrorStatus();
    	pxMBFrameCBByteReceived();
	}
}