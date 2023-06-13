/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "uart.h"
#include "uart0f.h"
#include "uart0h.h"
#include "irq.h"
#include "mcu.h"
#include "rdwr_reg.h"
/* ----------------------- Modbus includes ----------------------------------*/

#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define U0_CHAR                 ( 0x10 )        /* Data 0:7-bits / 1:8-bits */


#define UART_CLOCK_HZ		   	( 15990784 ) 	// The base clock frequency approximately to 16Mhz
#define uart_init()			uart_init_ex_ex( (UART_CLOCK_HZ/19200) - 1 ) //UART initial
#define uart_init_ex(bsp)	uart_init_ex_ex( (UART_CLOCK_HZ/bsp) - 1 )
/* ----------------------- Static variables ---------------------------------*/
UCHAR           ucGIEWasEnabled = FALSE;
UCHAR           ucCriticalNesting = 0x00;

/*--- UART Setting. ---*/
/*static const tUartSetParam	_uartSetParam = { 
	UART_BAUDRATE,
	UART_DATA_LENGTH,
	UART_PARITY_BIT,
	UART_STOP_BIT,
	UART_LOGIC,
	UART_DIRECTION
};*/

/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
//    	ENTER_CRITICAL_SECTION(  );

 	if( xRxEnable )
 		{	
		//----------------------RXD0----------------------------------------//
		/*--- Set port ---*/
		write_reg8(P7DO, 0x0); //enable port
		clear_bit(U00EN);
		set_bit(U01EN);
		/*--- Clear Error Status ---*/
		write_reg8( UA00STAT, 0x07 );
		write_reg8( UA01STAT, 0x07 );

		/*--- Set Reception Mode in UART ---*/
		set_reg8( UA00MOD, 0x1 );

		/*=== Transmission of a message system order parameter setting. ===*/
		/*=== I receive it, and it is worked to start. ===*/
		set_bit(U00EN);
		clear_bit(U01EN);


 		}
	else if( xTxEnable )	
		{
			//----------------------TXD1------------------------------------//
			set_bit(U01EN);
			clear_bit(U00EN);
			/*--- Change Transmission Mode in UART ---*/
			clear_reg8( UA00MOD, 0x1 );
			write_reg8(P7DO, 0x1); 
			pxMBFrameCBTransmitterEmpty();

		}
	else		
		{ 
			/*--- UART Communication stop  ---*/
			clear_bit(U00EN);
			clear_bit(U01EN);
		}
//    	EXIT_CRITICAL_SECTION(  );
}

BOOL xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            	bInitialized = TRUE;
   // tUartSetParam	_newUartSetParam; 
    
   // _newUartSetParam = _uartSetParam;
   // _newUartSetParam.br = ulBaudRate;

    if( bInitialized )
    {
        ENTER_CRITICAL_SECTION(  );

   		uart_init_ex(ulBaudRate);            
	
        EXIT_CRITICAL_SECTION(  );

    }
    return bInitialized;
}

BOOL xMBPortSerialPutByte( CHAR ucByte )
{
	/*--- Serial Communication Transmit/Receive Buffer  ---*/
	SD0BUFH=ucByte;//=SD0BUFH;
    return TRUE;
}

BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte= SD0BUFL;
    return TRUE;
}

void ENTER_CRITICAL_SECTION( void )
{
	//irq_di();
	__DI();
}

void EXIT_CRITICAL_SECTION( void )
{
	//irq_ei();
	__EI();
}

void vMBPortClose( void )
{
	/*--- Stop UART communication  ---*/
	 uart0f_stop();
	 uart0h1_stop();

}
