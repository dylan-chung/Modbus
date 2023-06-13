
#include "mb.h"

static USHORT usRegInputBuf[REG_INPUT_NREGS];

int main(void)
{

    eMBErrorCode    eStatus = MB_ENOERR;

    /*--- Disable Interrupt ---*/
    __DI();

    /*--- Initial.... ---*/
	

    write_reg16(P1MOD7, 0x21); // P17: UART_RX (RXD0)
	write_reg16(P2MOD0, 0x12); // P20: UART_TX (TXD1)

    eStatus = eMBInit( MB_RTU, 0x0A, 0, 38400, MB_PAR_EVEN );
     if( eStatus != MB_ENOERR )
    {
		eStatus = eMBEnable();
    }

    for (;;) 
	{
        eMBPoll();
	    usRegInputBuf[0]++;
    }
}
