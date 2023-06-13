/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "timer.h"
#include "timer0.h"
#include "timer1.h"
#include "timer2.h"
#include "mcu.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


/* ----------------------- Defines ------------------------------------------*/
/* Timer ticks are counted in multiples of 50us. Therefore 20000 ticks are
 * one second.
 */
#define MB_TIMER_TICKS          ( 20000L )


/* ----------------------- Static variables ---------------------------------*/
static USHORT   usTimerOCRADelta;
static USHORT   usTimerOCRBDelta;

/* ----------------------- Start implementation -----------------------------*/
void xMBPortTimersInit( USHORT usTim1Timeout50us )
{
	 /*
	 Interrupt clear and enable, intinal timer
	 Set different buadrate 16-bit timer counter and compare */
	 timer_modbus_init(usTim1Timeout50us);
}

void vMBPortTimersEnable( void )
{
    /* Reset timer counter and set compare interrupt. */
   /* ----------------------------Timer 16 bit x2------------------------ */
    /*--- Timer counter set ---*/
	write_reg16( TMH2C, 0x0000 );
	write_reg16( TMH1C, 0x0000 );
	/*--- Timer start counting ---*/
	set_bit( TH2RUN );
    set_bit( TH1RUN );
}

void vMBPortTimersDisable( void )
{
	/* ----------------------------Timer 16 bit x2------------------------ */
	/*--- Timer stop counting ---*/
	set_bit( TH1STP );
	set_bit( TH2STP );
}


