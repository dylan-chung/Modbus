
/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "mbtypedef.h"
#include "port.h"
#include "main.h"
#include "mcu.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"


#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif


#define REG_OVERRIDE_FLAG	0x01u

/* ----------------------- Static variables ---------------------------------*/
static const short   usRegCoilStart = REG_COIL_START;
UCHAR usRegCoilBuf[REG_COIL_NREGS/REG_COIL_LEN];
UCHAR usRegCoilWriteFlg[REG_COIL_NREGS/REG_COIL_LEN];
static const short   usRegDiscreteStart = REG_DISCRETE_START;
UCHAR usRegDiscreteBuf[REG_DISCRETE_NREGS/REG_DISCRETE_NREGS];
static const short   usRegInputStart = REG_INPUT_START;
short usRegInputBuf[REG_INPUT_NREGS];
static const short   usRegHoldingStart = REG_HOLDING_START;
short usRegHoldingBuf[REG_HOLDING_NREGS];
UCHAR usRegHoldingWriteFlg[REG_HOLDING_NREGS];

static const unsigned char	_usRegBitConvert[REG_COIL_LEN] = {0x01u, 0x02u, 0x04u, 0x08u, 0x10u, 0x20u, 0x40u, 0x80u};


/* ----------------------- Static variables ---------------------------------*/
UCHAR    ucMBAddress;
static eMBMode  eMBCurrentMode;

static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */
static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived ) ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired ) ( void );

BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_M_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_M_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;


    /* check preconditions */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;

        switch ( eMode )
        {
#if MB_RTU_ENABLED > 0
        case MB_RTU:
            pvMBFrameStartCur = eMBRTUStart;
            pvMBFrameStopCur = eMBRTUStop;
            peMBFrameSendCur = eMBRTUSend;
            peMBFrameReceiveCur = eMBRTUReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;

            eStatus = eMBRTUInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
#if MB_ASCII_ENABLED > 0
        case MB_ASCII:
            pvMBFrameStartCur = eMBASCIIStart;
            pvMBFrameStopCur = eMBASCIIStop;
            peMBFrameSendCur = eMBASCIISend;
            peMBFrameReceiveCur = eMBASCIIReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
            pxMBPortCBTimerExpired = xMBASCIITimerT1SExpired;

            eStatus = eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            eStatus = xMBPortEventInit();
            if( !eStatus )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                eMBCurrentMode = eMode;
                eMBState = STATE_DISABLED;
            }
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit( USHORT ucTCPPort )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( ( eStatus = eMBTCPDoInit( ucTCPPort ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit(  ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        pvMBFrameStartCur = eMBTCPStart;
        pvMBFrameStopCur = eMBTCPStop;
        peMBFrameReceiveCur = eMBTCPReceive;
        peMBFrameSendCur = eMBTCPSend;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
        ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        eMBCurrentMode = MB_TCP;
        eMBState = STATE_DISABLED;
    }
    return eStatus;
}
#endif

eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur(  );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
        vMBPortSerialEnable( TRUE, FALSE );	//Enable Rx Disable Tx
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }

    return eStatus;
}

eMBErrorCode
eMBDisable( void )
{
    eMBErrorCode    eStatus;

    if( eMBState == STATE_ENABLED )
    {
    	vMBPortSerialEnable( FALSE, FALSE );	//Disable Rx Tx
        pvMBFrameStopCur(  );
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBPoll( void )
{
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static UCHAR    ucFunctionCode;
    static USHORT   usLength;
    static eMBException eException;

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if( eMBState != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }


    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet( &eEvent ) == TRUE )
    {
        switch ( eEvent )
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:

            eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );
            if( eStatus == MB_ENOERR )
            {

                /* Check if the frame is for us. If not ignore the frame. */
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {
                    ( void )xMBPortEventPost( EV_EXECUTE );
                }
           
            }
            else
            {
            	 ( void )xMBPortEventPost( EV_READY );
            }	
            break;

        case EV_EXECUTE:
	
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                /* No more function handlers registered. Abort. */
                if( xFuncHandlers[i].ucFunctionCode == 0 )
                {
                    break;
                }
                else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
            if( ucRcvAddress != MB_ADDRESS_BROADCAST )
            {
                if( eException != MB_EX_NONE )
                {
                    /* An exception occured. Build an error frame. */
                    usLength = 0;
                    ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                    ucMBFrame[usLength++] = eException;
                }
			
                eStatus = peMBFrameSendCur( ucMBAddress, ucMBFrame, usLength );
            }
            break;

        case EV_FRAME_SENT:
        	break;
        case EV_FRAME_SENT_TIMEOUT:
        	vMBPortSerialEnable( TRUE, FALSE );
           	break;
        }
    }
    return MB_ENOERR;
}

eMBErrorCode    eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
    	int             iRegIndex;

    	if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    	{
        	iRegIndex = ( int )( usAddress - usRegInputStart );
        	while( usNRegs > 0 )
        	{
            		*pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            		*pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            		iRegIndex++;
            		usNRegs--;
        	}
    	}
    	else
    	{
	        eStatus = MB_ENOREG;
    	}

    	return eStatus;
}

eMBErrorCode    eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
    	int             iRegIndex;

    	if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    	{
        	iRegIndex = ( int )( usAddress - usRegHoldingStart );
        	switch ( eMode )
        	{
            		/* Pass current register values to the protocol stack. */
        		case MB_REG_READ:
            		while( usNRegs > 0 )
            		{
                		*pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                		*pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                		iRegIndex++;
                		usNRegs--;
            		}
            		break;

            		/* Update current register values with new values from the
             		* protocol stack. */
        		case MB_REG_WRITE:
            		while( usNRegs > 0 )
            		{
                		usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                		usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                		usRegHoldingWriteFlg[iRegIndex] = TRUE;
                		iRegIndex++;
                		usNRegs--;
            		}
        	}
    	}
    	else
    	{
        	eStatus = MB_ENOREG;
    	}
    	return eStatus;
}


eMBErrorCode    eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
    	int             iRegIndex;
    	UCHAR		iRegIndexChar, iRegCoil;

	
    	if( ( usAddress >= REG_COIL_START ) && ( usAddress + usNCoils <= REG_COIL_START + REG_COIL_NREGS ) )
    	{
     		iRegCoil = 0;
        	iRegIndexChar = 0;    		
        	
    		switch ( eMode )
        	{
        		case MB_REG_READ:

           			while( usNCoils > 0 )
        			{
        				iRegCoil = iRegCoil >> 1;
        				iRegCoil |= eMBRegGetCoilBit( usAddress );
        				if( ++iRegIndexChar == REG_COIL_LEN )
        				{
            					*pucRegBuffer++ = iRegCoil;
            					iRegCoil = 0;
            					iRegIndexChar = 0;
            				}
            				usAddress++;
            				usNCoils--;
        			}
       		
		       		while( iRegIndexChar != 0 )
		       		{
       					while( iRegIndexChar++ < REG_COIL_LEN )
       					{
        					iRegCoil = iRegCoil >> 1;	
        				}	
        				*pucRegBuffer++ = iRegCoil;
        			}	
        			break;
        			
			case MB_REG_WRITE:        			
                		if( *pucRegBuffer )
                			eMBRegSetBit( usAddress, TRUE);
                		else
                			eMBRegSetBit( usAddress, FALSE);
    		}
    	}	
    	else
    	{
	        eStatus = MB_ENOREG;
    	}

    	return eStatus;
}

eMBErrorCode    eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	eMBErrorCode    eStatus = MB_ENOERR;
    	int             iRegIndex;
    	UCHAR		iRegIndexChar, iRegDiscrete;

	
    	if( ( usAddress >= REG_DISCRETE_START ) && ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_NREGS ) )
    	{
     		iRegDiscrete = 0;
        	iRegIndexChar = 0;
           	while( usNDiscrete > 0 )
        	{
        		iRegDiscrete = iRegDiscrete >> 1;
        		iRegDiscrete |= eMBRegGetDiscreteBit( usAddress );
        		if( ++iRegIndexChar == REG_DISCRETE_LEN )
        		{
            			*pucRegBuffer++ = iRegDiscrete;
            			iRegDiscrete = 0;
            			iRegIndexChar = 0;
            		}
            		usAddress++;
            		usNDiscrete--;
        	}
       		
       		while( iRegIndexChar != 0 )
       		{
       			while( iRegIndexChar++ < REG_DISCRETE_LEN )
       			{
        			iRegDiscrete = iRegDiscrete >> 1;	
        		}	
        		*pucRegBuffer++ = iRegDiscrete;
        	}	
    	}
    	else
    	{
	        eStatus = MB_ENOREG;
    	}

    	return eStatus;
}

UCHAR eMBRegGetDiscreteBit( USHORT usAddress )
{
	UCHAR 	iRegIndex, iRegIndexBit;
	
	iRegIndex = ( int )( usAddress - usRegDiscreteStart );
	iRegIndexBit = iRegIndex % 8;
	iRegIndexBit = _usRegBitConvert[iRegIndexBit];
	iRegIndex = iRegIndex / 8;
	if( usRegDiscreteBuf[iRegIndex] & iRegIndexBit )
		return 0x80u;
	else
		return 0x00u;		
}

UCHAR eMBRegGetCoilBit( USHORT usAddress )
{
	UCHAR 	iRegIndex, iRegIndexBit;
	
	iRegIndex = ( int )( usAddress - usRegCoilStart );
	iRegIndexBit = iRegIndex % 8;
	iRegIndexBit = _usRegBitConvert[iRegIndexBit];
	iRegIndex = iRegIndex / 8;
	if( usRegCoilBuf[iRegIndex] & iRegIndexBit )
		return 0x80u;
	else
		return 0x00u;		
}

UCHAR eMBRegSetBit( USHORT usAddress, UCHAR set )
{
	UCHAR 	iRegIndex, iRegIndexBit;
	
	iRegIndex = ( int )( usAddress - usRegCoilStart );
	iRegIndexBit = iRegIndex % 8;
	iRegIndexBit = _usRegBitConvert[iRegIndexBit];
	iRegIndex = iRegIndex / 8;
	
	usRegCoilWriteFlg[iRegIndex] |= iRegIndexBit;
	
	if( set )
		usRegCoilBuf[iRegIndex] |= iRegIndexBit;
	else
		usRegCoilBuf[iRegIndex] &= ~iRegIndexBit;
}


