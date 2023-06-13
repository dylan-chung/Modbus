#ifndef _PORT_H
#define _PORT_H

/* ----------------------- Platform includes --------------------------------*/
//typedef char    BOOL;

typedef unsigned char UCHAR;

typedef char    CHAR;

typedef unsigned short USHORT;
typedef short   SHORT;

typedef unsigned long ULONG;
typedef long    LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

void            ENTER_CRITICAL_SECTION( void );
void            EXIT_CRITICAL_SECTION( void );


#endif
