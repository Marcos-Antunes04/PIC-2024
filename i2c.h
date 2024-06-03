#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h>

// Configura a frequência do oscilador
#define _XTAL_FREQ 20000000

// Funções I2C
void I2C_Init(const unsigned long c);
void I2C_Wait(void);
void I2C_Start(void);
void I2C_RepeatedStart(void);
void I2C_Stop(void);
void I2C_Write(unsigned data);
unsigned short I2C_Read(unsigned short ack);


#ifdef	__cplusplus
#endif
#ifdef	__cplusplus
}
#endif

#endif

