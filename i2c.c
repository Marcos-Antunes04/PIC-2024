#include "i2c.h"

void I2C_Init(const unsigned long c) {
    SSPCON = 0b00101000;  // Configura como Mestre, modo I2C
    SSPCON2 = 0;
    SSPADD = (_XTAL_FREQ / (4 * c)) - 1;  // Define o baud rate
    SSPSTAT = 0;
    //PORTC = 0X00;
    TRISC3 = 0;  // SCL como saida
    TRISC4 = 0;  // SDA como saida
}

void I2C_Wait(void) {
    while ((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));  // Espera at� que o I2C esteja dispon�vel
}

void I2C_Start(void) {
    I2C_Wait();
    SEN = 1;  // Inicia condi��o de in�cio
}

void I2C_RepeatedStart(void) {
    I2C_Wait();
    RSEN = 1;  // Inicia condi��o de in�cio repetida
}

void I2C_Stop(void) {
    I2C_Wait();
    PEN = 1;  // Inicia condi��o de parada
}

void I2C_Write(unsigned data) {
    I2C_Wait();
    SSPBUF = data;  // Envia dado
    while(!SSPIF);  // Espera o dado ser enviado
    SSPIF = 0;  // Limpa a flag de interrup��o
}

unsigned short I2C_Read(unsigned short ack) {
    unsigned short recvData;
    I2C_Wait();
    RCEN = 1;  // Habilita a recep��o
    while(!SSPIF);  // Espera a recep��o completar
    SSPIF = 0;  // Limpa a flag de interrup��o
    recvData = SSPBUF;  // L� o dado recebido
    I2C_Wait();
    ACKDT = (ack)?0:1;  // Define o valor do ACK
    ACKEN = 1;  // Envia ACK/NACK
    return recvData;
}
