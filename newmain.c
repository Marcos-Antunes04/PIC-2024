#pragma config FOSC = HS
#pragma config WDTE = OFF 
#pragma config PWRTE = ON 
#pragma config BOREN = OFF
#pragma config LVP = OFF 
#pragma config CPD = OFF 
#pragma config WRT = OFF 
#pragma config CP = OFF 
#define _XTAL_FREQ 20000000
#include <xc.h>
#include "i2c.h"
#define I2C_BAUDRATE 100000  // 100kHz
#define _XTAL_FREQ 20000000
void main(void) {
    TRISB=0x00;
    /*
    while(1){
        PORTB=0XFF;
        __delay_ms(500);
        PORTB=0X00;
        __delay_ms(500);
    }
    */
    
    while(1) {
        __delay_ms(500);  // Aguarda 1 segundo
        PORTB=0XFF;
        I2C_Init(I2C_BAUDRATE);
        I2C_Start();  // Envia condição de início
        I2C_Write(0xA0);  // Envia endereço do dispositivo (por exemplo, 0xA0)
        //I2C_Write(0x00);  // Envia dado (por exemplo, 0x00)
        //I2C_Stop();  // Envia condição de parada
        __delay_ms(500);  // Aguarda 1 segundo
        PORTB=0X00;
    }
return;
}