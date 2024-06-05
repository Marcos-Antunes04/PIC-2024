// PIC16F877A Configuration Bit Settings
// 'C' source line config statements
#include <xc.h>
#include <stdint.h>
#include <pic16f877a.h>

#pragma config FOSC = HS
#pragma config WDTE = OFF 
#pragma config PWRTE = OFF 
#pragma config BOREN = ON
#pragma config LVP = OFF 
#pragma config CPD = OFF 
#pragma config WRT = OFF 
#pragma config CP = OFF 
#define _XTAL_FREQ 20000000

void blink_led_nx(int n){
    for(int i = 0; i < n;i++){
        PORTB = 0xff;
        __delay_ms(500);
        PORTB = 0x00;
        __delay_ms(500); 
    }
}

void I2C_Master_Init(const unsigned long c){
  TRISCbits.TRISC3 = 1;        //Setting as input
  TRISCbits.TRISC4 = 1;        //Setting as input
  SSPCON = 0b00101000;
  SSPADD = (_XTAL_FREQ/(4*c))-1;

}

void waitmssp(){
    while(!SSPIF | !SSPBUF);
    SSPIF=0;
}

void I2C_Master_Start()
{
  SSPCON2bits.SEN = 1;
  while(SEN);
  PIR1bits.SSPIF = 0;
  // waitmssp();
}

void I2C_Master_RepeatedStart()
{
  SSPCON2bits.RSEN = 1;
  waitmssp();
}

void I2C_Master_Stop()
{
  SSPCON2bits.PEN = 1;
  while(PEN);
  return;
}

void I2C_Master_Write(unsigned char d){
  SSPBUF = d;
  waitmssp();
}

unsigned short I2C_Master_Read(uint8_t a){
  unsigned short temp;
  waitmssp();
  RCEN = 1;
  waitmssp();
  temp = SSPBUF;
  waitmssp();
  ACKDT = (a)?0:1;
  ACKEN = 1;
  return temp;
}
uint16_t ADC_Read(int channel){
    if(channel > 7)     // Channel range is 0 to 7
        return 0;
    
    ADCON0bits.CHS0 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS2 = 0;
    
    ADCON0 |= channel<<3;
    GO_DONE = 1;
    while(GO_DONE);
    return (uint16_t)((ADRESH << 8) + ADRESL);
}

void main()
{
  // TRIS and PORT registers definitions
  TRISA = 0XFF;
  TRISB = 0x00;                 //PORTB as output
  TRISD = 0x00;                 //PORTD as output
  PORTD = 0x00;                 //All LEDs OFF
  PORTB = 0X00;
  
  // A/D converter module is powered up and ADC clock = Fosc/4
  ADCON0 = 0X81;
  ADCON1 = 0b10000000;
  
  
  while(1){
      I2C_Master_Init(100000);
      I2C_Master_Start();
      I2C_Master_Write(0x33);
      __delay_ms(200);
  }
}