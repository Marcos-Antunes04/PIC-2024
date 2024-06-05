// PIC16F877A Configuration Bit Settings
// 'C' source line config statements
#include <xc.h>
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


void I2C_Master_Init(const unsigned long c){
  SSPCON = 0b00101000;
  //SSPCON2 = 0X00;
  SSPADD = (_XTAL_FREQ/(4*c))-1;
  //SSPSTAT = 0;
  //SSPSTAT |= (0b01000000);
  TRISCbits.TRISC3 = 1;        //Setting as input
  TRISCbits.TRISC4 = 1;        //Setting as input
}

void I2C_Master_Wait()
{
  //while(!SSPIF);
  //SSPIF=0;
  while ((SSPSTAT & 0x04) | (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
  SSPCON2bits.SEN=1;
  I2C_Master_Wait();
}

void I2C_Master_RepeatedStart()
{
  SSPCON2bits.RSEN = 1;
  I2C_Master_Wait();
}

void I2C_Master_Stop()
{
  SSPCON2bits.PEN = 1;
  I2C_Master_Wait();
}

void I2C_Master_Write(unsigned char d)
{
  SSPBUF = d;
  I2C_Master_Wait();
  //while(ACKSTAT);
}

unsigned short I2C_Master_Read(uint8_t a)
{
  unsigned short temp;
  I2C_Master_Wait();
  RCEN = 1;
  I2C_Master_Wait();
  temp = SSPBUF;
  I2C_Master_Wait();
  ACKDT = (a)?0:1;
  ACKEN = 1;
  return temp;
}

void main()
{
  nRBPU = 0;                    //Enable PORTB internal pull up resistor
  TRISB = 0x00;                 //PORTB as output
  TRISD = 0x00;                 //PORTD as output
  PORTD = 0x00;                 //All LEDs OFF
  I2C_Master_Init(100000);      //Initialize I2C Master with 100KHz clock
  while(1)
  {
    PORTB=0XFF;                 //All LEDs ON
    I2C_Master_Start();         //Start condition
    I2C_Master_Write(0x30);     //7 bit address + Write
    I2C_Master_Write(PORTB);    //Write data
    I2C_Master_Stop();          //Stop condition
    __delay_ms(500);
    PORTB=0X00;                 //All LEDs OFF
    I2C_Master_Start();         //Start condition
    //I2C_Master_Write(0x31);     //7 bit address + Read
    //PORTD = I2C_Master_Read(0); //Read + Acknowledge
    I2C_Master_Stop();          //Stop condition
    __delay_ms(500);
  }
}