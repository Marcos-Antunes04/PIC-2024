// PIC16F877A Configuration Bit Settings
// 'C' source line config statements
#include <xc.h>
#include <stdint.h>
#include <pic16f877a.h>
#include <stdio.h>
#include <string.h>

#pragma config FOSC = HS
#pragma config WDTE = OFF 
#pragma config PWRTE = OFF 
#pragma config BOREN = ON
#pragma config LVP = OFF 
#pragma config CPD = OFF 
#pragma config WRT = OFF 
#pragma config CP = OFF 
#define _XTAL_FREQ 20000000
#define DISPLAY_ADDR 0X27

// Defines used in GPIO Display
#define rs RC0
#define rw RC1
#define en RC2
#define delay for(int j=0;j<1000;j++)

void blink_led_nx(int n){
    for(int i = 0; i < n;i++){
        PORTB = 0xff;
        __delay_ms(500);
        PORTB = 0x00;
        __delay_ms(500); 
    }
}

void I2C_Master_Init(const unsigned long c){
  SSPCON = 0b00101000;
  SSPCON2 = 0x00;
  SSPSTAT = 0x00;
  SSPADD = (_XTAL_FREQ/(4*c))-1;
  TRISCbits.TRISC3 = 1;        //Setting as input
  TRISCbits.TRISC4 = 1;        //Setting as input
}

void I2C_IDLE()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Start()
{
I2C_IDLE();
SSPCON2bits.SEN = 1;  // initial start condition on SDA line
}

void I2C_Stop()
{
I2C_IDLE();
SSPCON2bits.PEN = 1; // Initiate Stop condition on SDA and SCL pins
}

void I2C_Restart()
{
I2C_IDLE();
SSPCON2bits.RSEN = 1; // Initiate Repeated Start condition on SDA and SCL pins.
}

void I2C_ACK(void)
{
  I2C_IDLE();
  SSPCON2bits.ACKDT = 0; //Acknowledge Data bit  
  SSPCON2bits.ACKEN = 1;  // Acknowledge Sequence Enable bit(
}
void I2C_NACK(void)
{
I2C_IDLE();
SSPCON2bits.ACKDT = 1; 
SSPCON2bits.ACKEN = 1; 
}

unsigned char I2C_Write(unsigned char Data)
{
I2C_IDLE();   // wait untill I2C_Bus of PIC18F4550 microcontroller becomes free 
SSPBUF = Data; // store data inside SSPBUF register of PIC18F4550 microcontroller
I2C_IDLE();  
return ACKSTAT; //return status of data or address transmission
}

unsigned char I2C_Read_Byte(void)
{
SSPCON2bits.RCEN = 1; // Enable & Start Reception
while(!SSPIF); // Wait Until Completion
SSPIF = 0; // Clear The Interrupt Flag Bit
return SSPBUF; // Return The Received Byte
}

void I2C_Multi_Send(uint8_t cmd, uint8_t address, uint8_t *data, int size){
    uint8_t send = (uint8_t) ((address << 1) & (0b11111110));
    I2C_Write(send);
    for(int n = 0; n < size; n++){
        I2C_Write(data[n]);
    }
}

// Must be implementted
uint16_t *I2C_Multi_Read(){
    uint16_t Accel[2];
    return Accel;
}

void cmd(unsigned char a)
{
    PORTB=a;
    rs=0;
    rw=0;
    en=1;
    delay;
    en=0;
}

void lcd_init()
{
    cmd(0x38);
    cmd(0x0c);
    cmd(0x06);
    cmd(0x80);
}

void dat(unsigned char b)
{
    PORTB=b;
    rs=1;
    rw=0;
    en=1;
    delay;
    en=0;
}
void show(unsigned char *s)
{
    while(*s) {
        dat(*s++);
    }
}

void main()
{   
  unsigned int i;
  // TRIS and PORT registers definitions
  TRISB = 0x00;                 //PORTB as output
  PORTB = 0X00;
  TRISC0= 0;
  TRISC1= 0;
  TRISC2= 0;
  
  // Configures MSSP peripheral in I2C Master mode
  I2C_Master_Init(100000);
    
    
  lcd_init();
  cmd(0x8A); //forcing the cursor at 0x8A position
  show("Antunes-Plot");
  while(1) {
      for(i=0;i<15000;i++);
      cmd(0x18);
      for(i=0;i<15000;i++);       
  }     
}