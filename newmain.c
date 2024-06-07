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
#define DISPLAY_ADDR 0X27

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

void ADC_Setup(void){
  ADCON0 = 0X81;
  ADCON1 = 0b10000000;
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

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
    I2C_Start();
    I2C_Multi_Send(0,DISPLAY_ADDR,data_t,sizeof(data_t));
    I2C_Stop();
    __delay_ms(50);
	// HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	I2C_Start();
    I2C_Multi_Send(0,DISPLAY_ADDR,data_t,sizeof(data_t));
    I2C_Stop();
    __delay_ms(50);
	// HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	__delay_ms(50);
	lcd_send_cmd (0x30);
	__delay_ms(5);
	lcd_send_cmd (0x30);
	__delay_ms(1);
	lcd_send_cmd (0x30);
	__delay_ms(10);
	lcd_send_cmd (0x20);  // 4bit mode
	__delay_ms(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	__delay_ms(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	__delay_ms(1);
	lcd_send_cmd (0x01);  // clear display
	__delay_ms(1);
	__delay_ms(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	__delay_ms(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

/* Global variable declarations*/
uint16_t adc_value = 0;


void main()
{   
  // TRIS and PORT registers definitions
  TRISA = 0XFF;
  TRISB = 0x00;                 //PORTB as output
  TRISD = 0x00;                 //PORTD as output
  PORTD = 0x00;                 //All LEDs OFF
  PORTB = 0X00;
  // Configures MSSP peripheral in I2C Master mode
  I2C_Master_Init(100000);  
  // A/D converter module is powered up and ADC clock = Fosc/4
  ADC_Setup();
  
  __delay_ms(2000);
  
  lcd_init();
  lcd_send_string("Hello World");
  
  __delay_ms(2000);
  
  lcd_put_cur(1,0);
  
  lcd_send_string("From Antunes");
  
  lcd_clear();
  int row = 0;
  int col = 0;
  
  while(1){
      for(int i = 0; i<128;i++){
          lcd_put_cur(row,col);
          lcd_send_data(i+48);
          col++;
          if(col>15) {row++; col=0;}
          if(row>1) row=0;
          __delay_ms(50);
      }
  }
}