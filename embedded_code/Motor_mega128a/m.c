/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 10/18/2015
Author  : 
Company : 
Comments: 


Chip type               : ATmega128A
Program type            : Application
AVR Core Clock frequency: 14.745600 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024
*******************************************************/

#include <mega128a.h>
#include <delay.h>
// Alphanumeric LCD functions
#include <alcd.h>
// Standard Input/Output functions
#include <stdio.h>
#include <stdlib.h>


//-------------------- PID --------------------
unsigned char BYTE_L;
unsigned char BYTE_H;

void updatePID(char motor, char dir, unsigned char command);
unsigned char getWheelSpeed(unsigned char addr);
void SRF08_setGain(unsigned char addr, unsigned char gain);
void SRF08_setRange(unsigned char addr, unsigned char range);
unsigned int SRF08_getRange(unsigned char addr);

// TWI Master Transmitter or Receiver mode selection
#define TWI_WRITE 0
#define TWI_READ  1
//unsigned char twi_status;

void initI2C(void);
/* Sets pullups and initializes bus speed to 100kHz (at FCPU=8MHz) */
void i2cWaitForComplete(void);
/* Waits until the hardware sets the TWINT flag */
void i2cStart(void);
/* Sends a start condition (sets TWSTA) */
void i2cStop(void);
/* Sends a stop condition (sets TWSTO) */
void i2cSend(unsigned char data);
/* Loads data, sends it out, waiting for completion */
unsigned char i2cReadAck(void);
/* Read in from slave, sending ACK when done (sets TWEA) */
unsigned char i2cReadNoAck(void);
/* Read in from slave, sending NOACK when done (no TWEA) */

unsigned char dir1; // Direction of Motor 1
unsigned char dir2; // Direction of Motor 2
unsigned char dir3; // Direction of Motor 3
unsigned char dir4; // Direction of Motor 4

unsigned char pwm1; // PWM of Motor 1
unsigned char pwm2; // PWM of Motor 2
unsigned char pwm3; // PWM of Motor 3
unsigned char pwm4; // PWM of Motor 4

unsigned char speed1,speed2,speed3,speed4; // Actual Speed of Motors
unsigned char command1,command2,command3,command4; // Actual Speed of Motors

unsigned char motorID;

void motorControl(unsigned char motor, unsigned char dir, unsigned char pwm);
unsigned char setPWM (unsigned char pwm);
void motorForward(void);
void motorBackward(void);
void motorRight(void);
void motorLeft(void);

#define INA1   PORTD2
#define INB1   PORTD3
#define PTINA1  PORTD   // PORT OF INA1
#define PTINB1  PORTD   // PORT OF INB1
#define PWM1H  OCR1AH
#define PWM1L  OCR1AL
#define EN1    PORTF7

#define INA2   PORTB7
#define INB2   PORTD4
#define PTINA2  PORTB   // PORT OF INA2
#define PTINB2  PORTD   // PORT OF INB2
#define PWM2H  OCR1BH
#define PWM2L  OCR1BL
#define EN2    PORTF6

#define INA3   PORTE5
#define INB3   PORTE6
#define PTINA3  PORTE   // PORT OF INA3
#define PTINB3  PORTE   // PORT OF INB3
#define PWM3H  OCR3AH
#define PWM3L  OCR3AL
#define EN3    PORTB0

#define INA4   PORTE2
#define INB4   PORTE7
#define PTINA4  PORTE   // PORT OF INA4
#define PTINB4  PORTE   // PORT OF INB4
#define PWM4H  OCR3BH
#define PWM4L  OCR3BL
#define EN4    PORTB1

#define FORWARD 0
#define BACKWARD 1
#define BRAKE_VCC 2
#define BRAKE_GND 3

#define BV(bit) (1 << bit)
#define setBit(byte, bit) (byte |= BV(bit))
#define clearBit(byte, bit) (byte &= ~BV(bit))
#define toggleBit(byte, bit) (byte ^= BV(bit))

#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 8
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0 <= 256
unsigned char rx_wr_index0=0,rx_rd_index0=0;
#else
unsigned int rx_wr_index0=0,rx_rd_index0=0;
#endif

#if RX_BUFFER_SIZE0 < 256
unsigned char rx_counter0=0;
#else
unsigned int rx_counter0=0;
#endif

// This flag is set on USART0 Receiver buffer overflow
bit rx_buffer_overflow0;

// USART0 Receiver interrupt service routine
interrupt [USART0_RXC] void usart0_rx_isr(void)
{
char status,data;
status=UCSR0A;
data=UDR0;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   rx_buffer0[rx_wr_index0++]=data;
#if RX_BUFFER_SIZE0 == 256
   // special case for receiver buffer size=256
   if (++rx_counter0 == 0) rx_buffer_overflow0=1;
#else
   if (rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
   if (++rx_counter0 == RX_BUFFER_SIZE0)
      {
      rx_counter0=0;
      rx_buffer_overflow0=1;
      }
#endif
   }
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
char data;
while (rx_counter0==0);
data=rx_buffer0[rx_rd_index0++];
#if RX_BUFFER_SIZE0 != 256
if (rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#endif
#asm("cli")
--rx_counter0;
#asm("sei")
return data;
}
#pragma used-
#endif

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}

    unsigned char actual;
    signed int error,error1,error2;
    signed long integral;
    float P,I,D;
    signed long drive;
    signed long last;
    float kP=1.2;
    float kI=0.5;
    float kD=0.1; 

// Timer 0 overflow interrupt service routine (for PID)
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{




}

void main(void)
{
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=0 Bit3=0 Bit2=0 Bit1=P Bit0=P 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);

// Port E initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In 
DDRE=(1<<DDE7) | (1<<DDE6) | (1<<DDE5) | (1<<DDE4) | (1<<DDE3) | (1<<DDE2) | (0<<DDE1) | (0<<DDE0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRG=(0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTG=(0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0 output: Disconnected
ASSR=0<<AS0;
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 14745.600 kHz
// Mode: Ph. correct PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Non-Inverted PWM
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.034587 ms
// Output Pulse(s):
// OC1A Period: 0.034587 ms Width: 0 us
// OC1B Period: 0.034587 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
TCCR2=(0<<WGM20) | (0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: 14745.600 kHz
// Mode: Ph. correct PWM top=0x00FF
// OC3A output: Non-Inverted PWM
// OC3B output: Non-Inverted PWM
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.034587 ms
// Output Pulse(s):
// OC3A Period: 0.034587 ms Width: 0 us
// OC3B Period: 0.034587 ms Width: 0 us
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(1<<COM3A1) | (0<<COM3A0) | (1<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (1<<WGM30);
TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (1<<CS30);
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);

// USART0 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART0 Receiver: On
// USART0 Transmitter: On
// USART0 Mode: Asynchronous
// USART0 Baud Rate: 115200
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL0) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x07;

// USART1 initialization
// USART1 disabled
UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

// ADC initialization
// ADC Clock frequency: 921.600 kHz
// ADC Voltage Reference: AVCC pin
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
SFIOR=(0<<ACME);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// Mode: TWI Master
initI2C();

// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTA Bit 0
// RD - PORTA Bit 1
// EN - PORTA Bit 2
// D4 - PORTA Bit 4
// D5 - PORTA Bit 5
// D6 - PORTA Bit 6
// D7 - PORTA Bit 7
// Characters/line: 16
lcd_init(16);

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/512k
//#pragma optsize-
//WDTCR=(1<<WDCE) | (1<<WDE) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);
//WDTCR=(0<<WDCE) | (1<<WDE) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);
//#ifdef _OPTIMIZE_SIZE_
//#pragma optsize+
//#endif

// Global enable interrupts
#asm("sei")

while (1)
      {
      // Place your code here
      char str[32];
      static signed char k=10;
      
      
/*************************Encoder Read Data******************************/
//speed2=getWheelSpeed(0x02);
//speed3=getWheelSpeed(0x03);
//speed4=getWheelSpeed(0x04);
//
//printf("speed1: %d\r\n",speed1);
//printf("speed2: %d\r\n",speed2);
//printf("speed3: %d\r\n",speed3);
//printf("speed4: %d\r\n",speed4);
//
//sprintf(str,"%d,%d,%d,%d",speed1,speed2,speed3,speed4);
//lcd_clear();
//lcd_puts(str);
//delay_ms(10);

/************************************************************************/

/*********************************PID***********************************/
//motorForward();

motorRight();
 
//sprintf(str,"%d:%d:%d:%d",speed1,speed2,speed3,speed4);
//
//lcd_clear();
//lcd_puts(str);

//motorBackward();

//motorLeft();

/**********************************************************************/

/*************************Motor Control**********************************/      
//      static char i=1;
//      
//      motorControl(i, 0, 100);
//      lcd_clear();
//      sprintf(str,"%d: Forward",i);
//      lcd_puts(str);
//     
//      delay_ms(1000);
//      
//      motorControl(i, 2, 100);
//      lcd_clear();
//      sprintf(str,"%d: VCC",i);
//      lcd_puts(str);
//
//      delay_ms(1000);
//      
//      motorControl(i, 1, 100);
//      lcd_clear();
//      sprintf(str,"%d: Backward",i);
//      lcd_puts(str);
//
//      delay_ms(1000);
//      
//      motorControl(i, 3, 100);
//      lcd_clear();
//      sprintf(str,"%d: GND",i);
//      lcd_puts(str);
//      
//      delay_ms(1000);
//      
//      i++; if(i==5) i=1;
/************************************************************************/

//      
//      lcd_clear();
//      lcd_puts("Forward");
//
//      motorForward();      
//      delay_ms(1000);
//      
//      lcd_clear();
//      lcd_puts("Right");
//      
//      motorRight();
//      delay_ms(1000);
//      
//      lcd_clear();
//      lcd_puts("Backward");
//      
//      motorBackward();
//      delay_ms(1000);
//      
//      lcd_clear();
//      lcd_puts("Left");
//
//      motorLeft();
//      delay_ms(1000);
/*************************Motor Control**********************************/
            
/*****************************SRF08*************************************/
//unsigned int range;
//
//SRF08_setRange(0x70,0x8C);
//
//SRF08_setGain(0x70,0x00);
//
//range=SRF08_getRange(0x70);
//  
//lcd_clear();
//   
//sprintf(str,"%d",range);
//
//lcd_puts(str);
/*****************************SRF08*************************************/
      }
}

void updatePID(char motor, char dir, unsigned char command)
{
    actual = getWheelSpeed(motor);
    //putchar(Actual);
    error = command - actual;
    
//    if (abs(Error) < IntThresh)  // prevent integral 'windup'
//    { 
//        Integral = Integral + Error; // accumulate the error integral
//    }
//    else
//    {
//        Integral=0; // zero it if out of bounds
//    }
    
    P = kP*(error-error1);              // calc proportional term
    I = kI*(error+error1)/2;           // integral term
    D = kD*(error-2*error1+error1);      // derivative term 
    
    drive = last + (int)(P + I + D);         // Total drive = P+I+D
    drive = (int)(drive); // scale Drive to be in the range 0-255

    
    if (drive<0) {
        drive=0;
    }
    
    if (drive>100) {
        drive=100;
    }
    
    motorControl(motor, dir, drive); // send PWM command to motor board
    last = drive; // save current value for next time 
    error2=error1;
    error1=error;
    
    //printf("%4d :: %4d:%4d:%4d :: %4d:%4d\r\n",Actual,Error2,Error1,Error,Drive,Last);   
}

unsigned char getWheelSpeed(unsigned char addr)
{
    unsigned char speed;
    
    //  if speed is 255 it means Data is not ready
    
    do
    {
        i2cStart();
        i2cSend((addr<<1) | TWI_WRITE);
        i2cSend(1);
        i2cStart();
        i2cSend((addr<<1) | TWI_READ);
        speed = i2cReadNoAck();
        i2cStop();
    } while (speed==255);
    
    switch (addr)
    {
        case 1:
            speed1=2*speed;
            break;
        case 2:
            speed2=2*speed;
            break;
        case 3:
            speed3=2*speed;
            break;
        case 4:
            speed4=2*speed;
            break;            
    }
    //max speed is about 40 so we double it
    return 2*speed;
}

// Set up SRF08 receiver sensitivity over I2C bus
void SRF08_setGain(unsigned char addr, unsigned char gain)
{
   i2cStart();
   i2cSend((addr<<1) | TWI_WRITE);
   i2cSend(0x01);    // Receiver gain register
   i2cSend(gain);    // Set max receiver gain
   i2cStop();
}

// Set up SRF08 max range over I2C bus
void SRF08_setRange(unsigned char addr, unsigned char range)
{
   i2cStart();
   i2cSend((addr<<1) | TWI_WRITE);
   i2cSend(0x02);    // Range register
   i2cSend(range);   // Set max range
   i2cStop();
}

// Get range data from SRF08
unsigned int SRF08_getRange(unsigned char addr)
{
   // Send Tx burst command over I2C bus
   i2cStart();
   i2cSend((addr<<1) | TWI_WRITE);
   i2cSend(0x00);    // Command register
   i2cSend(0x51);    // Ranging results in cm
   i2cStop();
   
   delay_ms(100);    // Wait for return echo
   
   // Read back range over I2C bus
   i2cStart();
   i2cSend((addr<<1) | TWI_WRITE);
   i2cSend(0x02);
   i2cStop();
   
   i2cStart();
   i2cSend((addr<<1) | TWI_READ);
   // Read two-byte echo result
   BYTE_H = i2cReadAck();
   BYTE_L = i2cReadNoAck();
   i2cStop();
   
   return ((unsigned int)BYTE_H<<8) | BYTE_L;
}

void initI2C(void) {
    TWBR = 65; /* set bit rate, see p. 242 */
    TWSR = 0;
    /* 14.745600 MHz / (16+2*TWBR*1) ~= 184kHz */
    // TWBR = 65 ---> 100kHz
    // TWBR = 36 ---> 167kHz
    // TWBR = 25 ---> 233kHz
    // TWBR = 11 ---> 388kHz    // Not Appropriate
    TWCR |= (1 << TWEN); /* enable */
}

void i2cWaitForComplete(void) {
    #asm("wdr")
    while (!(TWCR & (1<<TWINT)));
}

void i2cStart(void) {
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    i2cWaitForComplete();
}

void i2cStop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

unsigned char i2cReadAck(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    i2cWaitForComplete();
    return (TWDR);
}

unsigned char i2cReadNoAck(void) {
    TWCR = (1<<TWINT) | (1<<TWEN);
    i2cWaitForComplete();
    return (TWDR);
}

void i2cSend(unsigned char data) {
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN); /* init and enable */
    i2cWaitForComplete();
}


void motorControl(unsigned char motor, unsigned char dir, unsigned char pwm)
{
    switch(motor)
    {
        case 1:
        
            PWM1L = setPWM(pwm);
            //PWM1H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 1 Forward
                    setBit(PTINA1,INA1);       // INA1 = 1
                    clearBit(PTINB1,INB1);     // INB1 = 0
                    break;
                case 1:  // Motor 1 Backward
                    clearBit(PTINA1,INA1);    // INA1 = 0
                    setBit(PTINB1,INB1);      // INB1 = 1
                    break;
                case 2:  // Motor 1 Brake to VCC 
                    setBit(PTINA1,INA1);      // INA1 = 1
                    setBit(PTINB1,INB1);      // INB1 = 1
                    break;
                case 3:  // Motor 1 Brake to GND
                    clearBit(PTINA1,INA1);    // INA1 = 0
                    clearBit(PTINB1,INB1);    // INB1 = 0
                    break;
            }
            
            pwm1=pwm;
            dir1=dir;
            
            break;
            
        case 2:
        
            PWM2L = setPWM(pwm);
            //PWM2H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 2 Forward
                    setBit(PTINA2,INA2);       // INA2 = 1
                    clearBit(PTINB2,INB2);     // INB2 = 0
                    break;
                case 1:  // Motor 2 Backward
                    clearBit(PTINA2,INA2);    // INA2 = 0
                    setBit(PTINB2,INB2);      // INB2 = 1
                    break;
                case 2:  // Motor 2 Brake to VCC 
                    setBit(PTINA2,INA2);      // INA2 = 1
                    setBit(PTINB2,INB2);      // INB2 = 1
                    break;
                case 3:  // Motor 2 Brake to GND
                    clearBit(PTINA2,INA2);    // INA2 = 0
                    clearBit(PTINB2,INB2);    // INB2 = 0
                    break;
            }
            
            pwm2=pwm;
            dir2=dir;
            
            break;
            
        case 3:
        
            PWM3L = setPWM(pwm);
            //PWM3H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 3 Forward
                    setBit(PTINA3,INA3);        // INA3 = 1
                    clearBit(PTINB3,INB3);      // INB3 = 0
                    break;
                case 1:  // Motor 3 Backward
                    clearBit(PTINA3,INA3);     // INA3 = 0
                    setBit(PTINB3,INB3);       // INB3 = 1
                    break;
                case 2:  // Motor 3 Brake to VCC 
                    setBit(PTINA3,INA3);      // INA3 = 1
                    setBit(PTINB3,INB3);      // INB3 = 1
                    break;
                case 3:  // Motor 3 Brake to GND
                    clearBit(PTINA3,INA3);    // INA3 = 0
                    clearBit(PTINB3,INB3);    // INB3 = 0
                    break;
            }
            
            pwm3=pwm;
            dir3=dir;
            
            break;
            
        case 4:
        
            PWM4L = setPWM(pwm);
            //PWM4H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 4 Forward
                    setBit(PTINA4,INA4);       // INA4 = 1
                    clearBit(PTINB4,INB4);     // INB4 = 0
                    break;
                case 1:  // Motor 4 Backward
                    clearBit(PTINA4,INA4);     // INA4 = 0
                    setBit(PTINB4,INB4);       // INB4 = 1
                    break;
                case 2:  // Motor 4 Brake to VCC 
                    setBit(PTINA4,INA4);      // INA4 = 1
                    setBit(PTINB4,INB4);      // INB4 = 1
                    break;
                case 3:  // Motor 4 Brake to GND
                    clearBit(PTINA4,INA4);    // INA4 = 0
                    clearBit(PTINB4,INB4);    // INB4 = 0
                    break;
            }
            
            pwm4=pwm;
            dir4=dir;
            
            break;      
    }
}

unsigned char setPWM (unsigned char pwm)
{
    unsigned int ocr;

    if (pwm>100)
        pwm=100;
    else if (pwm<0)
        pwm=0;

    ocr=((unsigned int)255*pwm)/100;

    return (unsigned char)ocr;
}

void motorForward(void)
{
    motorControl(1, FORWARD, 100); 
    motorControl(2, FORWARD, 100);
    motorControl(3, FORWARD, 100);
    motorControl(4, FORWARD, 100);    
}

void motorBackward(void)
{
    motorControl(1, BACKWARD, 100); 
    motorControl(2, BACKWARD, 100);
    motorControl(3, BACKWARD, 100);
    motorControl(4, BACKWARD, 100);    
}

void motorRight(void)
{
    motorControl(1, FORWARD, 100); 
    motorControl(2, BACKWARD, 100);
    motorControl(3, BACKWARD, 100);
    motorControl(4, FORWARD, 100);    
}

void motorLeft(void)
{
    motorControl(1, BACKWARD, 100); 
    motorControl(2, FORWARD, 100);
    motorControl(3, FORWARD, 100);
    motorControl(4, BACKWARD, 100);    
}

