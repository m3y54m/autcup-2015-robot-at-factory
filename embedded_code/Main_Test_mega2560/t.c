/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
� Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 09/21/2015
Author  : 
Company : 
Comments: 


Chip type               : ATmega2560
Program type            : Application
AVR Core Clock frequency: 14.745600 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 2048
*******************************************************/

#include <mega2560.h>

#include <delay.h>

// Alphanumeric LCD functions
#include <alcd.h>

// Declare your global variables here
unsigned char BYTE_L;
unsigned char BYTE_H;

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

void motorControl(unsigned char motor, unsigned char dir, unsigned char pwm);
unsigned int setPWM (unsigned char pwm);
void motorForward(void);
void motorBackward(void);
void motorRight(void);
void motorLeft(void);

#define INA1   PORTG5
#define INB1   PORTH2
#define PINA1  PORTG   // PORT OF INA1
#define PINB1  PORTH   // PORT OF INB1
#define PWM1H  OCR3AH
#define PWM1L  OCR3AL
#define EN1    PORTH0

#define INA2   PORTE2
#define INB2   PORTE6
#define PINA2  PORTE   // PORT OF INA2
#define PINB2  PORTE   // PORT OF INB2
#define PWM2H  OCR3BH
#define PWM2L  OCR3BL
#define EN2    PORTH1

#define INA3   PORTB6
#define INB3   PORTH7
#define PINA3  PORTB   // PORT OF INA3
#define PINB3  PORTH   // PORT OF INB3
#define PWM3H  OCR4AH
#define PWM3L  OCR4AL
#define EN3    PORTB4

#define INA4   PORTB7
#define INB4   PORTG3
#define PINA4  PORTB   // PORT OF INA4
#define PINB4  PORTG   // PORT OF INB4
#define PWM4H  OCR4BH
#define PWM4L  OCR4BL
#define EN4    PORTB5

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

// Get a character from the USART3 Receiver
#pragma used+
char getchar3(void)
{
char status,data;
while (1)
      {
      while (((status=UCSR3A) & RX_COMPLETE)==0);
      data=UDR3;
      if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
         return data;
      }
}
#pragma used-

// Write a character to the USART3 Transmitter
#pragma used+
void putchar3(char c)
{
while ((UCSR3A & DATA_REGISTER_EMPTY)==0);
UDR3=c;
}
#pragma used-

// Standard Input/Output functions
#include <stdio.h>

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=(adc_input & 0x1f) | ADC_VREF_TYPE;
if (adc_input & 0x20) ADCSRB|=(1<<MUX5);
else ADCSRB&=~(1<<MUX5);
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}

void main(void)
{
// Declare your local variables here

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(1<<DDB7) | (1<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=P Bit4=P Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (1<<PORTB5) | (1<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=Out Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In 
DDRE=(0<<DDE7) | (1<<DDE6) | (0<<DDE5) | (1<<DDE4) | (1<<DDE3) | (1<<DDE2) | (0<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=0 Bit5=T Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit5=Out Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In 
DDRG=(1<<DDG5) | (0<<DDG4) | (1<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit5=0 Bit4=T Bit3=0 Bit2=T Bit1=T Bit0=T 
PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Port H initialization
// Function: Bit7=Out Bit6=In Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In 
DDRH=(1<<DDH7) | (0<<DDH6) | (1<<DDH5) | (1<<DDH4) | (1<<DDH3) | (1<<DDH2) | (0<<DDH1) | (0<<DDH0);
// State: Bit7=0 Bit6=T Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=P Bit0=P 
PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (0<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (1<<PORTH1) | (1<<PORTH0);

// Port J initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (0<<DDJ1) | (0<<DDJ0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);

// Port K initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRK=(0<<DDK7) | (0<<DDK6) | (0<<DDK5) | (0<<DDK4) | (0<<DDK3) | (0<<DDK2) | (0<<DDK1) | (0<<DDK0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<PORTK5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);

// Port L initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
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
// OC2A output: Disconnected
// OC2B output: Disconnected
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: 14745.600 kHz
// Mode: Ph. correct PWM top=0x01FF
// OC3A output: Non-Inverted PWM
// OC3B output: Non-Inverted PWM
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.069309 ms
// Output Pulse(s):
// OC3A Period: 0.069309 ms Width: 0 us// OC3B Period: 0.069309 ms Width: 0 us
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(1<<COM3A1) | (0<<COM3A0) | (1<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (1<<WGM31) | (0<<WGM30);
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

// Timer/Counter 4 initialization
// Clock source: System Clock
// Clock value: 14745.600 kHz
// Mode: Ph. correct PWM top=0x01FF
// OC4A output: Non-Inverted PWM
// OC4B output: Non-Inverted PWM
// OC4C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.069309 ms
// Output Pulse(s):
// OC4A Period: 0.069309 ms Width: 0 us// OC4B Period: 0.069309 ms Width: 0 us
// Timer4 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR4A=(1<<COM4A1) | (0<<COM4A0) | (1<<COM4B1) | (0<<COM4B0) | (0<<COM4C1) | (0<<COM4C0) | (1<<WGM41) | (0<<WGM40);
TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (1<<CS40);
TCNT4H=0x00;
TCNT4L=0x00;
ICR4H=0x00;
ICR4L=0x00;
OCR4AH=0x00;
OCR4AL=0x00;
OCR4BH=0x00;
OCR4BL=0x00;
OCR4CH=0x00;
OCR4CL=0x00;

// Timer/Counter 5 initialization
// Clock source: System Clock
// Clock value: Timer5 Stopped
// Mode: Normal top=0xFFFF
// OC5A output: Disconnected
// OC5B output: Disconnected
// OC5C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer5 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR5A=(0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) | (0<<COM5C1) | (0<<COM5C0) | (0<<WGM51) | (0<<WGM50);
TCCR5B=(0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (0<<WGM52) | (0<<CS52) | (0<<CS51) | (0<<CS50);
TCNT5H=0x00;
TCNT5L=0x00;
ICR5H=0x00;
ICR5L=0x00;
OCR5AH=0x00;
OCR5AL=0x00;
OCR5BH=0x00;
OCR5BL=0x00;
OCR5CH=0x00;
OCR5CL=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

// Timer/Counter 3 Interrupt(s) initialization
TIMSK3=(0<<ICIE3) | (0<<OCIE3C) | (0<<OCIE3B) | (0<<OCIE3A) | (0<<TOIE3);

// Timer/Counter 4 Interrupt(s) initialization
TIMSK4=(0<<ICIE4) | (0<<OCIE4C) | (0<<OCIE4B) | (0<<OCIE4A) | (0<<TOIE4);

// Timer/Counter 5 Interrupt(s) initialization
TIMSK5=(0<<ICIE5) | (0<<OCIE5C) | (0<<OCIE5B) | (0<<OCIE5A) | (0<<TOIE5);

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
// PCINT0 interrupt: Off
// PCINT1 interrupt: Off
// PCINT2 interrupt: Off
// PCINT3 interrupt: Off
// PCINT4 interrupt: Off
// PCINT5 interrupt: Off
// PCINT6 interrupt: Off
// PCINT7 interrupt: Off
// PCINT8 interrupt: Off
// PCINT9 interrupt: Off
// PCINT10 interrupt: Off
// PCINT11 interrupt: Off
// PCINT12 interrupt: Off
// PCINT13 interrupt: Off
// PCINT14 interrupt: Off
// PCINT15 interrupt: Off
// PCINT16 interrupt: Off
// PCINT17 interrupt: Off
// PCINT18 interrupt: Off
// PCINT19 interrupt: Off
// PCINT20 interrupt: Off
// PCINT21 interrupt: Off
// PCINT22 interrupt: Off
// PCINT23 interrupt: Off
PCMSK0=(0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);
PCMSK1=(0<<PCINT15) | (0<<PCINT14) | (0<<PCINT13) | (0<<PCINT12) | (0<<PCINT11) | (0<<PCINT10) | (0<<PCINT9) | (0<<PCINT8);
PCMSK2=(0<<PCINT23) | (0<<PCINT22) | (0<<PCINT21) | (0<<PCINT20) | (0<<PCINT19) | (0<<PCINT18) | (0<<PCINT17) | (0<<PCINT16);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

// USART0 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART0 Receiver: On
// USART0 Transmitter: On
// USART0 Mode: Asynchronous
// USART0 Baud Rate: 115200
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x07;

// USART1 initialization
// USART1 disabled
UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

// USART2 initialization
// USART2 disabled
UCSR2B=(0<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (0<<RXEN2) | (0<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);

// USART3 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART3 Receiver: On
// USART3 Transmitter: On
// USART3 Mode: Asynchronous
// USART3 Baud Rate: 9600
UCSR3A=(0<<RXC3) | (0<<TXC3) | (0<<UDRE3) | (0<<FE3) | (0<<DOR3) | (0<<UPE3) | (0<<U2X3) | (0<<MPCM3);
UCSR3B=(0<<RXCIE3) | (0<<TXCIE3) | (0<<UDRIE3) | (1<<RXEN3) | (1<<TXEN3) | (0<<UCSZ32) | (0<<RXB83) | (0<<TXB83);
UCSR3C=(0<<UMSEL31) | (0<<UMSEL30) | (0<<UPM31) | (0<<UPM30) | (0<<USBS3) | (1<<UCSZ31) | (1<<UCSZ30) | (0<<UCPOL3);
UBRR3H=0x00;
UBRR3L=0x5F;

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// ADC initialization
// ADC Clock frequency: 460.800 kHz
// ADC Voltage Reference: AVCC pin
// ADC Auto Trigger Source: ADC Stopped
// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
// ADC4: On, ADC5: On, ADC6: On, ADC7: On
DIDR0=(0<<ADC7D) | (0<<ADC6D) | (0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
// Digital input buffers on ADC8: On, ADC9: On, ADC10: On, ADC11: On
// ADC12: On, ADC13: On, ADC14: On, ADC15: On
DIDR2=(0<<ADC15D) | (0<<ADC14D) | (0<<ADC13D) | (0<<ADC12D) | (0<<ADC11D) | (0<<ADC10D) | (0<<ADC9D) | (0<<ADC8D);
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0);
ADCSRB=(0<<MUX5) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// Mode: TWI Master
initI2C();

// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTC Bit 0
// RD - PORTC Bit 1
// EN - PORTC Bit 2
// D4 - PORTC Bit 4
// D5 - PORTC Bit 5
// D6 - PORTC Bit 6
// D7 - PORTC Bit 7
// Characters/line: 16
lcd_init(16);
      
// Global enable interrupts
#asm("sei")

while (1)
      {
      // Place your code here
      
      char str[32];
      static signed char k=10;

/*************************Motor Control**********************************/      
      char i=4;
      
//      motorControl(i, 0, 100);
      lcd_clear();
      lcd_puts("Forward");

      motorForward();      
      delay_ms(1000);
      
//      motorControl(i, 2, 100);
      lcd_clear();
      lcd_puts("Right");
      
      motorRight();
      delay_ms(1000);
      
//      motorControl(i, 1, 70);
      lcd_clear();
      lcd_puts("Backward");
      
      motorBackward();
      delay_ms(1000);
      
//      motorControl(i, 3, 70);
      lcd_clear();
      lcd_puts("Left");

      motorLeft();
      delay_ms(1000);
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
    TWBR = 32; /* set bit rate, see p. 242 */
    /* 8MHz / (16+2*TWBR*1) ~= 100kHz */
    TWCR |= (1 << TWEN); /* enable */
}

void i2cWaitForComplete(void) {
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
        
            PWM1L = (char)(setPWM(pwm) & 0x00FF);
            PWM1H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 1 Forward
                    setBit(PINA1,INA1);       // INA1 = 1
                    clearBit(PINB1,INB1);     // INB1 = 0
                    break;
                case 1:  // Motor 1 Backward
                    clearBit(PINA1,INA1);    // INA1 = 0
                    setBit(PINB1,INB1);      // INB1 = 1
                    break;
                case 2:  // Motor 1 Brake to VCC 
                    setBit(PINA1,INA1);      // INA1 = 1
                    setBit(PINB1,INB1);      // INB1 = 1
                    break;
                case 3:  // Motor 1 Brake to GND
                    clearBit(PINA1,INA1);    // INA1 = 0
                    clearBit(PINB1,INB1);    // INB1 = 0
                    break;
            }
            
            pwm1=pwm;
            dir1=dir;
            
            break;
            
        case 2:
        
            PWM2L = (char)(setPWM(pwm) & 0x00FF);
            PWM2H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 2 Forward
                    setBit(PINA2,INA2);       // INA2 = 1
                    clearBit(PINB2,INB2);     // INB2 = 0
                    break;
                case 1:  // Motor 2 Backward
                    clearBit(PINA2,INA2);    // INA2 = 0
                    setBit(PINB2,INB2);      // INB2 = 1
                    break;
                case 2:  // Motor 2 Brake to VCC 
                    setBit(PINA2,INA2);      // INA2 = 1
                    setBit(PINB2,INB2);      // INB2 = 1
                    break;
                case 3:  // Motor 2 Brake to GND
                    clearBit(PINA2,INA2);    // INA2 = 0
                    clearBit(PINB2,INB2);    // INB2 = 0
                    break;
            }
            
            pwm2=pwm;
            dir2=dir;
            
            break;
            
        case 3:
        
            PWM3L = (char)(setPWM(pwm) & 0x00FF);
            PWM3H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 3 Forward
                    setBit(PINA3,INA3);        // INA3 = 1
                    clearBit(PINB3,INB3);      // INB3 = 0
                    break;
                case 1:  // Motor 3 Backward
                    clearBit(PINA3,INA3);     // INA3 = 0
                    setBit(PINB3,INB3);       // INB3 = 1
                    break;
                case 2:  // Motor 3 Brake to VCC 
                    setBit(PINA3,INA3);      // INA3 = 1
                    setBit(PINB3,INB3);      // INB3 = 1
                    break;
                case 3:  // Motor 3 Brake to GND
                    clearBit(PINA3,INA3);    // INA3 = 0
                    clearBit(PINB3,INB3);    // INB3 = 0
                    break;
            }
            
            pwm3=pwm;
            dir3=dir;
            
            break;
            
        case 4:
        
            PWM4L = (char)(setPWM(pwm) & 0x00FF);
            PWM4H = (char)(setPWM(pwm) >> 8);
            
            switch (dir)
            {
                case 0:  // Motor 4 Forward
                    setBit(PINA4,INA4);       // INA4 = 1
                    clearBit(PINB4,INB4);     // INB4 = 0
                    break;
                case 1:  // Motor 4 Backward
                    clearBit(PINA4,INA4);     // INA4 = 0
                    setBit(PINB4,INB4);       // INB4 = 1
                    break;
                case 2:  // Motor 4 Brake to VCC 
                    setBit(PINA4,INA4);      // INA4 = 1
                    setBit(PINB4,INB4);      // INB4 = 1
                    break;
                case 3:  // Motor 4 Brake to GND
                    clearBit(PINA4,INA4);    // INA4 = 0
                    clearBit(PINB4,INB4);    // INB4 = 0
                    break;
            }
            
            pwm4=pwm;
            dir4=dir;
            
            break;      
    }
}

unsigned int setPWM (unsigned char pwm)
{
    unsigned int ocr;

    if (pwm>100)
        pwm=100;
    else if (pwm<0)
        pwm=0;

    ocr=((unsigned long)511*pwm)/100;

    return ocr;
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
