/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 10/15/2015
Author  : 
Company : 
Comments: 


Chip type               : ATmega8A
Program type            : Application
AVR Core Clock frequency: 7.372800 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

#include <mega8.h>
#include <delay.h>

#define encoderA PIND.2
#define encoderB PIND.3
#define maxSpeed 135
#define RATIO (127.0F / maxSpeed)

//Wheels: 617 pulse per revolution
bit countStart=0;
signed int position=0;  // pulses count with direction
signed int lastPosition=0;
signed int speed=0;     // pulses per n*10ms
signed char data=0;

// Standard Input/Output functions
#include <stdio.h>

// TWI Slave receive buffer
#define TWI_RX_BUFFER_SIZE 1
unsigned char twi_rx_buffer=0;

// TWI Slave transmit buffer
#define TWI_TX_BUFFER_SIZE 1
unsigned char twi_tx_buffer=0;

// TWI Slave 7-bit address
// Shaft Encoder 1
#define TWI_SLAVE_ADDR 0x01

bit twi_tx_flag=0;
bit twi_rx_flag=0;

interrupt [TWI] void twi_interrupt(void)
{
   unsigned char twi_status;
   #asm("wdr")
   
   if (countStart==1)
   {     
        //Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”
        TWDR=-128; //error: data not ready
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
   }
   else
   {
       twi_status=TWSR & 0xF8;
       //printf("%X\r\n",twi_status);
             
       switch (twi_status)
       {
          // Own SLA+W has been received; ACK has been returned
          case 0x60:
          // General call address has been received; ACK has been returned
          case 0x70:
             // Data byte will be received and ACK will be returned 
             TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
             break;
             
          // Previously addressed with own SLA+W; data has been received; ACK has been returned
          case 0x80:
          // Previously addressed with general call; data has been received; ACK has been returned
          case 0x90:
             // Data byte will be received and ACK will be returned
             twi_rx_buffer = TWDR;
             twi_rx_flag = 1; 
             TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
             //printf("Received=%d\r\n",twi_rx_buffer);
             break;
             
          // Own SLA+R has been received; ACK has been returned
          case 0xA8:
             switch (twi_rx_buffer)
             {
                 case 1:
                    twi_tx_buffer = data;
                    break;
             }
             
             // Load data byte
             TWDR = twi_tx_buffer;
             twi_tx_flag = 1;
             // Last data byte will be transmitted and NOT ACK should be received
             TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
             //printf("Transmitted=%d\r\n",twi_tx_buffer);
             break;
          
          // A STOP condition or repeated START condition has been received while still addressed as slave
          case 0xA0:
          // Data byte in TWDR has been transmitted; NOT ACK has been received
          case 0xC0:
          // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
          case 0xC8:
             //Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”
             TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
             break;
                
          // Data byte in TWDR has been transmitted; ACK has been received
          case 0xB8:
             // Last data byte will be transmitted and NOT ACK should be received
             TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
             break;         
       }
   }
}

unsigned char lastStateA = 0;
unsigned char currentStateB = 0;

// External Interrupt 0 service routine
interrupt [EXT_INT0] void ext_int0_isr(void)
{
  if (currentStateB ^ lastStateA)
    position++;
  else
    position--;

  lastStateA = encoderA;

  //printf("%d\r\n",position);         
}


// External Interrupt 1 service routine
interrupt [EXT_INT1] void ext_int1_isr(void)
{
  currentStateB = encoderB; 
  
  if (currentStateB ^ lastStateA)
    position++;
  else
    position--; 

  //printf("%d\r\n",position);
}

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
static unsigned char n=0;
// Reinitialize Timer 0 value
TCNT0=0xB8;
// EACH INTERRUPT = 10ms

if (n==4)  //4*10ms
{
    n=0;
    if (countStart==0)
    {
        countStart=1;
        lastPosition = position;
        position = 0;  // reset position
    }
    else
    {   
        int temp;
        
        speed = position - lastPosition;
        
        temp = (signed int)(RATIO * speed);
        
        if (temp > 127) temp = 127;     //overflow
        if (temp < -127) temp = -127;   //overflow // -128 reserved for error situations
        
        data = temp;
        
        countStart=0;
    }
}

n++;

}

void main(void)
{

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=P Bit4=P Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC6) | (1<<PORTC5) | (1<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 7.200 kHz
TCCR0=(1<<CS02) | (0<<CS01) | (1<<CS00);
TCNT0=0xB8;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (1<<TOIE0);

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Any change
// INT1: On
// INT1 Mode: Any change
GICR|=(1<<INT1) | (1<<INT0);
MCUCR=(0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);
GIFR=(1<<INTF1) | (1<<INTF0);

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 115200
UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
UBRRH=0x00;
UBRRL=0x03;

// TWI initialization
// Mode: TWI Slave
// Match Any Slave Address: Off
// I2C Bus Slave Address: 0x01
       
// TWGCE=1 means TWI module will respond to general call address (0x00)
TWAR=(TWI_SLAVE_ADDR<<1) | (0<<TWGCE);
// Enable TWI module in Slave mode and enable TWI interrupt    
TWCR=(1<<TWEA) | (1<<TWEN) | (1<<TWIE);

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/1024k
#pragma optsize-
WDTCR=(1<<WDCE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
WDTCR=(0<<WDCE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Global enable interrupts
#asm("sei")

while (1);

}
