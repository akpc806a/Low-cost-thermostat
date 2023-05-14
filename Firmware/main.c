#include <stdbool.h>
//#include <stdio.h>

#include "pdkcommon.h"
#include "pfs173.h"


// microcontroller IO pins declarations
#define SERIAL_TX_PORT _pc
#define SERIAL_TX_PIN 0

#define LED_GREEN_PORT _pc
#define LED_GREEN_PIN 3

#define LED_RED_PORT _pa
#define LED_RED_PIN 7

#define LED_BLUE_PORT _pa
#define LED_BLUE_PIN 4

#define HEATER_1_PORT _pb
#define HEATER_1_PIN 6

#define HEATER_2_PORT _pb
#define HEATER_2_PIN 5

#define SERIAL_RX_PORT _pc
#define SERIAL_RX_PIN 1

#define _BV(x) (1 << (x))
#define PB3 3
#define PB1 1

// variables
volatile unsigned char sendcounter;
volatile unsigned int senddata;
volatile bool sending = false;
volatile bool test;

volatile bool receiving = false;
volatile bool received = false;
volatile unsigned char receive_counter = 0;
volatile unsigned int receive_data; // variable to store serial data while it is being received
volatile unsigned char received_data;

__sfr __at(0x23) _adcrl; // not in pfs173.h -- unique for PFS123 micro, bits 7:4 store bits 3:0 of the ADC conversion result


volatile bool timer_expired = false;
volatile unsigned short int timer_main_cycle = 0;

unsigned short int adc_heater1;
unsigned short int adc_heater2; // for future use
unsigned short int reference_heater1;

volatile bool preheated_heater1 = false;

long integral_heater1 = 0;

// periodic interrupt to send/receive a bit over serial link
void timer_interrupt(void) __interrupt(0)
{
  bool rx_bit;
  
  // Reset interrupt request, proceed only if we had a timer interrupt.
  if(!(_intrq & 0x40))
  {
    _intrq = 0x00;
    return;
  }
  _intrq = 0x00;
  
  timer_expired = true;


  rx_bit = SERIAL_RX_PORT & _BV(SERIAL_RX_PIN); // RXD signal state
  
  if (!receiving && !rx_bit)
  {
    // start bit
    receiving = true;
    receive_counter = 9;
    receive_data = 0;
  }
  else
  if (receiving)
  {
    receive_counter--;
    if (receive_counter == 0)
    {
      receiving = false;
      if (rx_bit) // stop bit
      {
        received = true;
        received_data = receive_data;
      }
    }
    else
    {
      receive_data >>= 1;
      if (rx_bit) 
        receive_data |= 0x80;
    }
  }
  

  if(!sending)
    return;
  
  if (senddata & 1)
    SERIAL_TX_PORT |= _BV(SERIAL_TX_PIN);
  else
    SERIAL_TX_PORT &= ~_BV(SERIAL_TX_PIN);

  senddata >>= 1;

  if(!--sendcounter)
  {
    sending = false;

    //_inten &= ~0x40;
  }
}

// send a character
void putchar(unsigned char c)
{
  while (sending) ;

  senddata = (c << 1) | 0x200;

  sendcounter = 10;

  _tm2ct = 0;

  sending = true;
}

// receive a character
unsigned char getchar(void)
{
  while (!received) ;
  
  received = false;

  return received_data;
}

short int Kp = 100;
short int Ki = 5;
short int Kd = 0;

#define maxOut 65535
#define minOut 0

long pidController(const short int actual, const short int reference, long* integral)
{
    short int error  = reference - actual;
	
    static short int error_prev = 0;
    short int differential = error - error_prev;
    long result = Kp * error + Ki * (*integral) + Kd * differential; // control output

    // integrator clamping: stop the integration if output is going to saturate
    if (((error > 0) && (result <= maxOut)) || ((error < 0) && (result >= minOut)))
    {
        *integral += error;
    }

    // clamp the output
    if (result > maxOut)
    {
        result = maxOut;
    }
    if (result < minOut)
    {
        result = minOut;
    }
    
    error_prev = error;

    return result;
}

void main(void)
{
  unsigned short int c;
  
  // setting up the microcontroller
  _ihrcr = *((const unsigned char*)(0x8bed)); // Use PFS173 factory calibration value for IHRC at 16 Mhz.
  _clkmd = 0x34; // Use IHRC / 2 = 8 Mhz for system clock, disable watchdog.
  _clkmd = 0x30; // Disable ILRC
  
  // Set timer 2 for interrupt for 9600 baud.
  _tm2c = 0x10; // Use CLK (8 Mhz)
  _tm2s = 0x06; // Divide by 6 + 1 ~> 1142857 Hz
  _tm2b = 118;  // Divide by 118 + 1 ~> 9604 Hz
  _inten = 0x40;
__asm
  engint
__endasm;

  // Set up the timer 3 for PWM
  _tm3c = 0x10 | 0x2 | (0xC); // Use CLK (8 Mhz), PWM mode, (PB7 output)
  _tm3s = 7; // Divide clock by (7 + 1) = 1 MHz (will give 1 MHz / 256 = 3.9 kHz PWM)
  _tm3b = 192;  // Duty cycle

  _pac = _BV(LED_RED_PIN) | _BV(LED_BLUE_PIN);
  _pbc = _BV(HEATER_1_PIN) | _BV(HEATER_2_PIN);
  _pcc = _BV(SERIAL_TX_PIN) | _BV(LED_GREEN_PIN);
  
  // turn off LEDs (they are logic high)
  LED_RED_PORT |= _BV(LED_RED_PIN);
  LED_GREEN_PORT |= _BV(LED_GREEN_PIN);
  LED_BLUE_PORT |= _BV(LED_BLUE_PIN);
  
  
  _pcdier = _BV(SERIAL_RX_PIN); // RXD
  
  
  // disable PGIO on analog input pins
  _pbc &= ~_BV(PB3);  
  _pbph &= ~_BV(PB3);
  _pbdier &= ~_BV(PB3);
  
  _pbc &= ~_BV(PB1);  
  _pbph &= ~_BV(PB1);
  _pbdier &= ~_BV(PB1);

  ADCM = ADCM_CLK_SYSCLK_DIV16; // 500 kHz ADC clock
  ADCC = ADCC_ADC_ENABLE | ADCC_CH_AD3_PB3; // Enable ADC
  ADCRGC = ADCRG_ADC_REF_VDD;
  
  
  _inten |= 0x40; // start interrupts from the soft uart timer

  // receiving commands and interpreting them
  for(;;)
  {
    if (received) // if we have a data to process
    {
      c = getchar();
      
      
      if (c == 'G')
      {
        c = getchar();
        if (c != '1')
          LED_GREEN_PORT |= _BV(LED_GREEN_PIN);
        else
          LED_GREEN_PORT &= ~_BV(LED_GREEN_PIN);
        
        putchar(c); // command execution acknowledgment
      }
      else
      if (c == 'R')
      {
        c = getchar();
        if (c != '1')
          LED_RED_PORT |= _BV(LED_RED_PIN);
        else
          LED_RED_PORT &= ~_BV(LED_RED_PIN);
        
        putchar(c); // command execution acknowledgment
      }
      else
      if (c == 'B')
      {
        c = getchar();
        if (c != '1')
          LED_BLUE_PORT |= _BV(LED_BLUE_PIN);
        else
          LED_BLUE_PORT &= ~_BV(LED_BLUE_PIN);
        
        putchar(c); // command execution acknowledgment
      }
      else
        
      if (c == 'H')
      {
        c = getchar();
        if (c == '1')
          HEATER_1_PORT |= _BV(HEATER_1_PIN);
        else
          HEATER_1_PORT &= ~_BV(HEATER_1_PIN);
        
        putchar(c); // command execution acknowledgment
      }
      
      else
      if (c == 'A')
      {
        c = getchar();

        putchar(c); // command execution acknowledgment
        
        putchar(adc_heater1 >> 8);
        putchar(adc_heater1);
      }
      else
      if (c == 'T')
      {
        c = getchar() << 8;
        c |= getchar();
        
        reference_heater1 = c;
        integral_heater1 = 0;
        preheated_heater1 = false;
        _tm3c = 0x10 | 0x2 | (0x8); // PB6 output
        
        // command execution acknowledgment
        putchar(reference_heater1 >> 8); 
        putchar(reference_heater1);
      }
      else
      if (c == 'P')
      {
        c = getchar() << 8;
        c |= getchar();
        
        Kp = c;

        // command execution acknowledgment
        putchar(Kp >> 8); 
        putchar(Kp);
      }
      else
      if (c == 'I')
      {
        c = getchar() << 8;
        c |= getchar();
        
        Ki = c;

        // command execution acknowledgment
        putchar(Ki >> 8); 
        putchar(Ki);
      }
      else
      if (c == 'D')
      {
        c = getchar() << 8;
        c |= getchar();
        
        Kd = c;

        // command execution acknowledgment
        putchar(Kd >> 8); 
        putchar(Kd);
      }
    }
    
    if (timer_expired)
    {
      timer_expired = false;
      
      timer_main_cycle++;
      if (timer_main_cycle == 960)
      {
        timer_main_cycle = 0;
        
        LED_GREEN_PORT |= _BV(LED_GREEN_PIN);
        
        // converting ADCs
        ADCC = ADCC_ADC_ENABLE | ADCC_CH_AD1_PB1; // enable ADC for this heater 1 pin
    
        ADCC |= _BV(6); // start conversion

        do {} while ((ADCC & _BV(6)) == 0); // wait for the conversion to finish
        
        adc_heater1 = (ADCR << 4) | (_adcrl >> 4);

        
        ADCC = ADCC_ADC_ENABLE | ADCC_CH_AD3_PB3; // enable ADC for this heater 2 pin
    
        ADCC |= _BV(6); // start conversion

        do {} while ((ADCC & _BV(6)) == 0); // wait for the conversion to finish

        adc_heater2 = (ADCR << 4) | (_adcrl >> 4);
        
        if (reference_heater1 != 0)
        {
          if (preheated_heater1)
          {
            // reference and actual are swapped which is equivalent to multiplying the error by -1 (in this particular circuit implementation the higher temperature corresponds to lower ADC voltage)
            _tm3b = pidController(reference_heater1, adc_heater1, &integral_heater1) >> 8; 
          }
          else
          {
            // we turn on PID control only when we are reasonably close to the target temperature, otherwise we just turn on the heater to heat it up
            if ( (adc_heater1 - reference_heater1) > 100 )
              _tm3b = 0xFF;
            else
              preheated_heater1 = true;
          }
        }
        else
          _tm3b = 0;
          
        
        LED_GREEN_PORT &= ~_BV(LED_GREEN_PIN);

        putchar(0x55);
        putchar(0xAA);        
        putchar(adc_heater1 >> 8);
        putchar(adc_heater1);
        putchar(adc_heater2 >> 8);
        putchar(adc_heater2);
      }
    }
  }
}