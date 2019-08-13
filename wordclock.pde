#include <stdio.h>
#include <string.h>
#include <DS1302.h>  // I use the library that Matt Sparks created
#include <avr/interrupt.h>
#include <avr/io.h>

// time test 1:  3:38:50clock=10:34:11 real - +3:39
//   13:04:40 7:59:59 +3:40

/**************************************************************************
 *                                                                         *
 *  W O R D C L O C K   - A clock that tells the time using words.         *
 *                                                                         *
 * Hardware: Arduino Dumelove with a set of individual LEDs under a word   *
 *            stencil.                                                     *
 *                                                                         *
 *   Copyright (C) 2009  Doug Jackson (doug@doughq.com)                    *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU General Public License as published by    *
 * the Free Software Foundation; either version 2 of the License, or       *
 * (at your option) any later version.                                     *
 *                                                                         *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU General Public License for more details.                            *
 *                                                                         *
 * You should have received a copy of the GNU General Public License       *
 * along with this program; if not, write to the Free Software             *
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,                   *
 * MA  02111-1307  USA                                                     *
 *                                                                         *
 ***************************************************************************
 * 
 * Revision History
 * 
 * Date  	By	What
 * 20010205	DRJ	Initial Creation of Arduino Version 
 *                      - based on Wordclock.c - from PIC version
 * 20100428     DRJ     Added support for DS1302 RTC Chip - Code verifies 
 *                      device is present - and if so, uses it for timekeeping
 * 20100511     DRJ     Added ability to detect old vs new Arduino boards
 *                      New board uses diferent time button sense.
 * 20100511     DRJ     Minute LED support on Analog5-1 pins.
 * 20100927     DRJ     Added brightness support by enabling interrupts and 
 *                      using an ISR to control the display
 */

// DAY Brightness setting 0 = off    20 = full
#define DAYBRIGHTNESS 20
// NIGHT Brightness setting 0 = off    20 = full
#define NIGHTBRIGHTNESS 20


// Display output pin assignments
#define MINUTES	Display1=Display1 | (1<<0)
#define MTEN 	Display1=Display1 | (1<<1)  
#define HALF	Display1=Display1 | (1<<2)
#define PAST	Display1=Display1 | (1<<3)
#define THREE	Display1=Display1 | (1<<4)
#define ITIS	Display1=Display1 | (1<<5)
#define TWENTY	Display1=Display1 | (1<<6)
#define TO	Display1=Display1 | (1<<7)

#define TWO	Display2=Display2 | (1<<0)
#define SIX	Display2=Display2 | (1<<1)
#define TWELVE	Display2=Display2 | (1<<2)
#define HFIVE	Display2=Display2 | (1<<3)
#define SEVEN	Display2=Display2 | (1<<4)
#define OCLOCK	Display2=Display2 | (1<<5)
#define ONE	Display2=Display2 | (1<<6)
#define QUARTER	Display2=Display2 | (1<<7)

#define EIGHT	Display3=Display3 | (1<<0)
#define MFIVE	Display3=Display3 | (1<<1)
#define ARDUINO	Display3=Display3 | (1<<2)
#define ELEVEN	Display3=Display3 | (1<<3)
#define HTEN	Display3=Display3 | (1<<4)
#define NINE	Display3=Display3 | (1<<5)
#define FOUR	Display3=Display3 | (1<<6)
#define DOUG	Display3=Display3 | (1<<7)

#define LED1    Led1=1
#define LED2    Led2=1
#define LED3    Led3=1
#define LED4    Led4=1



#define INIT_TIMER_COUNT 6
#define RESET_TIMER2 TCNT2 = INIT_TIMER_COUNT




int  hour=3, minute=38, second=00;
static unsigned long msTick =0;  // the number of Millisecond Ticks since we last 
// incremented the second counter
int  count;
int  selftestmode;          // 1 = in self test - flash display
int  DS1302Present=0;       // flag to indicate that the 1302 is there..    1 = present
char Display1=0, Display2=0, Display3=0, Led1=0, Led2=0, Led3=0, Led4=0;
int  OldHardware = 0;  // 1 = we are running on old hardwrae
int  BTNActive = 1;    // the sense of the button inputs (Changes based on hardware type)
int  timercount=10;    // used for interrupt counting to determine the brightnes of the display
                        
// hardware constants
int LEDClockPin=5;  // Arduino Pin#11 - 4094 Pin 3 clock
int LEDDataPin=3;   // Arduino Pin#5  - 4094 pin 2 Data
int LEDStrobePin=4; // Arduino Pin#6  - 4094 pin 1 Strobe

// buttons
int FWDButtonPin=6;
int REVButtonPin=7;

// 1302 RTC Constants
int DS1302IOPin=10;
int DS1302CEPin=8;
int DS1302CLKPin=9;

// Minute LED Pins
int LED1PIN=19;  // Arduino analog 5
int LED2PIN=18;  // Arduino analog 4
int LED3PIN=17;  // Arduino analog 3
int LED4PIN=16;  // Arduino analog 2

int current_brightnes=0;

/* Create buffers */
char buf[50]; // time output string for debugging


// create an object that talks to the RTC
DS1302 rtc(DS1302CEPin, DS1302IOPin, DS1302CLKPin);

void print_DS1302time()
{
  /* Get the current time and date from the chip */
  Time t = rtc.time();

  /* Format the time and date and insert into the temporary buffer */
  snprintf(buf, sizeof(buf), "DS1302 time: %02d:%02d:%02d",
  t.hr, t.min, t.sec);

  /* Print the formatted string to serial so we can see the time */
  Serial.println(buf);

}



void setup()
{
  // initialise the hardware	
  // initialize the appropriate pins as outputs:
  pinMode(LEDClockPin, OUTPUT); 
  pinMode(LEDDataPin, OUTPUT); 
  pinMode(LEDStrobePin, OUTPUT); 
  pinMode(FWDButtonPin, INPUT); 
  pinMode(REVButtonPin, INPUT); 

  //  setup 1302
  pinMode(DS1302IOPin, OUTPUT); 
  pinMode(DS1302CEPin, OUTPUT); 
  pinMode(DS1302CLKPin, OUTPUT); 

  // Minute LEDS
  pinMode(LED1PIN, OUTPUT);
  pinMode(LED2PIN, OUTPUT);
  pinMode(LED3PIN, OUTPUT);
  pinMode(LED4PIN, OUTPUT);

  current_brightnes=DAYBRIGHTNESS;
  

  Serial.begin(9600);   // setup the serial port to 9600 baud
  SWversion();          // Display the version number of the software

  // test whether the DS1302 is there
  Serial.print("Verifying DS1302 ");
  // start by verifying that the chip has a valid signature
  if (rtc.read_register(0x20) == 0xa5) {
    // Signature is there - set the present flag and mmove on
    DS1302Present=1;
    Serial.println("present - Valid Signature");
    rtc.write_protect(false);
    rtc.halt(false);
  }
  else 
  {
    // Signature isnt there - may be a new chip - 
    //   do a write to see if it will hold the signature 
    rtc.write_register(0x20,0xa5);
    if (rtc.read_register(0x20) == 0xa5) {
      // We can store data - assume that it is a new chip that needs initialisation
      // Start by turning off write protection and clearing the clock halt flag.
      rtc.write_protect(false);
      rtc.halt(false);
      // Make a new time object to set the date and time 
      Time t(2010, 04, 28, 10, 19, 00, 01);
      // Set the time and date on the chip 
      rtc.time(t);
      // set the DS1302 present flag
      DS1302Present=1;
      Serial.println("present - new chip initialised.");
    }
    else  Serial.println("absent");
  }  

  // determine whether we are running on old or new hardware
  // old hardware tied the push buttons to ground using 4k7 resistors
  // and relied on the buttons to pull them high
  // new hardware uses internal pullups, and uses the buttons
  // to pull the inputs down.

  digitalWrite(FWDButtonPin,HIGH);  // Turn on weak pullups
  digitalWrite(REVButtonPin,HIGH);  // Turn on weak pullups

  OldHardware=0;
  if ( digitalRead(FWDButtonPin)==0 && digitalRead(REVButtonPin)==0)
  {
    Serial.println("Detected Old Hardware");
    OldHardware=1;  // we have old hardware
    BTNActive = 1; // True = active for old hardware
    digitalWrite(FWDButtonPin,LOW);  // Turn off weak pullups
    digitalWrite(REVButtonPin,LOW);  // Turn off weak pullups

  }
  else
  {
    Serial.println("Detected New Hardware");
    OldHardware=0;  // we have old hardware
    BTNActive = 0; // True = active for old hardware
  }


  // Initialise Timer2: Timer Prescaler /64,
  TCCR2A = 0;
  TCCR2B |= (1<<CS22);
  TCCR2B &= ~((1<<CS21) | (1<<CS20));
  // Use normal mode
  TCCR2B &= ~((1<<WGM21) | (1<<WGM20));
  // Use internal clock - external clock not used in Arduino
  ASSR |= (0<<AS2);
  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
  RESET_TIMER2;
  sei();

   Serial.println("Finished setting up Timer2 Interrupt");


  msTick=millis();      // Initialise the msTick counter

  selftest();  // validate the hardware for the user

  selftestmode=0;

  if (DS1302Present==1) {
    // Get the current time and date from the chip 
    Time t = rtc.time();
    second=t.sec;     
    minute=t.min;
    hour=t.hr; 
  }

  displaytime();        // display the current time
}


// Interrupt handler - Arduino runs at 16 Mhz, so we have 1000 Overflows per second...
// 1/ ((16000000 / 64) / 256) = 1 / 1000
ISR(TIMER2_OVF_vect) {
  RESET_TIMER2;
  
  
  if (timercount-- <= current_brightnes){
    // Now we write the actual values to the hardware
    shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display3);
    shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display2);
    shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display1);
    digitalWrite(LEDStrobePin,HIGH);
    //delay(2);
    digitalWrite(LEDStrobePin,LOW);    
    digitalWrite(LED1PIN,Led1);
    digitalWrite(LED2PIN,Led2);
    digitalWrite(LED3PIN,Led3);
    digitalWrite(LED4PIN,Led4);
  }
  else
    {
    shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, 0);
    shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, 0);
    shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, 0);    
    digitalWrite(LEDStrobePin,HIGH);
    //delay(2);
    digitalWrite(LEDStrobePin,LOW);    
    digitalWrite(LED1PIN,0);
    digitalWrite(LED2PIN,0);
    digitalWrite(LED3PIN,0);
    digitalWrite(LED4PIN,0);
    
    }  
  if (timercount==0) timercount=20;
  


}



void ledsoff(void) {
  Display1=0;
  Display2=0;
  Display3=0;
  Led1=0;
  Led2=0;
  Led3=0;
  Led4=0;
}



void selftest(void){
  // start by clearing the display to a known state
  ledsoff(); 
  ITIS;    
  delay(500); 
  ledsoff(); 
  MTEN;    
  delay(500); 
  ledsoff(); 
  HALF;    
  delay(500); 
  ledsoff(); 
  TWENTY;  
  delay(500); 
  ledsoff(); 
  QUARTER; 
  delay(500); 
  ledsoff(); 
  MFIVE;   
  delay(500); 
  ledsoff(); 
  MINUTES; 
  delay(500); 
  ledsoff(); 
  PAST;    
  delay(500); 
  ledsoff(); 
  TO;      
  delay(500); 
  ledsoff(); 
  ONE;     
  delay(500); 
  ledsoff(); 
  TWO;     
  delay(500); 
  ledsoff(); 
  THREE;   
  delay(500); 
  ledsoff(); 
  FOUR;    
  delay(500); 
  ledsoff(); 
  HFIVE;   
  delay(500); 
  ledsoff(); 
  SIX;     
  delay(500); 
  ledsoff(); 
  SEVEN;   
  delay(500); 
  ledsoff(); 
  EIGHT;   
  delay(500); 
  ledsoff(); 
  NINE;    
  delay(500); 
  ledsoff(); 
  HTEN;    
  delay(500); 
  ledsoff(); 
  ELEVEN;  
  delay(500); 
  ledsoff(); 
  TWELVE;  
  delay(500); 
  ledsoff(); 
  OCLOCK;  
  delay(500); 
  ledsoff(); 
  DOUG;    
  delay(500); 
  ledsoff(); 
  ARDUINO; 
  delay(500); 
  ledsoff(); 
  LED1;    
  delay(500);  
  ledsoff(); 
  LED2;    
  delay(500);  
  ledsoff(); 
  LED3;    
  delay(500);  
  ledsoff(); 
  LED4;    
  delay(500); 

  for(int i=0; i<5; i++)
  {
    Display1=255; 
    Display2=255; 
    Display3=255;
    delay(500);
    ledsoff(); 
    delay(500);
  }

}


void displaytime(void){

  // start by clearing the display to a known state
  ledsoff();

  // Now, turn on the "It is" leds
  ITIS;
  Serial.print("It is ");

  // now we display the appropriate minute counter
  if ((minute>4) && (minute<10)) { 
    MFIVE; 
    MINUTES; 
    Serial.print("Five Minutes ");
  } 
  if ((minute>9) && (minute<15)) { 
    MTEN; 
    MINUTES; 
    Serial.print("Ten Minutes ");
  }
  if ((minute>14) && (minute<20)) {
    QUARTER; 
    Serial.print("Quarter ");
  }
  if ((minute>19) && (minute<25)) { 
    TWENTY; 
    MINUTES; 
    Serial.print("Twenty Minutes ");
  }
  if ((minute>24) && (minute<30)) { 
    TWENTY; 
    MFIVE; 
    MINUTES;
    Serial.print("Twenty Five Minutes ");
  }  
  if ((minute>29) && (minute<35)) {
    HALF;
    Serial.print("Half ");
  }
  if ((minute>34) && (minute<40)) { 
    TWENTY; 
    MFIVE; 
    MINUTES;
    Serial.print("Twenty Five Minutes ");
  }  
  if ((minute>39) && (minute<45)) { 
    TWENTY; 
    MINUTES; 
    Serial.print("Twenty Minutes ");
  }
  if ((minute>44) && (minute<50)) {
    QUARTER; 
    Serial.print("Quarter ");
  }
  if ((minute>49) && (minute<55)) { 
    MTEN; 
    MINUTES; 
    Serial.print("Ten Minutes ");
  } 
  if (minute>54) { 
    MFIVE; 
    MINUTES; 
    Serial.print("Five Minutes ");
  }



  if ((minute <5))
  {
    switch (hour) {
    case 1:
    case 13: 
      ONE; 
      Serial.print("One ");
      break;
    case 2:
    case 14: 
      TWO; 
      Serial.print("Two ");
      break;
    case 3: 
    case 15:
      THREE; 
      Serial.print("Three ");
      break;
    case 4: 
    case 16:
      FOUR; 
      Serial.print("Four ");
      break;
    case 5: 
    case 17:
      HFIVE; 
      Serial.print("Five ");
      break;
    case 6: 
    case 18:
      SIX; 
      Serial.print("Six ");
      break;
    case 7: 
    case 19:
      SEVEN; 
      Serial.print("Seven ");
      break;
    case 8: 
    case 20:
      EIGHT; 
      Serial.print("Eight ");
      break;
    case 9: 
    case 21:
      NINE; 
      Serial.print("Nine ");
      break;
    case 10:
    case 22: 
      HTEN; 
      Serial.print("Ten ");
      break;
    case 11:
    case 23: 
      ELEVEN; 
      Serial.print("Eleven ");
      break;
    case 0:
    case 12: 
      TWELVE; 
      Serial.print("Twelve ");
      break;
    }
    OCLOCK;
    Serial.println("O'Clock");
  }
  else
    if ((minute < 35) && (minute >4))
    {
      PAST;
      Serial.print("Past ");
      switch (hour) {
      case 1:
      case 13: 
        ONE; 
        Serial.println("One ");
        break;
      case 2: 
      case 14:
        TWO; 
        Serial.println("Two ");
        break;
      case 3: 
      case 15:
        THREE; 
        Serial.println("Three ");
        break;
      case 4: 
      case 16:
        FOUR; 
        Serial.println("Four ");
        break;
      case 5: 
      case 17:
        HFIVE; 
        Serial.println("Five ");
        break;
      case 6: 
      case 18:
        SIX; 
        Serial.println("Six ");
        break;
      case 7: 
      case 19:
        SEVEN; 
        Serial.println("Seven ");
        break;
      case 8: 
      case 20:
        EIGHT; 
        Serial.println("Eight ");
        break;
      case 9: 
      case 21:
        NINE; 
        Serial.println("Nine ");
        break;
      case 10:
      case 22: 
        HTEN; 
        Serial.println("Ten ");
        break;
      case 11:
      case 23: 
        ELEVEN; 
        Serial.println("Eleven ");
        break;
      case 0:
      case 12: 
        TWELVE; 
        Serial.println("Twelve ");
        break;
      }
    }
    else
    {
      // if we are greater than 34 minutes past the hour then display
      // the next hour, as we will be displaying a 'to' sign
      TO;
      Serial.print("To ");
      switch (hour) {
      case 1: 
      case 13:
        TWO; 
        Serial.println("Two ");
        break;
      case 14:
      case 2: 
        THREE; 
        Serial.println("Three ");
        break;
      case 15:
      case 3: 
        FOUR; 
        Serial.println("Four ");
        break;
      case 4: 
      case 16:
        HFIVE; 
        Serial.println("Five ");
        break;
      case 5: 
      case 17:
        SIX; 
        Serial.println("Six ");
        break;
      case 6: 
      case 18:
        SEVEN; 
        Serial.println("Seven ");
        break;
      case 7: 
      case 19:
        EIGHT; 
        Serial.println("Eight ");
        break;
      case 8: 
      case 20:
        NINE; 
        Serial.println("Nine ");
        break;
      case 9: 
      case 21:
        HTEN; 
        Serial.println("Ten ");
        break;
      case 10: 
      case 22:
        ELEVEN; 
        Serial.println("Eleven ");
        break;
      case 11: 
      case 23:
        TWELVE; 
        Serial.println("Twelve ");
        break;
      case 0:
      case 12: 
        ONE; 
        Serial.println("One ");
        break;
      }
    }



  // now we can illuminate the extra minute LEDs

  if ((minute-(minute/5)*5)==1) { 
    LED1; 
  }
  if ((minute-(minute/5)*5)==2) { 
    LED1; 
    LED2; 
  }
  if ((minute-(minute/5)*5)==3) { 
    LED1; 
    LED2; 
    LED3; 
  }
  if ((minute-(minute/5)*5)==4) { 
    LED1; 
    LED2; 
    LED3; 
    LED4; 
  }


}


void incrementtime(void){
  // increment the time counters keeping care to rollover as required
  second=0;
  if (++minute >= 60) {
    minute=0;
    if (++hour == 25) {
      hour=1;  
    }
  }
  // debug outputs
  Serial.println();
  if (DS1302Present==1) print_DS1302time(); 
  else Serial.print("Arduino Time: " );
  Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.println(second);

}


void SWversion(void) {
  delay(2000);
  Serial.println();
  Serial.println("Wordclock -Arduino v3.0a - reduced brightnes version");
  Serial.println("(c)2009,2010 Doug Jackson");
}


void loop(void)
{
int previous_minute;

  if (DS1302Present==1) 
    previous_minute = minute;  // keep track of the last minute read to spot faulty DS1302 chips
  
  //Serial.println("Loop Started");
  
  // heart of the timer - keep looking at the millisecond timer on the Arduino
  // and increment the seconds counter every 1000 ms
  if ( millis() - msTick >999) {
    msTick=millis();
    second++;
    // Flash the onboard Pin13 Led so we know something is hapening!
    digitalWrite(13,HIGH);
    delay(50);
    digitalWrite(13,LOW);  
    delay(50);
    digitalWrite(13,HIGH);
    delay(50);
    digitalWrite(13,LOW);  

    if (second%5==0) {
      Serial.print(second);
      Serial.print("..");
    }
  }



  //test to see if we need to increment the time counters
  if (second==60) 
  {
    incrementtime();
    displaytime();
  }

  if (DS1302Present==1) {
    // Get the current time and date from the chip 
    Time t = rtc.time();
    second=t.sec;     
    minute=t.min;
    hour=t.hr; 
    
    //occasionally, we can get a DS1302 that fails to increment the minute counter
    // spot the failure, then flash ONE forever as an error code.
    //if ((previous_minute > minute) & previous_minute!=59) 
    //  {
    //    Serial.println("rtc error - replace DS1302 please ");
    //    while (1) {
    //       ledsoff();
    //       delay(500);
    //       ONE;
    //       delay(500);
    //    }
    //  }
  }

  // set the brightnes level based on the current hour - night=7pm - 6.59am
  //
  if ((hour <7) | (hour >=19)) 
    current_brightnes=NIGHTBRIGHTNESS;
  else 
    current_brightnes=DAYBRIGHTNESS;
  

  // test to see if both buttons are being held down
  // if so  - start a self test till both buttons are held
  // down again.
  if ( digitalRead(FWDButtonPin)==BTNActive && digitalRead(REVButtonPin)==BTNActive)
  {
    selftestmode = !selftestmode;
    if (selftestmode) Serial.println("Selftest Mode TRUE");
    else Serial.println("Selftest mode FALSE");
  }

  //    if (selftestmode) { 
  //      for(int i=0; i<100; i++)
  //      {
  //      Display1=255; Display2=255; Display3=255;
  //      WriteLEDs(); delay(101-i);
  //      ledsoff(); WriteLEDs();delay(101-i);
  //      if (digitalRead(FWDButtonPin)==1) selftestmode=!selftestmode;
  //     
  //      }
  //      displaytime();
  //      
  //    }


  // test to see if a forward button is being held down
  // for time setting
  if (digitalRead(FWDButtonPin) ==BTNActive ) 
    // the forward button is down
    // and it has been more than one second since we
    // last looked

  {
    Serial.println("Forward Button Down");
    //minute=(((minute/5)*5) +5); 
    incrementtime();
    second++;  // Increment the second counter to ensure that the name
    // flash doesnt happen when setting time
    if (DS1302Present==1) {
      // Make a new time object to set the date and time 
      Time t(2010, 04, 28, hour, minute, second, 01);
      // Set the time and date on the chip 
      rtc.time(t);
    } 
    delay(100);
    displaytime();
  }

  // test to see if the back button is being held down
  // for time setting
  if (digitalRead(REVButtonPin)==BTNActive ) 
  {

    Serial.println("Backwards Button Down");
    minute--; 
    minute--;
    second=0; // decrement the minute counter
    if (minute<0) { 
      minute=58; 
      if (--hour <0) hour=23;
    } 
    incrementtime();
    second++;  // Increment the second counter to ensure that the name
    // flash doesnt happen when setting time  


    if (DS1302Present==1) {
      // Make a new time object to set the date and time 
      Time t(2010, 04, 28, hour, minute, second, 01);
      // Set the time and date on the chip 
      rtc.time(t);
    }

    displaytime();
    delay(100);
  }

}		  




