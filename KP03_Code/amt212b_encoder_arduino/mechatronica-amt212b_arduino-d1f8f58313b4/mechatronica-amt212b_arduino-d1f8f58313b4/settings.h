/* settings
 * 
 * Due to the Arduino IDE's behaviour to first run all #include files during compiling, any #defines placed before an include are ignored
 * To give an nice an structured overview of the DEBUG options a seperate settings.h is included and run first during compilation to prefent this
 * 
 * 
 * ISR Timer interrupt example on: https://electronoobs.com/eng_arduino_tut140.php
 * 
 * 100ms using timer1 and prescalar of 256.
 * Calculations example(for 100ms):  
 *  - System clock 16 Mhz and Prescalar 256;
 *  - Timer 1 speed = 16Mhz/256 = 62.5 Khz 
 *  - Pulse time = 1/62.5 Khz =  16us  
 *  - Count up to = 100ms / 16us = 6250 (so this is the value the OCR register should have)
 *  
 *  - f
 *  
 */
//#define RS485_DEBUG                    // Prints information received over the RS485 BUS
#define AMT_DEBUG                        // Prints AMT212B debug info on USB serial port
//#define USB_DEBUG                      // Print information recieved on the USB serial port

#define ISR_PIN = 2;
 
volatile bool ISR_PIN_FLAG = false;
volatile bool ISR_TIMER_FLAG = false;

// Pin interrupt service routine
void ISRINT1()
{
  ISR_PIN_FLAG = true;
}

// Timer interrupt service routine
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                             //First, set the timer back to 0 so it resets for next interrupt
  ISR_TIMER_FLAG = true;
}

// Configure interrupts
void configISR(void){
   //Timer 1 interrupt
  cli();                                  //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR1A = 0;                             // Reset entire TCCR1A to 0 
  TCCR1B = 0;                             // Reset entire TCCR1B to 0
 
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR1B |= B00000100;                    //Set CS12 to 1 so we get prescalar 256  
  
  /*3. Set the value of register A to 31250*/
  OCR1A = 6250;                          //Finally we set compare register A to this value  
  sei();                                  //Enable back the interrupts

  //External pin interrupt
  //pinMode(ISR_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(ISR_PIN), ISRINT1, FALLING);
}

void enableInterruptTimer(void){
  cli();                                  //stop interrupts for till we make the settings
  bitWrite(TIMSK1,1,1);                   //Enable compare match A (BIT OCIE1A)                 
  sei();                                  //Enable back the interrupts
}

void disableInterruptTimer(void){
  cli();                                  //stop interrupts for till we make the settings
  bitWrite(TIMSK1,1,0);                   //Disabeling compare match A (BIT OCIE1A)
  sei();                                  //Enable back the interrupts
}
