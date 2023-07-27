/* 
 * File:   Encoder.c
 * Author: Gary Stofer
 *
 * Created on Jan 12, 2017, 9:35 AM
 * 
 * Module to handle the rotary encoder knob 
 */

#include "Encoder.h" 

char EncoderCnt = 1;  // Default startup display -- can be set to any valid Display Enum value
char EncoderPressedCnt = 0; // This changes after the user held the knob  pressed for long press timeout and then turns while still pressing.
char EncoderDirection = -1; // So that it decremets to a valid display in case the default display is not currently valid due to sensor lacking
unsigned char ShortPressCnt = 0;
unsigned char LongPressCnt = 0;


/*
 * The ISR to handle the rotary encoder interrupt
 * 
  This ISR handles the reading of a quadrature encoder knob and increment or decrements  two global encoder variables
  depending on the direction of the encoder and the state of the button. The encoder functions at 1/4 the maximal possible resolution
  and generally provides one inc/dec per mechanical detent.

  One of the quadrature inputs is used to trigger this interrupt handler, which checks the other quadrature input to decide the direction.
  It is assumed that the direction  quadrature signal is not bouncing while the first phase is causing the initial interrupt as it's signal
  is 90deg opposed.  RC filtering of the contacts is required.

  Three encoder count variables are being modified by this ISR
  1) EncoderCnt increments or decrements when the knob is turned without the button being press
  2) EncoderPressCnt increments or decrements when the knob is  turned while the button is also pressed.
     This only happens after a LONG press timeout
  3) EncoderDirection either +1 or -1 depending on the direction the user turned the knob last

  Encoder sequence and detent position:
  A.) It is preferable to use an encoder with a centered detent position on one phase, while the other phase is 90 degree shifted, such as in the Panasonic 
      EVEJBE line of encoders. This provides the biggest marging for contact bouncing and resulting false detcetion of rotation. The following diagram shows the
      relationship between phase A and B with the detent. 

    Detent          |         |       
    Phase A     __-----_____-----__    denetnt at 90deg of phase A -- Use Phase A edge to triggert count
    Phase B     ____-----_____-----    detent at 0 deg of phase B
    
     Best margin for bouncing contact is achieved when triggering on the opening edge of Phase A since contact bounce is oftewn less pronounced on opening.
      
  B.) Often however, encoders employ a shifted detent where the detent is at 135 deg on Phase A and 45 deg on B, such as in the Bourns PEC series. 
      This has the effect that the encoder seems to bounce back or forward one position when let go or jostled a little.

    Detent        |         |
    PhaseA    _-----_____-----__    Detent at 135 degree of phase A
    PhaseB    ___-----_____-----    Detent ar 45 degree of phase B

      
*/

// This ISR gets triggered when the knob gets turned -- It should to be attached to the RISING edge of the interrupt 
// Use "attachInterrupt(0, ISR_KnobTurn, RISING )" to attach this to Int 0 pin
void ISR_KnobTurn( void)
{
  if ( digitalRead( Enc_B_PIN ) )
    EncoderDirection = -Enc_DIRECTION;
  else
    EncoderDirection = Enc_DIRECTION;

  if ( digitalRead( Enc_PRESS_PIN ) )  // only incremnt when button is not pressed. 
    EncoderCnt += EncoderDirection;
  else
    EncoderPressedCnt += EncoderDirection; // This happens after Longpress is registered (has timed out) and user starts to turn 

}


/*
 The ISR to handle the button press interrupt.

  Two modes of button presses are recognized. A short, momentary press and a long, timing-out press.
  While the hardware debounced button signal is sampled for up to TIMEOUT time in this ISR no other code is being executed. If the time-out occurs
  a long button press-, otherwise a short button press is registered. Timing has to be done by a software counter since interrupts are disabled
  and function millis() and micros() don't work during this time.
  Software timing is CPU clock dependent and therefore has to be adjusted to the clock frequency.
*/

// This ISR gets triggered when the button is presses. 
// Use "attachInterrupt(1, ISR_ButtonPress, FALLING)" to connect this to Int 1 pin
void ISR_ButtonPress(void)
{
  volatile unsigned long t = 0;
  while ( !digitalRead( Enc_PRESS_PIN ))
  {
    if (t++ > LONGPRESS_TIMEOUT)    // timing done with code loop -- Caution clock dependent 
    {
      LongPressCnt++;
      return;
    }
  }
  ShortPressCnt++;
}

/* Setup the encoder pins and interrupts 
 *  NOTE: Encoder Phase A needs to be connected to INT0 and the push button to INT1
 *        THIS is HARDCODED below in the attachInterrupt calls 
 */
void EncoderInit( unsigned  char PinA, unsigned char PinB, unsigned char PinSwitch )
{
  // Setup the Encoder pins to be inputs with pullups
  pinMode(PinA, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(PinB, INPUT);    // Use external 10K pullup and 100nf to gnd for debounce
  pinMode(PinSwitch, INPUT);// Use external 10K pullup and 100nf to gnd for debounce

  // NOTE the hardcoded relationship between pins and INT0 / INT1
  attachInterrupt(0, ISR_KnobTurn, RISING );    // for the rotary encoder knob rotating -- on the raising edge for less bounce from the contacts
  attachInterrupt(1, ISR_ButtonPress, FALLING);    // for the rotary encoder knob push
}



