/* 
 * File:   Encoder.h
 * Author: Gary Stofer
 *
 * Created on Jan 12, 2017, 9:35 AM
 * 
 * Module to handle the rotary encoder knob 
 */

#include "Arduino.h"
#include "build_opts.h"


#define Enc_DIRECTION (-1)  // polarity of encoder , either -1 or +1 depending on the phase relation of the two signals in regard of the turn direction

#define LONGPRESS_TIMEOUT 50000 // Note: not in milli seconds since this is used in a SW while loop timer
#ifdef  __cplusplus
extern "C" {
#endif
extern char EncoderCnt ;
extern char EncoderPressedCnt ;
extern char EncoderDirection ;
extern unsigned char ShortPressCnt;
extern unsigned char LongPressCnt ;
extern void EncoderInit( unsigned  char PinA, unsigned char PinB, unsigned char PinSwitch );
 
#ifdef  __cplusplus
}
#endif
