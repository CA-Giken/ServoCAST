#ifndef _Dcore_h
#define _Dcore_h

#include  "Arduino.h"

namespace dcore{
  typedef void (*StartCB_t)();
  typedef uint16_t (*ContCB_t)(int32_t interval_usec,int32_t turnon_usec);
  typedef void (*EndCB_t)();
  extern uint8_t RunLevel;
  void config(int sensPort,int gatePort,int ndiv,int debounce_usec,int pwmSwDelay=0,int pwmNote=1);
  void run(StartCB_t,ContCB_t,EndCB_t);
  void shift();  //switch to deceleration mode
  void sleep(uint16_t ms);
}

#endif
