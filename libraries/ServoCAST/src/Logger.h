#ifndef _Logger_h
#define _Logger_h

#ifdef ARDUINO
#include  "Arduino.h"
#else
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
typedef short int16_t;
#endif

#define   POLYNOMINAL 6
#include  "../../CA_Math/napprox.h"

namespace logger{
#ifdef TARGET_NRF52840
  struct ALOG{
    uint32_t stamp;
    uint8_t duty;
    uint8_t eval;
    int16_t beta;
    uint8_t mode,cmd;
    uint16_t interval;
    uint16_t latency;
    uint16_t omega;
    void print();
  };
#endif
#ifdef _RENESAS_RA_
  struct ALOG{
    uint32_t stamp;
    uint8_t duty;
    uint8_t eval;
    uint8_t cmd;
    int16_t beta;
    void print();
  };
#endif
  extern ALOG stage;
  extern int16_t length;
  void start();
  void latch();
  int limit();
  ALOG *trace(int index);
  float N(int nsamp,int dim,int wgh=100);
  float analyze(int samp,int dim,int wgh=100);
  void sweep();
}

#endif
