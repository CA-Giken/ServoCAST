#ifndef _Logger_h
#define _Logger_h

#include  "Arduino.h"
#define   POLYNOMINAL 6
#include  "napprox.h"

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
  int variation(int *dat,int samp);
  int N(int nsamp,int dim);
  int analyze(int samp,int dim);
  void sweep();
}

#endif
