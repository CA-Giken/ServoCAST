#ifndef _Logger_h
#define _Logger_h

#include  "Arduino.h"
#define   POLYNOMINAL 6
#include  "napprox.h"

namespace logger{
#ifdef TARGET_NRF52840
  struct ALOG{
    uint32_t stamp;
    uint8_t mode;
    uint8_t duty;
    uint8_t cmd;
    uint16_t interval;
    uint16_t latency;
    uint16_t omega;
    int16_t beta;
    int16_t sigma;
    int16_t eval;
  };
#endif
#ifdef _RENESAS_RA_
  struct ALOG{
    uint32_t stamp;
    uint16_t interval;
    int16_t duty;
    int16_t beta;
    int16_t eval;
  };
#endif
  extern ALOG stage;
  extern ALOG *data;
  extern int size;
  void start();
  void latch();
  void dump();
  int length();
  int limit();
  ALOG *trace(int backtrace);
  inline int variation(int *dat,int samp){
    float sig=0;
    for(int i=0;i<samp;i++){
      int d=dat[i];
      sig+=d*d;
    }
    return sqrt(sig/samp);
  }
  inline int N(int nsamp,int dim){
    logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
    logger::ALOG *p0=logger::data;   //head of data
    if(p1<p0+nsamp) return -1;
    logger::ALOG *pv=p1-nsamp;

    int cval[200];
    for(int i=0;i<nsamp;i++){
      cval[i]=pv[i].beta;
    }
    double coef[POLYNOMINAL];
    int nofs=approx(dim,cval,nsamp,coef);
    auto neq=[&](double x){
      double y=0;
      x=(x-nofs)/nofs;
      for(int i=dim-1;i>=0;i--) y=y*x+coef[i];
      return y;
    };
    for(int i=0;i<nsamp;i++){
      int cc=neq(i);
      int cv=cval[i];
      cval[i]=(cv-cc);
#ifdef ONE_SPAN_TEST
      printf("%lu %d %d %d %d %d %d\n",pv[i].stamp,pv[i].beta,pv[i].sigma,pv[i].duty,cv,cc,cval[i]);
#endif
    }
    float sig=variation(cval,nsamp);
    return sig;
  }
  inline int analyze(int samp,int dim){
    if(samp<1000)
      return N(samp,dim);

    logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
    logger::ALOG *p0=logger::data;   //head of data
    int ts0=p1->stamp;
    int vsamp=0;
    for(;;p1--){
      if(p1<=p0) return -1;
      if(ts0-p1->stamp>samp) break;
      vsamp++;
    }
    if(vsamp>1) return N(vsamp,dim);
    else return -1;
  }
}

#endif
