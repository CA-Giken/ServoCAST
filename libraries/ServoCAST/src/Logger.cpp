#include "Arduino.h"
#include "Logger.h"
#include <SetTimeout.h>

#define WSAMP 150   //sampling window size

namespace logger{
#ifdef TARGET_NRF52840
  static struct ALOG buf[1000];
  void ALOG::print(){
    Serial.print("$$ ");
    Serial.print(stamp); Serial.print(" ");
    Serial.print(duty); Serial.print(" ");
    Serial.print(beta); Serial.print(" ");
    Serial.print(eval); Serial.print(" ");
    Serial.print(interval); Serial.print(" ");
    Serial.print(mode); Serial.print(" ");
    Serial.print(latency); Serial.print(" ");
    Serial.print(omega); Serial.print(" ");
    Serial.println(cmd);
  }
#endif
#ifdef _RENESAS_RA_
#define RINGBUF
  static struct ALOG buf[200];
  void ALOG::print(){
    Serial.print("$$ ");
    Serial.print(stamp); Serial.print(" ");
    Serial.print(duty); Serial.print(" ");
    Serial.print(beta); Serial.print(" ");
    Serial.println(eval);
  }
#endif
  static struct ALOG *data=buf;
  struct ALOG stage;
  static int32_t tzero,tdmp;
  static int dcount=0;
  void clear(){
    memset(&stage,0,sizeof(stage));
    stage.stamp=0xFFFFFFFF;
  }
  int limit(){
    return sizeof(buf)/sizeof(buf[0]);
  }
  void start(){
    clear();
    dcount=0;
    tzero=micros();
    tdmp=0;
  }
  void latch(){
    if(stage.stamp==0xFFFFFFFF) stage.stamp=micros()-tzero;
    if(dcount<limit()) buf[dcount++]=stage;
    stage.stamp=0xFFFFFFFF;
  }
  ALOG *trace(int index){
    if(index<0){
      int n=dcount+index;
      return n<0? NULL:buf+n;
    }
    else return index>=dcount? NULL:buf+index;
  }
  inline int variation(int *dat,int samp){
    float sig=0;
    for(int i=0;i<samp;i++){
      int d=dat[i];
      sig+=d*d;
    }
    return sqrt(sig/samp);
  }
  int N(int nsamp,int dim){
    if(logger::trace(-nsamp)==NULL) return -1;  //buffer not enough
    int cval[WSAMP];
    for(int i=-nsamp,j=0;i<0;i++,j++){
      cval[j]=logger::trace(i)->beta;
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
  int analyze(int samp,int dim){
    if(samp<1000)   //"samp" defined as data count, else as interval in micro second
      return N(samp,dim);

    int ts0=logger::trace(-1)->stamp;
    int vsamp=0;
    for(int n=-1;;n--){
      logger::ALOG *p=logger::trace(n);
      if(p==NULL) return -1;
      if(ts0-p->stamp > samp) break;
      vsamp++;
    }
    if(vsamp>1) return N(vsamp,dim);
    else return -1;
  }
  void sweep(){
    if(!Serial) return;
    ALOG *p=trace(-1);
    if(p==NULL) return;
    else if(p->stamp<=tdmp) return;
    int ntr=-1;
    for(;;ntr--){
      p=trace(ntr-1);
      if(p==NULL || p->stamp<=tdmp) break;
    }
    for(;ntr<0;ntr++){
      p=trace(ntr);
      p->print();
      tdmp=p->stamp;
    }
  }
}
