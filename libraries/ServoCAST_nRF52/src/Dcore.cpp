#include "Arduino.h"
#include <mbed.h>
#include <rtos.h>
#include <SetTimeout.h>
#include "Dcore.h"
#include "Logger.h"
#include "NRF52_MBED_TimerInterrupt.h"

static NRF52_MBED_Timer ITimer0(NRF_TIMER_3);   //PWM
static NRF52_MBED_Timer ITimer1(NRF_TIMER_4);   //Debouncer and PWM blocker

#define T_PWM_MIN 70 //usec
#define T_WAIT 200 //usec
#define T_BOUNCE 200  //usec
#define T_INTR_MIN 1000 //usec

#define LENGTH(x) (sizeof(x))/(sizeof(x[0]))

namespace dcore{//DC core
  uint8_t RunLevel;
  uint32_t tusec;
  uint16_t tmsec;
  static StartCB_t start_callback;
  static ContCB_t cont_callback;
  static EndCB_t end_callback;
  static uint16_t swdelay;
  static void init();
}

namespace pwm{//methods for gate on(turn pwm pulse high)
  static uint8_t ndiv=1;
  static uint8_t DO;//digital out pin
  static uint16_t Count; //Switch count
  static uint16_t Interval; //Switch interval
  static uint16_t Treq; //Turn on time cmd
  static uint16_t Tact; //actual turn on time sum
  static uint16_t Ton; //latch for Tact
  static uint16_t Tirqe; //IRQ enable delay after intr_off
//	698Hz/F5 784Hz/G5 880Hz/A5 988/B5 1046Hz/C6	1175Hz/D6
//  const uint16_t Tpwm0=1000000/880;  //primary period
//  const uint16_t TpwmS=1000000/1046; //shorter period
//  const uint16_t TpwmL=1000000/784;  //longer priod
  const uint16_t Tpwm[]={1000000/860,1000000/906,1000000/967,1000000/1019,1000000/1088,1000000/1146,1000000/1208,1000000/1290};
  const auto Npwm=sizeof(Tpwm)/sizeof(Tpwm[0]);
  static uint8_t Ipwm;
  static uint8_t Duty; //duty command
  static bool Block;
  void on(); //to turn gate on
  void off(); //to turn gate off
  static void intr_on(); //interrupt callback to turn pulse high
  static void intr_off(); //interrupt callback to turn pulse low
  void stop(); //stop pwm sequence
  void start();
  void start1(uint32_t t_rev);
  void startN(uint32_t t_rev);
}

namespace sens{
  static rtos::Thread thread(osPriorityHigh);  //rotation sensor
  static rtos::Semaphore sema;
  static long wdt;
  static volatile int32_t Tm,Tb;
  static volatile uint32_t Interval,Interval_1;
  static uint8_t ndiv,cdiv,Nint;
  static mbed::InterruptIn* irq;
  static uint8_t DI;
  static uint16_t debounce=0;
//methods for sensing(pin interrupt)
  static void intr();
  static void start();
  static void stop();
  static void task();
}

namespace debouncer{
  static uint16_t Tcmd;  //Duty command
  static uint16_t Ton; //Turn-on time
  static volatile uint32_t Interval;
}

namespace pwm{//methods for pwm
  void init(){
    off();
    Ton=Tact=Treq=Count=Interval=Duty=0;
    Block=true;
  }
  void on(){ digitalWrite(DO,HIGH);}
  void off(){ digitalWrite(DO,LOW);}
  static void intr_off(){
    off();
    ITimer1.setFrequency(1,[](){});
    Tact+=Treq;
    if(Count==0){
      Block=true;
      Ton=Tact;
      Tact=0;
      sens::start();
    }
  }
  static void intr_off2(){
    off();
    Block=true;
    Ton=Tact+=Treq;
    Tact=0;
    if(Tirqe>0) ITimer1.setInterval(Tirqe,[](){
      ITimer1.setFrequency(1,[](){});
      sens::start();
    });
    else{
      ITimer1.setFrequency(1,[](){});
      sens::start();
    }
  }
  static void intr_on(){
    switch(Count){
      case 0: return;
      case 101: ITimer0.setInterval(Interval,intr_on);
    }
    Count--;
    if(!Block){
      int32_t tnow=micros();
      int32_t tnex=sens::Tm+sens::Interval-T_WAIT;
      int32_t tnex2=sens::Tm+sens::Interval*13/14;
      if(tnex>tnex2) tnex=tnex2;
      Treq=(long)Interval*Duty>>8;
      tnex=tnex-tnow;
      if(Treq>tnex){
        Treq=MAX(0,tnex);
      }
      Tirqe=0;
      if(Interval>tnex){
        if(Treq>T_PWM_MIN){
          on();
          ITimer1.setInterval(Treq,intr_off2);
          tnex-=Treq;
          if(tnex>T_WAIT) Tirqe=tnex;
        }
        else{
          Block=true;
          Ton=Tact;
          Tact=0;
          if(tnex>T_WAIT){
            ITimer1.setInterval(tnex,[](){
              ITimer1.setFrequency(1,[](){});
              sens::start();
            });
          }
          else sens::start();
        }
      }
      else{
        if(Treq<T_PWM_MIN){
          Treq=T_PWM_MIN;
        }
        on();
        ITimer1.setInterval(Treq,intr_off);
      }
    }
  }
  void stop(){ Count=0;}
  void start(){
    ITimer0.setInterval(Interval,intr_on);
    if(Duty>0){
      on();
      Treq=(long)Interval*Duty>>8;
      if(Treq<T_PWM_MIN) Treq=T_PWM_MIN;
      ITimer1.setInterval(Treq,intr_off);
      Tirqe=0;
    }
    else Treq=0;
  }
  void start1(uint32_t trev){
//    uint16_t pcnt=trev/Tpwm[0];
//    pcnt|=1; //division should be odd number
    uint16_t pcnt= ndiv==1? 3: ndiv==2? 4:3;
    Interval=trev/pcnt;
    start();
    Count=pcnt-1;
    Block=false;
  }
  void startN(uint32_t trev){
/*    int tf=trev*130/400;
    uint16_t *tbl=Tpwm;
    if(tf>Tpwm[sizeof(Tpwm)/sizeof(Tpwm[0])-1]){
      if(tf/2>Tpwm[0]) tbl++;
    }
    else{
      while(tf>*tbl) tbl++;
    }*/
    int tw=Tpwm[Ipwm];
    int tf=trev/(ndiv==1? 4: ndiv==2? 6:2);
    int dw=tw-tf;
    if(dw>0){
      if(dw<200) tw=Tpwm[Ipwm-1];
    }
    else{
      if(dw>-200) tw=Tpwm[Ipwm+1];
    }
    
    if(Count==0){  //Start pwm
      Interval=tw;
      start();
      Count=100;
    }
    else if(Interval!=tw){  //Change frequency
      Interval=tw;
      Count=101;
    }
    else Count=99;  //Extend duration
    Block=false;
  }
  void sweep(uint32_t duration){
    Interval=Tpwm[LENGTH(Tpwm)-1];
    Count=duration/Interval;
    ITimer0.setInterval(Interval,[](){
      if(Count==0) ITimer0.setInterval(1,[](){});
      else on();
    });
    ITimer1.setInterval(Interval*Duty/256,[](){
      off();
      ITimer1.setInterval(Interval,[](){
        off();
        Count--;
        if(Count==0) ITimer1.setInterval(1,[](){});
      });
    });
  }
}

namespace debouncer{
  void init(){ Interval=Tcmd=Ton=0;}
  void intr_ready(){
    sens::start();
    ITimer0.setFrequency(1,[](){});
  }
  void intr_off(){
    pwm::off();
    ITimer1.setFrequency(1,[](){});
  }
  void start(uint16_t dtn,uint16_t dto){
    if(dto==0){  //wait half turn
      ITimer0.setInterval(T_BOUNCE,intr_ready);
      ITimer1.setFrequency(1,[](){});
      Ton=0;
    }
    else{
      int dt_1=(int)dto/30;
      int dt0=(int)dtn/30;
      int a=-dt0+dt_1;
      int dtx=(int)dtn;
      if(a>0){
        int v0=(-(dt0*dt0)+(dt_1*dt_1)+2*(dt_1*dt0));
        int d0=dt0*dt_1*(dt0+dt_1);
//      dtx=-(-v0+v0*std::sqrt(1+2.0*d0*a/v0/v0))/a *10;  //theorial
        int x=d0/v0;
        dtx=(x-x*x*a/v0/4) *30;  //approx
      }
      uint16_t t1= Interval==0? dtx*4/5:dtx*6/7;
      Interval=t1;
      ITimer0.setInterval(t1,intr_ready);
      t1-=50;
      if(t1>T_INTR_MIN && Tcmd>T_PWM_MIN){
        Ton= Tcmd<t1? Tcmd:t1;
        pwm::on();
        ITimer1.setInterval(Ton,intr_off);
      }
      else{
        ITimer1.setFrequency(1,[](){});
        Ton=0;
      }
    }
  }
}

//utility
static int16_t log_lat;
static void log_pre();
static void log_post();

namespace sens{
  void init(){ wdt=Interval=Interval_1=cdiv=0;}
  void stop(){
    irq->rise(NULL);
  }
  void start(){
    irq->rise(mbed::callback((voidFuncPtrParam)intr, (void *)NULL));
  }
  void intr(){
    stop();
    long tnow=micros();
    if(tnow-Tb<debounce){
      start();
      Tb=tnow;
      return;
    }
    long dt=tnow-Tm;
    Tm=Tb=tnow;
    switch(dcore::RunLevel){
    case 0:{
      debouncer::start(dt,0);
      Interval_1=dt;
      int tc=dt*(ndiv+1);
      if(tc<20000){
        dcore::RunLevel=1;
        sema.release();
      }
      break;
    }
    case 1:
      debouncer::start(dt,0);
      switch(cdiv){
      case 0:
        if(ndiv){
          Interval_1+=dt;
          cdiv= ndiv==1? 1:3;
        }
        else{
          Interval_1=dt;
          dcore::RunLevel=2;
        }
        break;
      case 3:
        Interval_1+=dt;        
        cdiv--;
        break;
      case 2:
        Interval+=dt;
        cdiv--;
        break;
      case 1:
        Interval+=dt;
        dcore::RunLevel=2;
        break;
      }
      break;
    case 2:
      debouncer::start(Interval+=dt,Interval_1);
      dcore::RunLevel=3;
      sema.release();
      break;
    case 3:
      pwm::Ton=debouncer::Ton;      
      debouncer::start(dt,Interval);
      Interval=dt;
      sema.release();
      break;
    case 4:
      dcore::RunLevel=5;
    case 5:
      Interval=dt;
      pwm::start1(dt);
      sema.release();
      break;
    case 6:
      Interval=dt;
      pwm::startN(dt);
      sema.release();
      break;
    }
  }
  void task(){
  WAIT:
    sema.acquire();
    switch(dcore::RunLevel){
    case 1:
      Serial.print("RunLevel 1 / ");
      Serial.println(Interval_1);
      wdt=setTimeout.set([](){
        Serial.println("sens wdt:RunLevel 1=>0");
        dcore::init();
      },20);
      (*dcore::start_callback)();
      logger::start();
      break;
    case 3:
      dcore::tmsec=(dcore::tusec+=Interval)/1000;
      log_pre();
      debouncer::Tcmd=(*dcore::cont_callback)(Interval,pwm::Ton);
      log_post();
/*      if(logger::length()==0){
        Serial.print("RunLevel 3 / ");
        Serial.print(Interval_1);
        Serial.print(" / ");
        Serial.print(Interval);
        Serial.print(" / ");
        Serial.println(debouncer::Interval);
      }
      if(dcore::RunLevel>3){
        Serial.print("Runlevel 3=>");
        Serial.println(dcore::RunLevel);
      }*/
      logger::latch();
      setTimeout.clear(wdt);
      wdt=setTimeout.set([](){
        Serial.println("sens wdt:RunLevel 3=>0");
        (*dcore::end_callback)();
        dcore::init();
      },20);
      break;
    case 5:
    case 6:
      dcore::tmsec=(dcore::tusec+=Interval)/1000;
      log_pre();
      pwm::Duty=(*dcore::cont_callback)(Interval,pwm::Ton);
      log_post();
      logger::latch();
      if(wdt!=0) setTimeout.clear(wdt);
      wdt=setTimeout.set([](){
        pwm::Duty=170;
        stop();
//        pwm::stop();
        pwm::sweep(1000000L);  //usec
        setTimeout.set(dcore::init,1000);  //msec
        Serial.println("sens wdt:RunLevel 6=>0");
        (*dcore::end_callback)();
      },20);
      break;
    }
    goto WAIT;
  }
}

namespace dcore{
  void init(){
    pwm::init();
    sens::init();
    debouncer::init();
    sens::start();
    RunLevel=0;
    tusec=0;
    ITimer0.setFrequency(1,[](){});
    ITimer1.setFrequency(1,[](){});
  }
  void shift(){
    if(RunLevel==3){
      RunLevel=4;
      if(swdelay>0){
        setTimeout.set([](){
          RunLevel=6;
        },swdelay);
      }
    }
  }
  void config(int DI,int DO,int div,int boun,int swdelay,int note){
    sens::DI=DI;
    pwm::DO=DO;
    pwm::ndiv=sens::ndiv=div;
    sens::debounce=boun;
    dcore::swdelay=swdelay;
    pwm::Ipwm= note<1? 1: note>pwm::Npwm-2? pwm::Npwm-2:note;
  }
  void run(StartCB_t cb_start,ContCB_t cb_cont,EndCB_t cb_end){
    start_callback=cb_start;
    cont_callback=cb_cont;
    end_callback=cb_end;
    sens::Tm=(sens::Tb=micros())-1000000;
    sens::Nint=digitalPinToInterrupt(sens::DI);
    sens::irq = new mbed::InterruptIn(digitalPinToPinName(sens::Nint));
    sens::thread.start(mbed::callback(sens::task));
    pwm::off();
    init();
    attachInterrupt(sens::Nint, sens::intr, RISING);
    pinMode(sens::DI,INPUT); //disable pullup after attachInterrupt
    
  }
  void sleep(uint16_t ms){
    long t=millis();
    do{
      ::sleep();
      // __WFI();
    }while((long)millis()-t<ms);
  }
}

void log_pre(){
  log_lat=micros();
  logger::stage.stamp=dcore::tusec;
  logger::stage.latency=log_lat-sens::Tm;
  logger::stage.interval=sens::Interval;
  logger::stage.mode=dcore::RunLevel;
  logger::stage.duty=((uint32_t)pwm::Ton<<8)/sens::Interval;
}
void log_post(){
  logger::stage.cmd=pwm::Duty;
}
