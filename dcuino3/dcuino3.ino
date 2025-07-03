#include <SetTimeout.h>
#include <ServoCAST.h>
#include <ServoCAST_nRF52.h>

//Algor.cpp
extern void algor_prepare();
extern uint16_t algor_update(int32_t time,int32_t duty);
extern uint8_t algor_param[8*7];

void setup() {
  Serial.begin(115200);
  param::run(algor_param,sizeof(algor_param));

  int di_sen=getPin(PRM_ReadData(0));
  int do_fet=getPin(PRM_ReadData(1));
  pinMode(di_sen,INPUT);  //rotation sensor
  pinMode(do_fet,OUTPUT);  //FET
  dcore::config(di_sen,do_fet,1,PRM_ReadData(2),PRM_ReadData(2)%10);
  dcore::run(
    [](){//start callback
      algor_prepare();
      NRF_WDT->CONFIG=0x01;     // Configure WDT to run when CPU is asleep
      NRF_WDT->CRV=3276L*3;      // Timeout[s] = (CRV-1)/32768
      NRF_WDT->RREN=0x01;       // Enable the RR[0] reload register
      NRF_WDT->TASKS_START=1;   // Start WDT
    },
    [](int32_t dt,int32_t on_dt){//cyclic callback
      uint16_t d=algor_update(dt,on_dt);
      return d;
    },
    [](){//end callback 
      ble::logdump();
      digitalWrite(LEDR,HIGH);
      digitalWrite(LEDG,HIGH);
    }
  );

  ble::run(
    "arDCino",  //device name 
    "10014246-f5b0-e881-09ab-42000ba24f83",  //service uuid
    "20024246-f5b0-e881-09ab-42000ba24f83",  //request uuid
    "20054246-f5b0-e881-09ab-42000ba24f83"   //notification uuid
  );
  ble::led_pin=LEDB;
#if defined(ARDUINO_SEEED_XIAO_NRF52840) || defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE)
  ble::led_invert=true;
#endif
  pinMode(LEDR,OUTPUT);
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);
  digitalWrite(LEDR,HIGH);
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDB,LOW);
  setTimeout.set([]{
    digitalWrite(LEDB,HIGH);//Power LED Turn off
  },1000);
}

void loop() {
  if(setTimeout.spinOnce()==NULL){
    if(millis()>100 && dcore::RunLevel==0) dcore::sleep(10);
    NRF_WDT->RR[0]=WDT_RR_RR_Reload;
    logger::sweep();
  }
}