#include <SetTimeout.h>
#include <ServoCAST.h>
#include <ServoCAST_nRF52.h>
#include <ServoCAST_Algor.h>

//params Algor.cpp
uint8_t algor_param[]={
  10,9,0,4,  150,200,70,0,
  150,200,10,15,  20,150,30,33,
  20,33,5,1,  50,0,0,0,
  0,116,12,120,  25,117,33,110,
  64,60,128,44,  193,38,255,33,
  200,100,50,0,  10,40,30,0,
  10,10,0,40,  70,0,5,0
};

void setup() {
  Serial.begin(115200);
  param::run(algor_param,sizeof(algor_param));

  int di_sen=getPin(PRM_ReadData(0));
  int do_fet=getPin(PRM_ReadData(1));
  pinMode(di_sen,INPUT);  //rotation sensor
  pinMode(do_fet,OUTPUT);  //FET
  dcore::config(di_sen,do_fet,1,0);
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
      logger::dump();
    }
  );

//  while (!Serial);

  ble::run(
    "arDCino",  //device name 
    "10014246-f5b0-e881-09ab-42000ba24f83",  //service uuid
    "20024246-f5b0-e881-09ab-42000ba24f83",  //request uuid
    "20054246-f5b0-e881-09ab-42000ba24f83"   //notification uuid
  );
  ble::led_pin=LED_PWR;
#if defined(ARDUINO_SEEED_XIAO_NRF52840) || defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE)
  ble::led_invert=true;
#endif
  pinMode(LEDB,INPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR,HIGH);//Power LED Turn off
}

void loop() {
  if(setTimeout.spinOnce()==NULL){
    if(millis()>500) dcore::sleep(10);
    NRF_WDT->RR[0]=WDT_RR_RR_Reload;
  }
}
