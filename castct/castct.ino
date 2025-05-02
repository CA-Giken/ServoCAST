#include <SetTimeout.h>
#include <ServoCAST.h>
#include <ServoCAST_nRF52.h>

uint8_t algor_param[]={
  0,0,0,0,  0,0,0,0,
};
uint16_t algor_prepare(){}
uint16_t algor_update(int32_t time,int32_t duty){
//  if(dcore::RunLevel==3) dcore::shift();  //RunLevel =>4
  return 0;
}

void setup() {
  Serial.begin(115200);
  param::run(algor_param,sizeof(algor_param));
//  while(!Serial);
  pinMode(LEDB,OUTPUT);
  pinMode(LEDR,OUTPUT);
  pinMode(D7,OUTPUT);  //Anode
  pinMode(D8,OUTPUT);  //Vcc
  pinMode(D9,INPUT);  //Comp. out

  dcore::config(D9,LEDR,0,1000);
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
      Serial.print("end callback ");
      Serial.println(logger::length());
      if(logger::length()<50) return;
      ble::logdump();
//      logger::dump();
    }
  );

  ble::run(
    "arDCino",  //device name 
    "10014246-f5b0-e881-09ab-42000ba24f83",  //service uuid
    "20024246-f5b0-e881-09ab-42000ba24f83",  //request uuid
    "20054246-f5b0-e881-09ab-42000ba24f83"   //notification uuid
  );
  ble::led_pin=LED_PWR;
  ble::led_invert=true;    //for Seeed Xiao
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR,HIGH);//Power LED Turn off
}

void loop() {
  if(setTimeout.spinOnce()==NULL){
    if(millis()>500) dcore::sleep(10);
    NRF_WDT->RR[0]=WDT_RR_RR_Reload;
    digitalWrite(D7,HIGH);
    digitalWrite(D8,HIGH);
    analogWrite(A5,8);  //comparator reference
    if(digitalRead(D9)){
      digitalWrite(LEDB,HIGH);
    }
    else{
      digitalWrite(LEDB,LOW);
    }
  }
}
