#include <Arduino.h>
#include <SetTimeout.h>
#include <mbed.h>
#include <rtos.h>

#define TICKS 10   //msec
#define VCAL 3.25 //V
#define DCAL 1024 //bits

#define TEMP_SENS A1
#define TEMP_ENABLE D0
#define TEMP_GND D2
#define TEMP_CAL 0.01  //V/deg
#define TEMP_ZERO 0.6  // V at zero
#define TEMP_THRES 40

#define TEMP_LAMP D7
#define UPS_LAMP D8
#define RUN_LAMP D9

float temp=0;
int battery=0;
long ts_recv=0;  //timestamp when  recieved string via Serial
long wdt=0;  //elapsed time since the last recieve event

rtos::Thread th_recv;

void sampler(){
  digitalWrite(TEMP_ENABLE,HIGH);
  int aval=analogRead(TEMP_SENS);
  float volt=VCAL*aval/DCAL;
  float t=(volt-TEMP_ZERO)/TEMP_CAL;
  temp=temp+(t-temp)*0.1;
  setTimeout.set(sampler,200);
}
void logger(){
  Serial.print("inner_temp=");
  Serial.println(temp);
  Serial.print("battery=");
  Serial.println(battery);
  Serial.print("wdt=");
  Serial.println(wdt);
  setTimeout.set(logger,1000);
}

void TEMP_scan(){
  if(temp>TEMP_THRES){
    digitalWrite(TEMP_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(TEMP_LAMP,LOW);
    },333);
    setTimeout.set(TEMP_scan,1000);
  }
  else{
    digitalWrite(TEMP_LAMP,LOW);
    setTimeout.set(TEMP_scan,1000);
  }
}

void UPS_scan(){
  if(battery==0){
    digitalWrite(UPS_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(UPS_LAMP,LOW);
    },333);
    setTimeout.set(UPS_scan,1000);
  }
  else{
    digitalWrite(UPS_LAMP,HIGH);
    setTimeout.set(UPS_scan,1000);
  }
}

void RUN_scan(){
  wdt=millis()-ts_recv;
  if(wdt>5000){
    digitalWrite(RUN_LAMP,HIGH);
    digitalWrite(TEMP_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(RUN_LAMP,LOW);
      digitalWrite(TEMP_LAMP,LOW);
    },30);
    setTimeout.set(RUN_scan,200);
  }
  else{
    digitalWrite(RUN_LAMP,HIGH);
    setTimeout.set([]{
      digitalWrite(RUN_LAMP,LOW);
    },333);
    setTimeout.set(RUN_scan,1000);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TEMP_ENABLE,OUTPUT);
  pinMode(TEMP_GND,OUTPUT);
  digitalWrite(TEMP_LAMP,HIGH);
  delay(2000);
  digitalWrite(TEMP_LAMP,LOW);
  pinMode(RUN_LAMP,OUTPUT);
  digitalWrite(RUN_LAMP,HIGH);
  delay(2000);
  digitalWrite(RUN_LAMP,LOW);
  pinMode(UPS_LAMP,OUTPUT);
  digitalWrite(UPS_LAMP,HIGH);
  delay(2000);
  digitalWrite(UPS_LAMP,LOW);
  setTimeout.set([](){
    RUN_scan();
    UPS_scan();
    TEMP_scan();
  },2000);

  logger();
  sampler();

  th_recv.start( mbed::callback([](){  //Thread for serial recieve
    while(true){
      String str = Serial.readString();
      String args[2] = {"\0"};
      int index = split(str, '=', args);
      if(index<0) continue;
      if(args[0].equals( "battery") == true ){
        battery = args[1].toInt();  
        ts_recv=millis();
      }
    }
  }));
  ts_recv=millis();

  NRF_WDT->CONFIG=0x01;     // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV=3276L*3;      // Timeout[s] = (CRV-1)/32768
  NRF_WDT->RREN=0x01;       // Enable the RR[0] reload register
  NRF_WDT->TASKS_START=1;   // Start WDT
}

int split(String data, char delimiter, String *dst){
  if(data.indexOf(delimiter)<0) return -1;
  int index = 0;
  int datalength = data.length();

  for (int i  = 0; i  < datalength; i++){
    char tmp = data.charAt(i);
    if ( tmp == delimiter) {
      index++;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}

void loop() {
  setTimeout.spinOnce();
  delay(10);
  NRF_WDT->RR[0]=WDT_RR_RR_Reload;
}
