#include "Dcore.h"
//Macros
#define MAX(a,b) ((a)>(b)? a:b)
#define MIN(a,b) ((a)<(b)? a:b)
#define ARRSZ(a) (sizeof(a)/sizeof(a[0]))
#define DOMAIN(x,a,b) ((x)>=(a) && (x)<=(b))

#include <SetTimeout.h>
#include <ServoCAST.h>

uint8_t algor_param[]={
  10,9,153,4,  90,200,70,0,
  150,200,10,15,  25,31,30,0,
  20,30,4,100,  100,0,0,0,
  0,225,32,224,  41,157,51,103,
  65,66,92,37,  188,34,255,33,
  250,100,0,0,  30,30,100,0,
  0,0,0,0,  20,30,0,0,
  50,200,50,50, 60,0,0,0
};

//elapsed time
static uint16_t revs;
//observer vars
static float wmax,wh,bh,hcoef1,hcoef2,bf,wro;
//profile
static uint8_t iflag,iprof;
static int16_t ivmax; //duty maximum over the profile
static float ibbase; //bh minimum for every mode
static float ilngamm; //log gamma
static uint32_t itugamm; //elapsed gamma time in usec
static float idegamm; //gamma derivative
//controls
static uint8_t zflag,zovrd,zfblim;
static float zinteg;
//table
static uint8_t tbl_index;
//Ocillation analyzer
static int32_t fvalue;
static void (*ffunc)();
static uint16_t ftime;
#define FSAMP 33   //msec or 30Hz sampling

/**************************************************************************/
static int satuate(int val,int lo =0, int hi =255){
  return MIN(hi,MAX(lo,val));
}
static int interp(int y1,int y2,int dx,int w){
  if(w<0) return y1;
  else if(w>dx) return y2;
  else return (y2*w+y1*(dx-w))/dx;
}
static int readTbl4k(int p,int w,int32_t &dec){
  auto x1=PRM_ReadData(p)<<4;
  auto x2=PRM_ReadData(p+2)<<4;
  uint8_t y1,y2;
  if(w<0) w=0;
  if(w>4095) w=4095;
  tbl_index=0;
  while(x2<w){
    x1=x2;
    tbl_index++;
    p+=2;
    x2=PRM_ReadData(p+2)<<4;
  }
  y1=PRM_ReadData(p+1);
  y2=PRM_ReadData(p+3);
  int dx=x2-x1;
  dec=((int)y2-(int)y1)*1000/dx;
  auto y=interp(y1,y2,dx,w-x1);
  return MAX(0,y);
}
static int readProf(int prof_tbl,int prof_tmsec,uint8_t &idx,int32_t &dec){
  auto hval=readTbl4k(prof_tbl,prof_tmsec,dec);
  idx=tbl_index;
  return hval;
}
static int readProf(int prof_tbl,int idx){
  return PRM_ReadData(prof_tbl+(idx<<1)+1);
}
static int readGrad(int prof,int prof_tmsec,int w){
  int x1=PRM_ReadData(prof)<<4;
  int y1=PRM_ReadData(prof+1);
  int x2=PRM_ReadData(prof+2)<<4;
  int y2=PRM_ReadData(prof+3);
  for(;;){
    if(w<=y1 && w>y2){
      return x2>x1? (y2-y1)*1000/(x2-x1):0;
    }
    else if(x2>4000){
      return 0;
    }
    x1=x2;
    y1=y2;
    prof+=2;
    x2=PRM_ReadData(prof+2)<<4;
    y2=PRM_ReadData(prof+3);
  }
  return 0;
}
static void setPol(float polr,float poli){
  hcoef1=2*polr;
  hcoef2=polr*polr+poli*poli;
}
void algor_prepare(){
  revs=0;
  ilngamm=PRM_ReadData(17)*0.01;
  itugamm=0;
  idegamm=1;
}
uint16_t algor_update(int32_t dtu,int32_t otu){
  if(dtu==0) return 0;
//Measuring
  auto dt=dtu*1.0e-6;
  auto wrps=2*M_PI/dt;
  if(revs==0){
    iprof=iflag=zflag=0;
    wmax=wh=wro=wrps;
    bh=0;
    setPol(PRM_ReadData(5),0);
  }
  revs++;
  if(wmax<wrps) wmax=wrps;
  auto uat=(float)otu/dtu*PRM_ReadData(4)/(8*M_PI); //duty/Tau
  uint8_t nloop=dtu/500;
  if(nloop<2) nloop=2;
  auto dtn=dt/nloop;
  auto bho=bh;
  for(int i=0;i<nloop;i++){
    auto ii=i+1;
    auto wi=(ii*wrps+(nloop-ii)*wro)/nloop;
    auto werr=wi-wh;
    auto db=werr*hcoef2;
    wh=wh+(werr*hcoef1+bh-wi*uat)*dtn;
    bh=bh+db*dtn;
  }
  wro=wrps;
  float dbh=(bh-bho)/dt;
//Logger
// logger::stage.omega=round(wrps);
  logger::stage.beta=satuate(round(bh),-32768,32767);
  switch(PRM_ReadData(3)){
    case 1: logger::stage.eval=satuate(iflag*20,0,255); break;
    case 4: logger::stage.eval=satuate(fvalue,0,255); break;
    case 5: logger::stage.eval=satuate(ftime,0,255); break;
    default: logger::stage.eval=satuate(dcore::RunLevel*20,0,255); break;
  }

//i-Block: base profile
  int32_t igrad;
  auto ivalue=readProf((24),dcore::tmsec,iprof,igrad);
  int sigma=0;
  switch(iflag){
    case 0:
      iflag=1;
      ivmax=ivalue;
      ibbase=bh;
      fvalue=ftime=0;
      ffunc=NULL;
    case 1:
      if(ivmax<ivalue) ivmax=ivalue;
      if(ibbase<bh) ibbase=bh;
      if(wrps>PRM_ReadData10x(8)){
        iflag=2;
        break;
      }
      else if(dcore::tmsec>50 && bh<(int)PRM_ReadData100x(9)){
        iflag=4;
        wh=wrps;
        bh=ibbase=0;
        setPol(PRM_ReadData(6),0);
        dcore::shift();  //RunLevel =>4
        break;
      }
      break;
    case 2:
      if(ivmax<ivalue) ivmax=ivalue;
      if(ibbase<bh) ibbase=bh;
      else if(bh<(int)PRM_ReadData100x(9)){
        iflag=3;
        ibbase=bh;
        dcore::shift();  //RunLevel =>4
      }
      break;
    case 3:
      if(ivmax<ivalue) ivmax=ivalue;
      if(ibbase>bh) ibbase=bh;
      else if(bh-ibbase>(int)PRM_ReadData100x(10) || dcore::tmsec>PRM_ReadData10x(11)){
        iflag=4;
        wh=wrps;
        bh=ibbase=0;
        setPol(PRM_ReadData(6),0);
      }
      break;
    case 4:
      setTimeout.set(ffunc=[](){
        int t0=micros();
        fvalue=logger::analyze(PRM_ReadData10000x(16)/idegamm,PRM_ReadData(18),PRM_ReadData(19))*PRM_ReadData(20)/100;
        ftime=((int)micros()-t0)/1000;
        iflag=6;
        if(dcore::RunLevel>0) setTimeout.set(ffunc,FSAMP);
      },PRM_ReadData10x(12));
      iflag=5;
    case 5:
    case 6:{
      int bref=PRM_ReadData100x(14);
      sigma= (bh-bref)+PRM_ReadData(13)*dbh/wrps >0;
      if(iflag==6) idegamm-=ilngamm*idegamm*dt;
      itugamm+=dtu*idegamm;
      if(PRM_ReadData(3)==6) logger::stage.eval=satuate(itugamm/10000,0,255);
      break;
    }
  }

  int zmax=satuate(ivmax*zovrd/100,0,ivmax);
  int zmin=readProf((24),7);
  int zcmd=zmax;
  switch(zflag){
    case 0:   //speed is low
      zflag=1;
      zinteg=0;  //integral
    case 1:
      zflag=iflag;
      zovrd=interp(PRM_ReadData(41),100,PRM_ReadData10x(8),wrps);
      zcmd=0;
      break;
    case 2:   //tension > thres
      zflag=iflag;
      zovrd=100;
      zcmd=PRM_ReadData10x(40);   //overrun surpressor in 10usec
      break;
    case 3:   //down trend
      zflag=iflag;
      zcmd=satuate(zcmd,zmin,ivalue);
      break;
    case 4:
    case 5:{
      int prof=satuate(zcmd,zmin,ivalue);
      if(zinteg==0 || zinteg>prof) zinteg=prof;
      if(sigma){
        int ki=interp(0,PRM_ReadData(45),PRM_ReadData10x(12),itugamm/1000);
        zinteg-=(bh*ki/10000)*zinteg*dt;
        int ilow=prof*PRM_ReadData(44)/100;
        if(zinteg<ilow) zinteg=ilow;
        zcmd=zmin;
      }
      else zcmd=zinteg;
      if(iflag>5){
        zinteg=interp(zinteg,prof,100,PRM_ReadData(46));
      }
      zflag=iflag;
      if(PRM_ReadData(3)==4) logger::stage.eval=satuate(zinteg,0,255);
      break;
    }
    case 6:{
      int er=fvalue-(int)PRM_ReadData(56);
      int ki=satuate(er*(int)PRM_ReadData(57)/100,0,100);
      zinteg+=(zinteg<ivalue? igrad*zinteg/ivalue : readGrad((24),dcore::tmsec,zinteg))*ki/100*dt;
      er=fvalue-(int)PRM_ReadData(58);
      int kp=satuate(er*(int)PRM_ReadData(59)/100,-PRM_ReadData(60),PRM_ReadData(60));
      zcmd=satuate(zinteg*(100-kp)/100,zmin,zmax);  //P control
      int td=PRM_ReadData10000x(52)/2;  //dithering period
      int ad=PRM_ReadData10x(53)/2;  //dithering amplitude
      if((itugamm/td)&1) ad=-ad;
      int kd=interp(-ad,ad,td,(itugamm%td));
      zcmd=satuate(zcmd-zmax*kd/1000,10,245);
      if(sigma && zcmd>zmin) zcmd=zmin;
    }
  }
  return zcmd;
}
