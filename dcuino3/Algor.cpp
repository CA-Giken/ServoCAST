//Macros
#define MAX(a,b) ((a)>(b)? a:b)
#define MIN(a,b) ((a)<(b)? a:b)
#define ARRSZ(a) (sizeof(a)/sizeof(a[0]))
#define DOMAIN(x,a,b) ((x)>=(a) && (x)<=(b))

#include <SetTimeout.h>
#include <ServoCAST.h>

uint8_t algor_param[]={
  10,9,153,4,  150,200,70,0,
  150,200,10,15,  20,150,30,70,
  20,50,5,1,  50,0,0,0,
  0,116,12,120,  25,117,33,110,
  64,60,128,44,  193,38,255,33,
  200,100,100,0,  10,40,25,0,
  10,10,0,20,  50,0,7,0
};

//elapsed time
static uint16_t revs;
//observer vars
static float wmax,wh,bh,hcoef1,hcoef2,bf,wro;
//profile
static uint8_t iflag,iprof;
static int16_t ivmax; //duty maximum over the profile
static float ibbase; //bh minimum durling mode 1
static uint16_t itsw1; //time switching from mode 1 to 2
static uint16_t itsw2; //time switching from mode 2 to 3
//controls
static uint8_t zflag,zovrd;
static float zinteg;
//table
static uint8_t tbl_index;
//Ocillation analyzer
static int32_t fvalue;
static uint16_t fduty;
static void (*ffunc)();
static uint16_t fspan;
static uint8_t fcema;

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
static void setPol(float polr,float poli){
  hcoef1=2*polr;
  hcoef2=polr*polr+poli*poli;
}
void algor_prepare(){
  revs=0;
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
  logger::stage.omega=round(wrps);
  logger::stage.beta=satuate(round(bh),-32768,32767);
  switch(PRM_ReadData(3)){
    case 0: logger::stage.eval=satuate(iflag*20,0,255); break;
    case 1: logger::stage.eval=satuate(dcore::RunLevel*20,0,255); break;
    case 4: logger::stage.eval=satuate(fvalue,0,255); break;
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
      fvalue=fduty=fcema=0;
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
        itsw1=dcore::tmsec;
        itsw2=itsw1+PRM_ReadData10x(12);
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
        itsw1=dcore::tmsec;
        itsw2=itsw1+(PRM_ReadData(12)<100? PRM_ReadData10x(12):(PRM_ReadData(12)%100*10)*wmax/PRM_ReadData10x(8));
        setPol(PRM_ReadData(6),0);
      }
      break;
    case 4:{
      int bref=PRM_ReadData100x(14);
      bref=interp(bref,bref*PRM_ReadData(15)/100,itsw2-itsw1,dcore::tmsec-itsw1);
      sigma= (bh-bref)+PRM_ReadData(13)*dbh/wrps >0;
      if(ffunc==NULL){
        fspan=PRM_ReadData10x(16);
        setTimeout.set(ffunc=[](){
          uint8_t ival=PRM_ReadData(20);
          if(dcore::RunLevel>0) setTimeout.set(ffunc,ival);
          if(iflag<5){
            int val=0;
            int n=0;
            int u1=(int)itsw1*1000;
            if(itsw2-itsw1>fspan) u1=((int)itsw2-(int)fspan)*1000;
            for(;;n++){
              logger::ALOG *p1=logger::trace(-n-1);
              if(p1==NULL || p1->stamp<u1) break;
              val+=p1->duty;
            }
            fduty=val/n;
          }
          int order=PRM_ReadData(18);
          int tspan=(1000*100+(dcore::tusec/1000-itsw2)*PRM_ReadData(17))*(int)fspan/100;
          fvalue=logger::analyze(tspan,order)>>PRM_ReadData(19);
          fcema=ival;
          iflag=5;
        },itsw2-itsw1);
      }
    }
    case 5:
      break;
  }

  int zcmd=ivalue;
  int zmax=satuate(ivmax*zovrd/100,0,ivalue);
  int zmin=ivmax*PRM_ReadData(48)/100;
  switch(zflag){
    case 0:   //speed is low
      zflag=1;
      zovrd=PRM_ReadData(42);
      zinteg=0;  //integral
    case 1:
      zflag=iflag;
      zcmd=0;
      break;
    case 2:   //tension > thres
      zflag=iflag;
      zovrd=100;
      zcmd=PRM_ReadData10x(40);   //overrun surpressor in 10usec
      break;
    case 3:   //down trend
      zflag=iflag;
      zcmd=(int)ivalue*PRM_ReadData(41)/100;    //initial middle brake
      break;
    case 4:{  //Collision state(Sliding mode control)
      int cm0=zcmd;
      if(sigma){
        zinteg+=bh*dt/2;
        zcmd=ivmax*PRM_ReadData(44)/100;    //low brake;
      }
      int cm1=satuate(cm0-cm0*zinteg/100*PRM_ReadData(46)/100,0,zmax);  //high brake
      if(!sigma){
        zcmd=satuate(cm1,ivmax*PRM_ReadData(45)/100,zmax);
      }
      if(iflag>4){
        zinteg=interp(fduty,ivalue,100,PRM_ReadData(49));
        zflag=5;
      }
      if(PRM_ReadData(3)==4) logger::stage.eval=satuate(zinteg,0,255);
      else if(PRM_ReadData(3)==5) logger::stage.eval=satuate(zinteg,0,255);
      break;
    }
    case 5:{ //Steady state(Tri-state-control)
      int refl=PRM_ReadData(51);  //fvalue low
      int refh=PRM_ReadData(52);  //fvalue high
      int stat= fvalue>refh? 1: fvalue<refl? -1:0;
      if(fcema){  //update zinteg
        if(stat>=0){
          float dz=igrad*0.001*fcema*zinteg/(int)ivalue;
          zinteg+=dz;
          if(stat>0){
            float dk=zinteg*PRM_ReadData(54)*0.01;
            zinteg-=dk;
          }
        }
        fcema=0;
      }
      zinteg=satuate(zinteg,zmin,zmax);
      zmax=ivmax*PRM_ReadData(53)/100;
      zmin= PRM_ReadData(55)==0? ivalue:fduty*PRM_ReadData(55)/100;
      if(zmax>ivalue) zmax=ivalue;
      if(zmin>ivalue) zmin=ivalue;
      zcmd= stat<0? MAX(zinteg,zmax) : stat>0? MIN(zinteg,zmin) : zinteg;
      if(PRM_ReadData(3)==5) logger::stage.eval=satuate(zinteg,0,255);
      break;
    }
  }
  return zcmd;
}
