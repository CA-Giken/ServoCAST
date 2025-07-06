#ifndef NAPPROXH
#define NAPPROXH

#include "lu.h"

template <typename buffer_t> inline int approx(int dim,buffer_t *dat,int samp,int wgh,double *ans){
  double snv[POLYNOMINAL];
  double snn[POLYNOMINAL*2];
  for(int i=0;i<dim;i++) snn[2*i]=snn[2*i+1]=snv[i]=0;
  int hamp=samp/2;
  double wg=wgh*0.01;
  for(int i=0,n=-hamp;i<samp;i++,n++){
    double nn=1;
    double w=(i*wg+(samp-i))/samp;
    double y=dat[i];
    double gmma=(double)n/hamp;
    for(int d=0;d<dim*2;d++,nn*=gmma){
      double wnn=w*nn;
      snn[d]+=wnn;
      if(d<dim) snv[d]+=wnn*y;
    }
  }
  double lu[POLYNOMINAL*POLYNOMINAL];
  double *tg=lu;
  for(int i=0;i<dim;i++){
    double *sc=snn+i;
    for(int j=0;j<dim;j++,tg++) *tg=sc[j];
  }
  int pivot[POLYNOMINAL];
  LU_decomposition(dim, pivot, lu);
  LU_solver(dim, pivot, lu, snv, ans);
  return hamp;
}

#endif
