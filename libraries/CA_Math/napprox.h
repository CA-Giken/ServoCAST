#ifndef NAPPROXH
#define NAPPROXH

#include "lu.h"

inline int approx(int dim,int *dat,int samp,double *ans){
  double snv[POLYNOMINAL];
  double snn[POLYNOMINAL*2];
  for(int i=0;i<dim;i++) snn[2*i]=snn[2*i+1]=snv[i]=0;
  int hamp=samp/2;
  for(int i=0,n=-hamp;i<samp;i++,n++){
    double nn=1;
    double y=dat[i];
    double gmma=(double)n/hamp;
    for(int d=0;d<dim*2;d++,nn*=gmma){
      snn[d]+=nn;
      if(d<dim) snv[d]+=nn*y;
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
