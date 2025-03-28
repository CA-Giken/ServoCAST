#ifndef ServoCAST
#define ServoCAST

#include  "Dcore.h"
#include  "Logger.h"
#include  "Param.h"

inline int getPin(int n){
  switch(n){
    case 0: return D0;
    case 1: return D1;
    case 2: return D2;
    case 3: return D3;
    case 4: return D4;
    case 5: return D5;
    case 6: return D6;
    case 7: return D7;
    case 8: return D8;
    case 9: return D9;
    case 10: return D10;
  }
}

#endif
