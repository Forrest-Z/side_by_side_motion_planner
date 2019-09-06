#include "param.hpp"
//#include <math.h>
#include <cmath>
using namespace std;
Param::Param(){}

Param::Param(double a, double b, double c, double k){
  this->a=a;
  this->b=b;
  this->c=c;
  this->k=k;
  obstacle=false;
}

Param::Param(double a, double b, double k){
  this->a=a;
  this->b=b;
  this->k=k;
  obstacle=true;
}

double Param::util(double x){
  double unweighted;
  if(obstacle){
    //return(x>0.5?0:-2);
    //unweighted= -1 * pow(abs(x/a),-2*b);
    unweighted = -abs( pow( x/a, -2*b ) ) ;
  }
  else
    //unweighted= -abs( pow( x/a, -2b  ) ;
    unweighted= 1 - pow( abs( (x-c) / a ) , 2*b ) ;
  return k*unweighted;
}
