#ifndef PARAM_H
#define PARAM_H
class Param
{
 public:
  Param();
  Param(double a, double b, double c, double k);
  Param(double a, double b, double k);
  double util(double x);
  
 private:

  double a;
  double b;
  double c;
  double k;
  bool obstacle;
};

#endif
