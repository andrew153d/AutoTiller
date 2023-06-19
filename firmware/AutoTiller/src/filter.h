#ifndef filter_h
#define filter_h

#include "Arduino.h"


class filter
{
private:
  float *vals;
  bool checkIndex(uint8_t index);

public:
  int filterLength;
  filter();
  filter(int fl);
  void input(float inputVal);
  void setLength(int len);
  float getValue(uint8_t index = 0);
  float getValueScaled(int scale, uint8_t index = 0);
  float getDeriv(uint8_t index = 1);
  float getIntegral();
  bool iscloseTo(int val, float precision);
};

#endif