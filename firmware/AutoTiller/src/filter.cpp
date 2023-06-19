#include "filter.h"
#include "Arduino.h"
#define LENGTHNOTSET 0
filter::filter(int fl)
{
  filterLength = fl;
  vals = new float[filterLength];
}
filter::filter(){
  filterLength = LENGTHNOTSET;
}

void filter::setLength(int len){
  
  filterLength = len;
  vals = new float[filterLength];
  for(int i = 0; i<filterLength; i++){
    vals[i] = 0;
  }
}

bool filter::checkIndex(uint8_t index)
{
  if(filterLength == LENGTHNOTSET){
    return false;
  }
  return !(index < 0 || index >= filterLength);
}

void filter::input(float inputVal)
{
  if(filterLength == LENGTHNOTSET){
    return;
  }
  for (int i = filterLength - 1; i > 0; i--)
  {
    vals[i] = vals[i - 1];
  }
  vals[0] = inputVal;
}

float filter::getValue(uint8_t index)
{
  if(filterLength == LENGTHNOTSET){
    return 0;
  }
  if (index >= filterLength || index < 0)
  {
    return NULL;
  }
  if (index != 0)
  {
    return vals[index];
  }
  float sum = 0;
  for (int i = 0; i < filterLength; i++)
  {
    sum += vals[i];
  }
  return sum / filterLength;
}

float filter::getValueScaled(int scale, uint8_t index)
{
  if(filterLength == LENGTHNOTSET){
    return 0;
  }
  if (index >= filterLength || index < 0)
  {
    return NULL;
  }
  if (index != 0)
  {
    return vals[index];
  }
  float sum = 0;
  for (int i = 0; i < filterLength; i++)
  {
    sum += vals[i];
  }
  return scale * (sum / filterLength);
}

float filter::getDeriv(uint8_t index)
{
  if(filterLength == LENGTHNOTSET){
    return 0;
  }
  if (!checkIndex(index))
  {
    return 0;
  }

  return vals[0] - vals[index];
}

float filter::getIntegral()
{
  if(filterLength == LENGTHNOTSET){
    return 0;
  }
  return NULL;
}
