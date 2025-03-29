/* 
 *  18650 Ion-Li battery
 */

#ifndef Battery_H
#define Battery_H

#include "Arduino.h"

#define DEFAULT_CONVERSION_FACTOR 1.718//1.718
#define DEFAULT_READS 100


  int getBatteryChargeLevel(bool useConversionTable = false);
  double getBatteryVolts(double conversionFactor);  //加入double conversionFactor 每次读取电压都重新新设置 系数 用于校准电压
  


#endif
