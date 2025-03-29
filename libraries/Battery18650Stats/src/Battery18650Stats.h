/* 
 *  18650 Ion-Li battery
 */

#ifndef Battery_18650_Stats_H
#define Battery_18650_Stats_H

#include "Arduino.h"

#define DEFAULT_PIN 35
#define DEFAULT_CONVERSION_FACTOR 1.718//1.718
#define DEFAULT_READS 100

class Battery18650Stats {
 public:
  Battery18650Stats();
  ~Battery18650Stats();
  Battery18650Stats(int adcPin);
  Battery18650Stats(int adcPin, double conversionFactor);
  Battery18650Stats(int adcPin, double conversionFactor, int reads);


  int getBatteryChargeLevel(bool useConversionTable = false);
  double getBatteryVolts(double conversionFactor);  //加入double conversionFactor 每次读取电压都重新新设置 系数 用于校准电压
  

 private:
  int _adcPin;
  int _reads;
  double _conversionFactor;
  double *_conversionTable = nullptr;

  void _initConversionTable();
  int _getChargeLevelFromConversionTable(double volts);
  int _calculateChargeLevel(double volts);
  int _avgAnalogRead(int pinNumber, int reads);
  double _analogReadToVolts(int readValue);
};

#endif
