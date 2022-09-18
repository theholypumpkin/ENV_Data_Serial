#ifndef Arduino_h
    #include <Arduino.h>
#endif 
#ifndef _MAIN_HPP
#define _MAIN_HPP
void setup();
void loop();
void setReadFlagISRCallback();
void setupEEPROM();
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue);
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue, 
    float temperatureValue, float humidityValue);
bool readDHTSensor(float &temperatureValue, float &humidityValue);
uint16_t readGP2YSensor(uint16_t numberOfSamples = (uint16_t)20U);
float readGPY2SensorBaselineCanidate();
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue,
    float temperatureValue, float humidityValue, float dustSensorBaseline);
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue,
    float dustSensorBaseline);
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue,
    float temperatureValue, float humidityValue);
void transmitSerial(uint16_t eco2Value, uint16_t tvocValue, uint16_t dustDensityValue);
#endif