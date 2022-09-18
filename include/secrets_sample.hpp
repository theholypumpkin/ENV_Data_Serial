#ifndef Arduino_h
    #include <Arduino.h>
#endif
#ifndef _SECRETS_HPP
#define _SECRETS_HPP
extern const char* g_name = "a name for your sensor";
extern const char* g_location = "a location where your sensor is located";
extern const char* g_influxDbMeasurement = "the influxdb measurment name";
#endif