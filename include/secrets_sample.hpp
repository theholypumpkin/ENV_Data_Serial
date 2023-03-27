#ifndef Arduino_h
    #include <Arduino.h>
#endif
#ifndef _SECRETS_HPP
#define _SECRETS_HPP

#define DASH "-"
#define G_NAME "a name you like"
#define HA_TEMP_ENTITY_ID G_NAME DASH "temp"
#define HA_HMD_ENTITY_ID G_NAME DASH "hmd"
#define HA_HI_ENTITY_ID G_NAME DASH "hI"
#define HA_CO2_ENTITY_ID G_NAME DASH "co2"
#define HA_AQI_ENTITY_ID G_NAME DASH "aqi"
#define HA_BATV_ENTITY_ID G_NAME DASH "batV"
#define HA_BATP_ENTITY_ID G_NAME DASH "batP"
#define HA_RSSI_ENTITY_ID G_NAME DASH "rssi"
#define HA_LOC_ENTITY_ID G_NAME DASH "loc"
extern const char* g_name = G_NAME;
extern const char* g_location = "kitchen or whatever";
extern const IPAddress g_staticIPAddress(0,0,0,0);
extern const IPAddress g_DNSAddress(1,1,1,1);
extern const IPAddress g_gateway(0,0,0,0);
extern const IPAddress g_subnet(255,255,255,0);
extern const IPAddress g_mqttServerUrl(0,0,0,0);
extern const uint16_t g_mqttServerPort = 8883;
extern const char* g_mqttUsername = "your usename";
extern const char* g_mqttPassword = "your password";
extern const char* g_wifiSsid = "wifi name";
extern const char* g_wifiPass = "wifi password";
extern const char* g_ntpTimeServerURL = "http://some-url.net";

extern const char* g_indoorAirQualityTopic = "data/topic/";
#endif