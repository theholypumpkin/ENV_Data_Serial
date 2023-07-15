/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
//#include <ArduinoJson.h>
#include <JC_Button.h>
//#include <PubSubClient.h>
#include <ArduinoHA.h>
#include <cmath>
#include <cstring>
#include <RTCZero.h> //Also inplements sleep
/* SAMD21. SAMD51 chips do not have EEPROM. This library provides an EEPROM-like API and hence
 * allows the same code to be used.
 */
#ifdef ARDUINO_SAMD_NANO_33_IOT
#include <FlashAsEEPROM.h>
#endif
/*================================================================================================*/
#define RANDOM_SEED_ADC_PIN A2 //NEVER CONNECT A SENSOR TO THIS PIN
#define BATTERY_VOLTAGE_ADC_PIN A6
#define MAX_1676_LOW_BATTERY_OUT_INTERRUPT_PIN 21 // LBO PIN (A7)
#define CCS_811_INTERRUPT_PIN 15 //A1
#define CCS_811_nWAKE 14 //A0
#define DHTPIN 7
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 8
#define EEPROM_EMULATION_SIZE 64 //We really only need 2 bytes but it still less the the default 1k.
#define DOCUMENT_SIZE 511 //If publishing fails, increase the Document Size
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    READ_BATTERY,
    PUBLISH_MQTT,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
uint8_t g_lastRtcUpdateDay;
uint16_t g_uuid;
//We use two 9V block batteries to keep current low
const float MAX_BATTERY_VOLTAGE = 4.2, //use a R1 = 1k, 3.3k or or higher in same ratio.
            ADC_VOLTAGE_FACTOR = MAX_BATTERY_VOLTAGE / powf(2.0, ADC_RESOLUTION);

volatile bool b_isrFlag = false; // a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
RTCZero rtc;
DHT tempHmdSensor(DHTPIN, DHTTYPE); // Create the DHT object
Adafruit_CCS811 co2Sensor;
WiFiClient wifiClient;
//PubSubClient mqttClient(g_mqttServerUrl, g_mqttServerPort, wifiClient);
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
HADevice device;
HAMqtt mqttClient(wifiClient, device);

HASensorNumber tempHASensor(HA_TEMP_ENTITY_ID, HASensorNumber::PrecisionP2);
HASensorNumber hmdHASensor(HA_HMD_ENTITY_ID, HASensorNumber::PrecisionP2);
HASensorNumber heatIndexHASensor(HA_HI_ENTITY_ID, HASensorNumber::PrecisionP2);
HASensorNumber co2HASensor(HA_CO2_ENTITY_ID);
HASensorNumber tvocHASensor(HA_AQI_ENTITY_ID);
HASensorNumber batVoltageHASensor(HA_BATV_ENTITY_ID, HASensorNumber::PrecisionP2);
HASensorNumber batPercentHASensor(HA_BATP_ENTITY_ID, HASensorNumber::PrecisionP2);
HASensorNumber rssiHASensor(HA_RSSI_ENTITY_ID);
HASensor locationHASensor(HA_LOC_ENTITY_ID);
/*================================================================================================*/
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    analogReadResolution(ADC_RESOLUTION);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    Serial1.begin(9600);            // Use Hardware Serial (not USB Serial) to debug
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    setupEEPROM();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic engine of CCS811
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if (!co2Sensor.begin())
    {
        while (1)
        {
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); // When falure blink LED rapidly
            Serial1.println(F("CCS Error"));
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Begin the Wifi Connection
    Serial1.print(F("Connecting..."));
    WiFi.setHostname(g_name);
    //esablishing wifi connection
    while(WiFi.begin(g_wifiSsid, g_wifiPass) != WL_CONNECTED){ //retry connect indef
        delay(100);
        Serial1.print(F("."));
        //TODO Wire gpio to reset pin and triger it after some time
    }
    WiFi.lowPowerMode();
    Serial1.println(F("connected"));
    Serial1.print(F("IP Address: "));
    Serial1.println(WiFi.localIP());
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    uint16_tByte uuid_arr; //a union to convert uint16_t to a byte
    uuid_arr.i = g_uuid;
    byte arr[2] = {uuid_arr.b[1], uuid_arr.b[0]}; //flip endianness

    device.setUniqueId(arr, 2);
    device.setName(g_name);
    mqttClient.begin(g_mqttServerUrl, g_mqttServerPort, g_mqttUsername, g_mqttPassword, 90);
    mqttClient.loop(); //connect for the first time
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Setting the parameters of the Sensors in a way Home Assitant understands them
    tempHASensor.setName("Temperature");
    tempHASensor.setUnitOfMeasurement("°C");
    tempHASensor.setIcon("mdi:temperature-celsius");
    tempHASensor.setDeviceClass("temperature");

    hmdHASensor.setName("Humidity");
    hmdHASensor.setUnitOfMeasurement("%");
    hmdHASensor.setIcon("mdi:water-percent");
    hmdHASensor.setDeviceClass("humidity");

    heatIndexHASensor.setName("Heat Index");
    heatIndexHASensor.setUnitOfMeasurement("°C");
    heatIndexHASensor.setIcon("mdi:temperature-celsius");
    heatIndexHASensor.setDeviceClass("temperature");

    delayMicroseconds(100);
    
    co2HASensor.setName("CO2");
    co2HASensor.setUnitOfMeasurement("ppm");
    co2HASensor.setIcon("mdi:molecule-co2");
    co2HASensor.setDeviceClass("carbon_dioxide");

    delayMicroseconds(100);

    tvocHASensor.setName("Air Quality");
    tvocHASensor.setUnitOfMeasurement("ppb");
    tvocHASensor.setIcon("mdi:air-filter");
    tvocHASensor.setDeviceClass("volatile_organic_compounds");

    delayMicroseconds(100);

    batVoltageHASensor.setName("Battery Voltage");
    batVoltageHASensor.setUnitOfMeasurement("V");
    batVoltageHASensor.setIcon("mdi:battery");
    batVoltageHASensor.setDeviceClass("voltage");

    delayMicroseconds(100);

    batPercentHASensor.setName("Battery Percentage");
    batPercentHASensor.setUnitOfMeasurement("%");
    batPercentHASensor.setIcon("mdi:battery");
    batPercentHASensor.setDeviceClass("battery");

    delayMicroseconds(100);

    rssiHASensor.setName("WiFi Signal Strength");
    rssiHASensor.setUnitOfMeasurement("dBm");
    rssiHASensor.setIcon("mdi:wifi-strength-3-alert");
    rssiHASensor.setDeviceClass("signal_strength");
    
    delayMicroseconds(100);

    locationHASensor.setName("Roomname");
    locationHASensor.setIcon("mdi:house");
    delayMicroseconds(100);
    locationHASensor.setValue(g_location);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    rtc.begin(); //begin the rtc at "random time" probably  Jan 1 2000 at 00:00:00 o'clock
    g_lastRtcUpdateDay = rtc.getDay();
    if(updateNetworkTime()) //set real time acording to network
        Serial1.println(F("Network Time successful"));
    else
        Serial1.println(F("ERROR: No Network Time"));
    rtc.setAlarmSeconds(rtc.getSeconds()-1);
    rtc.enableAlarm(rtc.MATCH_SS); //Set Alarm every minute
    rtc.attachInterrupt(alarmISRCallback); //When alarm trigger this callback.
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Attach LBO Interrupt
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delay(10000); // give us some time to upload a new program
}
/*________________________________________________________________________________________________*/
/**
 * @brief The main loop.
 * Because the microcontroller has native USB-Serial and we are constantly connected to a power
 * source, we will not sleep, because this would mess with the server software which reads the
 * Serial connection.
 */
void loop()
{
    // make static to retain variables even if out of scope.
    static uint16_t eco2Value, tvocValue;
    static float temperatureValue, humidityValue, heatIndexValue, batteryPercentage, batteryVoltage;
    static bool b_ENVDataCorrection;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if (b_isrFlag)
    {
        e_state = READ_DHT_SENSOR; // This is safer than setting the value inside the ISR itself
        b_isrFlag = false;
    }
    if(g_lastRtcUpdateDay != rtc.getDay()){ //update network time daily
        if(updateNetworkTime()){ //set real time acording to network
            Serial1.println(F("Updated Network Time"));
        }else{
            Serial1.println(F("Network Time Update failed"));
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch(e_state){
    case IDLE:
        Serial1.println(F("Sleeping now"));
        rtc.standbyMode();
        //delay(60000); //delay a minute
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue, heatIndexValue);
        e_state = READ_CCS_SENSOR;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if (b_ENVDataCorrection)
        {
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }
        else
        {
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = READ_BATTERY;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_BATTERY:
        batteryVoltage = analogRead(BATTERY_VOLTAGE_ADC_PIN) * ADC_VOLTAGE_FACTOR;
        batteryPercentage = calcBatteryPercentageLiPo(batteryVoltage);
        e_state = PUBLISH_MQTT;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case PUBLISH_MQTT:
        mqttClient.loop();
        long wifiSignalStrength = WiFi.RSSI();
        if (b_ENVDataCorrection)
        {
            publishMQTT(eco2Value, tvocValue, wifiSignalStrength, temperatureValue, 
            humidityValue, heatIndexValue, batteryVoltage, batteryPercentage);
        }
        else
        {
            publishMQTT(eco2Value, tvocValue, wifiSignalStrength, batteryVoltage, 
            batteryPercentage);
        }
        //mqttClient.loop();
        e_state = IDLE;
        break;
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Set the Read Flag ISR Callback
 */
void alarmISRCallback()
{
    b_isrFlag = true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Connects to WiFi using a static IP-Address, DNS, Gateway and Subnet than
 * it fetchs the latest network time and set the Real Time Clock to this time.
 * Afterwards it disconnects from wifi to conserve battery.
 */
bool updateNetworkTime(){
    WiFiUDP ntpUdpObject;
    NTPClient ntpClient(ntpUdpObject, g_ntpTimeServerURL, 7200);
    ntpClient.begin();
    uint8_t connection_attempts = 0;
    while(!ntpClient.update()){ //attempt to connect to ntp server up to 5 times.
        connection_attempts++;
        if(connection_attempts == 4){
            return false;
        }
        delay(1000);
    }
    unsigned long epochTime = ntpClient.getEpochTime();
    rtc.setEpoch(epochTime);
    g_lastRtcUpdateDay = rtc.getDay(); //set to actual day
    return true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief writes a random UUID to EEPROM (on SAMD chips to Flash) if never set, else it reads the
 * existing UUID from the EEPROM (or on SAMD chips to Flash). If in the first 5 seconds of the setup
 * a button is pressed, it will clear the flash and generate a new UUID
 */
void setupEEPROM()
{
    Serial1.print(F("Awaiting Button Press"));
    //BUG UUID is alway reset it should stay the same!
    Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN);
    eepromClearButton.begin();
    unsigned long loopEnd = millis() + 5000;
    while (millis() < loopEnd)
    { // check for 5 seconds if the button is pressed
        #ifdef ARDUINO_SAMD_NANO_33_IOT
        if(!EEPROM.isValid()) //If never written to the EEPROM Emulation, ignore the button.
        #endif
            break;
        Serial1.print(F("."));
        eepromClearButton.read();
        if (eepromClearButton.isPressed())
        {
            Serial.println(F("Button was pressed"));
            // This loop will take about 3.3*256 ms to complete which is about 0.85 seconds.
            for (uint16_t i = 0; i < EEPROM.length(); i++)
            {
                EEPROM.write(i, 0);
            }
            break; // when we reset the eeprom terminate the while loop
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /* For compability reasons and portability to other microcontroller we will not use the more
     * convinient EEPROM.get(addr, g_uuid); and EEPROM.put(addr, g_uuid); methods.
     * Instead we read individual bytes in the little endian order as it is standard on the AVR
     * and common on the ARM Architecture.
     */
    uint16_t addr = 0;
    uint16_t uuidUpperByte = (uint16_t)EEPROM.read(addr + 1) << 8;
    uint16_t uuidLowerByte = (uint16_t)EEPROM.read(addr);
    g_uuid = uuidUpperByte + uuidLowerByte; // leftshift by 8 bit
    Serial1.print(F("Current UUID: "));
    Serial1.println(g_uuid, HEX);
    /* Under the curcumstance that we had reset the eeprom once all bytes are 0.
     * When we never wrote anything to the EEPROM of the microcontroller all bytes will be FF.
     * Because we have a two byte variable, we have to check of the value not 0 or FFFF
     *
     * We only generate a random variable between 0x100 and 0xFFFE, to we always use both bytes,
     * but never use the "never written" value of FFFF.
     */
    if ((g_uuid == 0x0) || (g_uuid == 0xFFFF))
    {
        // When we have to generate a new UUID generate new seed for Random function
        randomSeed(analogRead(RANDOM_SEED_ADC_PIN));
        g_uuid = (uint16_t)random(0x100, 0xFFFE); // generates random uuid in range of 0x100 to 0xFFFE
        uuidUpperByte = g_uuid >> 8;
        uuidLowerByte = (g_uuid << 8) >> 8; // remove the upper 8 bit
        // EEPROM.put(addr, g_uuid);
        // Write the generated UUID to EEPROM
        EEPROM.write(addr, (uint8_t)uuidLowerByte);
        EEPROM.write(addr + 1, (uint8_t)uuidUpperByte);
        #ifdef ARDUINO_SAMD_NANO_33_IOT
        EEPROM.commit(); //Accually writing the data to the EEPROM Emulation!
        #endif
    }
    Serial1.print(F("New UUID: "));
    Serial1.println(g_uuid, HEX);
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 *
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the tvoc Value should be saved at
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue)
{
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic Engine of co2Sensor
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    if (!co2Sensor.readData())
    {
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 *
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the eco2 Value should be saved at
 * @param temperatureValue The current temperature to create a more accurate reading
 * @param humidityValue The current humidity to create a more accurate reading
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue,
                   float temperatureValue, float humidityValue)
{
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic Engine of co2Sensor
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    co2Sensor.setEnvironmentalData(humidityValue, temperatureValue);
    if (!co2Sensor.readData())
    {
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief Gets the Current Temperature and Humidity
 *
 * @param temperatureValue A Reference where the temperature Value should be saved at
 * @param humidityValue A Reference where the humidity Value should be saved at
 * @param heatIndexValue A Reference where the heat index Value should be saved at
 * @return true if the Reading was sucessful
 * @return false if the reading was unsucessful and the read value is "Not A Number"
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue, float &heatIndexValue)
{
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue))
    {
        return false;
    }
    heatIndexValue = tempHmdSensor.computeHeatIndex(temperatureValue, humidityValue, false);
    return true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 *
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param rssi The WiFi Signal Strength
 * @param voltageValue the battery voltage
 * @param percentageValue the calculated battery percentage
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssiValue, 
float voltageValue, float percentageValue)
{
    bool success = true;
    success = co2HASensor.setValue(eco2Value);
    success = tvocHASensor.setValue(tvocValue);
    success = rssiHASensor.setValue(rssiValue);
    success = batVoltageHASensor.setValue(voltageValue);
    success = batPercentHASensor.setValue(percentageValue);

    char buff[128];
    sprintf(buff, 
        "mqtt successful: %s, eco2: %u, tvoc: %u, rssi: %li, voltage: %.2f, percentage: %.2f", 
        success ? "true" : "false", eco2Value, tvocValue, rssiValue, voltageValue, percentageValue
    );
    Serial1.println(buff);
    /*Serial1.print(F("eco2: ")); Serial1.print(eco2Value);
    Serial1.print(F("tvoc: ")); Serial1.print(tvocValue);
    Serial1.print(F("rssi: ")); Serial1.print(rssiValue);
    Serial1.print(F("voltage: ")); Serial1.print(voltageValue);
    Serial1.print(F("percentage: ")); Serial1.println(percentageValue);*/
} 
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 *
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param rssiValue The WiFi Signal Strength
 * @param temperatureValue The read Temperature Value
 * @param humidityValue The read Humidity Value
 * @param heatIndexValue The calculated heat index value
 * @param voltageValue the battery voltage
 * @param percentageValue the calculated battery percentage
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssiValue,
                 float temperatureValue, float humidityValue, float heatIndexValue, 
                 float voltageValue, float percentageValue)
{
    bool success = true;
    success = tempHASensor.setValue(temperatureValue);
    success = hmdHASensor.setValue(humidityValue);
    success = heatIndexHASensor.setValue(heatIndexValue);
    success = co2HASensor.setValue(eco2Value);
    success = tvocHASensor.setValue(tvocValue);
    success = rssiHASensor.setValue(rssiValue);
    success = batVoltageHASensor.setValue(voltageValue);
    success = batPercentHASensor.setValue(percentageValue);
    
    char buff[192];
    sprintf(buff, 
        "mqtt connected: %s, temperature: %.2f, humidity: %.2f, heatIndex: %.2f, eco2:%u, tvoc: %u, rssi: %li, voltage: %.2f, percentage: %.2f", 
        mqttClient.isConnected() ? "true" : "false", temperatureValue, humidityValue, heatIndexValue, eco2Value, 
        tvocValue, rssiValue, voltageValue, percentageValue
    );
    Serial1.println(buff);
    /*Serial1.print(F("temperature: ")); Serial1.print(temperatureValue);
    Serial1.print(F(", humidity: ")); Serial1.print(humidityValue);
    Serial1.print(F(", heatIndex: ")); Serial1.print(heatIndexValue);
    Serial1.print(F(", eco2: ")); Serial1.print(eco2Value);
    Serial1.print(F("tvoc: ")); Serial1.print(tvocValue);
    Serial1.print(F("rssi: ")); Serial1.print(rssiValue);
    Serial1.print(F("voltage: ")); Serial1.print(voltageValue);
    Serial1.print(F("percentage: ")); Serial1.println(percentageValue);*/
}
/*________________________________________________________________________________________________*/
/**
 * @brief Calculate the battery percentage acording to two differnet formulas depending on the
 * voltage read by the ADC.
 *
 * @param voltageValue the read battery voltage
 * @return float the battery percantage based on the formula y = 120x-404 for change above
 * 63% and 255x-930 for charge below 63%.
 */
float calcBatteryPercentageLiPo(float voltageValue)
{
    if(voltageValue > 3.896)
        return 120.0f*voltageValue-404;
    else if (voltageValue > 3.648)
        return 255.0f*voltageValue - 930.0f;      
    else
        return 0.0;
}
/*end of file*/