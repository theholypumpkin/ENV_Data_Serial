/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
/*================================================================================================*/
#define CCS_811_INTERRUPT_PIN 6 // 0,1 are UART, 2,3 are i2c so 7 is the only remaining pin 
#define CCS_811_nWAKE 3
#define DHTPIN 7
#define DHTTYPE DHT11
#define COLUMS           16   //LCD columns
#define ROWS             2    //LCD rows
#define LCD_SPACE_SYMBOL 0x20 //space symbol from LCD ROM, see p.9 of GDM2004D datasheet
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    LCD_DISPLAY,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
volatile bool b_isrFlag = false; //a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
DHT tempHmdSensor(DHTPIN, DHTTYPE); //Create the DHT object
Adafruit_CCS811 co2Sensor;
LiquidCrystal_I2C lcd(0x27, 16, 2);
/*================================================================================================*/
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    * Attach hardware interrupt BEFORE Setting up the CCS 811 sensor, because of a race condition.
    * It cloud happen, that the pin was already driven low by the sensor, before we attach the
    * interrupt. When the pin is already low, we can no longer detect a falling edge, hence the ISR
    * would never trigger.
    */
    attachInterrupt(digitalPinToInterrupt(CCS_811_INTERRUPT_PIN), setReadFlagISRCallback, FALLING);
    digitalWrite(CCS_811_nWAKE, LOW); //Enable logic engine of CCS811
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if(!co2Sensor.begin()){
        while(1){
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); //When failure blink LED rapidly
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    co2Sensor.enableInterrupt();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable logic engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    Begin the LCD*/
    lcd.begin();
    lcd.backlight(HIGH);
    lcd.print("I'm working");
    delay(2000);

    lcd.clear();

}
/*________________________________________________________________________________________________*/
/**
 * @brief The main loop.
 * Because the microcontroller has native USB-Serial and we are constantly connected to a power
 * source, we will not sleep, because this would mess with the server software which reads the
 * Serial connection.
 */
void loop() {
    //make static to retain variables even if out of scope.
    static uint16_t eco2Value, tvocValue, dustDensityValue;
    static float temperatureValue, humidityValue, dustSensorBaseline;
    static bool b_ENVDataCorrection;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if(b_isrFlag){
        e_state = READ_DHT_SENSOR; //This is safer than setting the value inside the ISR itself
        b_isrFlag = false;
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch (e_state)
    {
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue);
        e_state = READ_CCS_SENSOR;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if(b_ENVDataCorrection){
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }else{
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = LCD_DISPLAY;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case LCD_DISPLAY:
        if(b_ENVDataCorrection){
        }else{
        }
        e_state = IDLE; //set the loop to idle until new interrupt triggers a change.
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case IDLE:
    default:
        delay(10); //Slow down a little bit
        break;
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Set the Read Flag ISR Callback
 * 
 */
void setReadFlagISRCallback(){
    b_isrFlag = true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 * 
 * @param eco2Value A reference where the eco2 value should be saved at
 * @param tvocValue A reference where the tvoc value should be saved at
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue){
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic Engine of co2Sensor
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    if(!co2Sensor.readData()){
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable logic engine of co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 * 
 * @param eco2Value A reference where the eco2 value should be saved at
 * @param tvocValue A reference where the eco2 value should be saved at
 * @param temperatureValue The current temperature to create a more accurate reading
 * @param humidityValue The current humidity to create a more accurate reading
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue, 
    float temperatureValue, float humidityValue){
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic Engine of co2Sensor
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    co2Sensor.setEnvironmentalData(humidityValue, temperatureValue);
    if(!co2Sensor.readData()){
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief Gets the Current Temperature and Humidity
 * 
 * @param temperatureValue A reference where the temperature value should be saved at
 * @param humidityValue A reference where the humidity value should be saved at
 * @return true if the reading was sucessful
 * @return false if the reading was unsucessful and the read value is 'Not A Number'
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue){
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue)) {
        return false;
    }
    return true;
} 
/*________________________________________________________________________________________________*/
/**
 * @brief Displays the latest readings from the sensors on an LCD Display
 * 
 * @param eco2Value 
 * @param tvocValue 
 * @param temperatureValue A reference to the temperature value to be displayed
 * @param humidityValue A reference to the humidity value to be displayed
 */
void printReadingsToLCD(uint16_t &eco2Value, uint16_t &tvocValue, 
                        float &temperatureValue, float &humidityValue){

                        }
/*end of file*/