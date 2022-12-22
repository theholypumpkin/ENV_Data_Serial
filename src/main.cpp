/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include <LiquidCrystal.h>
/*================================================================================================*/
#define CCS_811_INTERRUPT_PIN 2
#define CCS_811_nWAKE 13
#define DHTPIN 17
#define DHTTYPE DHT11
#define LCD_RS_PIN 16
#define LCD_EN_PIN 4
#define LCD_D0_PIN 5
#define LCD_D1_PIN 6
#define LCD_D2_PIN 7
#define LCD_D3_PIN 8
#define LCD_D4_PIN 9
#define LCD_D5_PIN 10
#define LCD_D6_PIN 15
#define LCD_D7_PIN 12
#define LCD_BACKLIGHT_PWM 11
#define RED_LED_PIN 14
#define YELLOW_LED_PIN 3
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    LCD_DISPLAY_CCS,
    LCD_DISPLAY_TEMPERATURE,
    LCD_DISPLAY_HUMIDITY,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
volatile bool b_isrFlag = false; //a flag which is flipped inside an isr
uint8_t 
szLetter[8] = {0b01110, 0b10001, 0b10001, 0b10110, 0b10001, 0b10001, 0b10110, 0b10000},
ueLetter[8] = {0b10001, 0b00000, 0b10001, 0b10001, 0b10001, 0b10011, 0b01101, 0b00000},
lowerTwo[8] = {0b00000, 0b00000, 0b00000, 0b01100, 0b10010, 0b00100, 0b01000, 0b11110},
degreeSymbol[8] = {0b01100, 0b10010, 0b10010, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000};
/*________________________________________________________________________________________________*/
DHT tempHmdSensor(DHTPIN, DHTTYPE); //Create the DHT object
Adafruit_CCS811 co2Sensor;
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, 
                  LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN, 
                  LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
/*================================================================================================*/
void setup() {
    pinMode(CCS_811_nWAKE, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, HIGH);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    digitalWrite(CCS_811_nWAKE, LOW); //Enable logic engine of CCS811
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if(!co2Sensor.begin()){
        while(1){
            bool state = digitalRead(YELLOW_LED_PIN);
            digitalWrite(YELLOW_LED_PIN, !state);
            digitalWrite(RED_LED_PIN, state);
            delay(500); //When failure blink LED rapidly
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable logic engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    Begin the LCD*/
    lcd.begin(16,2);
    lcd.write("Waiting 4 data!");
    lcd.createChar(0, szLetter);
    lcd.createChar(1, ueLetter);
    lcd.createChar(2, lowerTwo);
    lcd.createChar(3, degreeSymbol);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    LCD Brigthness PWM*/
    analogWrite(LCD_BACKLIGHT_PWM, 63);

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
    static uint16_t eco2Value, tvocValue;
    static float temperatureValue, heatIndexValue, humidityValue;
    static bool b_ENVDataCorrection;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    digitalWrite(CCS_811_nWAKE, LOW); //Enable logic engine of CCS811
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    if(co2Sensor.available()){
        e_state = READ_DHT_SENSOR; //This is safer than setting the value inside the ISR itself
    }
    delayMicroseconds(25); // Time until active after nWAKE asserted = 50 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Enable logic engine of CCS811
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch (e_state)
    {
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue, heatIndexValue);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if(b_ENVDataCorrection){
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }else{
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = LCD_DISPLAY_CCS;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case LCD_DISPLAY_CCS:
        if(b_ENVDataCorrection){
            lcd.clear();
            lcd.print("CO"); lcd.write(uint8_t(2)); lcd.print(": ");
            lcd.print(eco2Value); lcd.print(" ppm");
            lcd.setCursor(0,1);
            lcd.print("TVOC: ");
            lcd.print(tvocValue); lcd.print(" ppb");
            e_state = LCD_DISPLAY_TEMPERATURE;
        }else{
            e_state = IDLE;
        }
        if(tvocValue > 250){
            digitalWrite(YELLOW_LED_PIN, HIGH);
            digitalWrite(RED_LED_PIN, LOW);
        }
        else if (tvocValue > 100){
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, LOW);
        }
        else if (tvocValue > 50){
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, HIGH);
        }
        else{
            digitalWrite(RED_LED_PIN, HIGH);
            digitalWrite(YELLOW_LED_PIN, HIGH);
        }
        delay(10000);
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case LCD_DISPLAY_TEMPERATURE:
        lcd.clear();
        lcd.print("Geme"); lcd.write(uint8_t(0)); lcd.print("en: ");
        lcd.print(temperatureValue); lcd.write(uint8_t(3)); lcd.print("C");
        lcd.setCursor(0,1);
        lcd.print("Gef"); lcd.write(uint8_t(1)); lcd.print("hlt: ");
        lcd.print(heatIndexValue); lcd.write(uint8_t(3)); lcd.print("C");
        e_state = LCD_DISPLAY_HUMIDITY;
        delay(10000);
        break;
     /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case LCD_DISPLAY_HUMIDITY:
        lcd.clear();
        lcd.print("Luftfeuchtigkeit");
        lcd.setCursor(0,1);
        lcd.print(humidityValue);
        lcd.print(" %");
        e_state = LCD_DISPLAY_CCS;
        delay(10000);
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case IDLE:
    default:
        delay(100); //Slow down a little bit
        break;
    }
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
 * @param heatIndexValue A reference where the heat index value should be saved at
 * @return true if the reading was sucessful
 * @return false if the reading was unsucessful and the read value is 'Not A Number'
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue, float &heatIndexValue){
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue)) {
        return false;
    }
    heatIndexValue = tempHmdSensor.computeHeatIndex(false);
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