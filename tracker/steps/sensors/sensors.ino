#include <CanSatKit.h>
#include <Wire.h>
#include <BME280I2C.h>

using namespace CanSatKit;

BME280I2C bme;

void setup() {
    SerialUSB.begin(9600);


    SerialUSB.println("start!");

    if (!bme.begin()) {
        SerialUSB.println("Could not find a valid BME280 sensor, check wiring!");
    }
}


void loop()
{
    float temperature, humidity, pressure;

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
    bme.read(pressure, temperature, humidity, tempUnit, presUnit);


    SerialUSB.print(temperature);
    SerialUSB.print(";");
    SerialUSB.print(humidity);
    SerialUSB.print(";");
    SerialUSB.println(pressure);

    delay(1000);
}