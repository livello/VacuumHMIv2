#include "Arduino.h"
#include "HardwareSerial.h"

/*
BME280 I2C Test.ino

This code shows how to record data from the BME280 environmental sensor
using I2C interface. This file is an example file, part of the Arduino
BME280 library.

GNU General Public License

Written: Dec 30 2015.
Last Updated: Oct 07 2017.

Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

 */

#include <BME280I2C.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <DHT.h>
#include <OneWire.h>
#include "Ethernet.h"
#include "personal_data.h"

#ifndef PERSONAL_DATA_H
#define my_personal_mac_address {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}
#endif

#define SERIAL_BAUD 115200
#define RELAYS_NUM 6
#define DHT11_PIN 36
#define DS18B20_CLOCK_PIN 38
#define DS18B20_STEEL_PIN 34
#define SOIL_SENSOR_PIN1 A0
#define SOIL_SENSOR_PIN2 40
DHT dht11Sensor(DHT11_PIN, DHT11);
OneWire ds(DS18B20_CLOCK_PIN);  // on pin 10 (a 4.7K resistor is necessary)

byte mac[] = my_personal_mac_address;
IPAddress ip(192, 168, 3, 177);
EthernetServer server(80);
tmElements_t tm;
const int relayPins[] = {31, 33, 35, 37, 39, 41};

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
float bme280Temperature(NAN), bme280Humidity(NAN), bme280Pressure(NAN);
float dht11Temperature(NAN), dht11Humidity(NAN);
float ds18b20SteelTemperature(NAN), ds18b20ClockTemperature(NAN);

void readTemperature();
void ds18b20Read(void);

//////////////////////////////////////////////////////////////////
void ethernet_setup() {
    Ethernet.begin(mac, 15000);
    server.begin();
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());
}

void setup() {
    Serial.println("DS1307RTC Read Test");
    Serial.println("-------------------");
    Serial.begin(SERIAL_BAUD);

    while (!Serial) {} // Wait

    Wire.begin();

    while (!bme.begin()) {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch (bme.chipModel()) {
        case BME280::ChipModel_BME280:
            Serial.println("Found BME280 sensor! Success.");
            break;
        case BME280::ChipModel_BMP280:
            Serial.println("Found BMP280 sensor! No Humidity available.");
            break;
        default:
            Serial.println("Found UNKNOWN sensor! Error!");
    }
    ethernet_setup();
    for (int i = 0; i < RELAYS_NUM; i++) {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], i % 2);
    }
    dht11Sensor.begin();
}

void ethernet_loop() {
    // listen for incoming clients
    EthernetClient client = server.available();
    if (client) {
        Serial.println("new client");
        // an http request ends with a blank line
        boolean currentLineIsBlank = true;
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                Serial.write(c);
                // if you've gotten to the end of the line (received a newline
                // character) and the line is blank, the http request has ended,
                // so you can send a reply
                if (c == '\n' && currentLineIsBlank) {
                    // send a standard http response header
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println(
                            "Connection: close");  // the connection will be closed after completion of the response
                    client.println("Refresh: 5");  // refresh the page automatically every 5 sec
                    client.println();
                    client.println("<!DOCTYPE HTML>");
                    client.println("<html>");
                    // output the value of each analog input pin
                    client.println("Temp: ");
                    client.println(bme280Temperature);
                    client.println("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
                    client.println("\t\tHumidity: ");
                    client.println(bme280Humidity);
                    client.println("% RH");
                    client.println("\t\tPressure: ");
                    client.println(bme280Pressure);
                    client.println(" Pa");
                    client.println("<br />");
                    if (RTC.read(tm)) {
                        Serial.print("Ok, Time = ");
                        client.print(tm.Hour);
                        client.print(':');
                        client.print(tm.Minute);
                        client.print(':');
                        client.print(tm.Second);
                        client.print(", Date (D/M/Y) = ");
                        client.print(tm.Day);
                        client.print('/');
                        client.print(tm.Month);
                        client.print('/');
                        client.print(tmYearToCalendar(tm.Year));
                        client.println("<br />");
                    } else {
                        if (RTC.chipPresent()) {
                            client.println("The DS1307 is stopped.  Please run the SetTime");
                            client.println("example to initialize the time and begin running.");
                        } else {
                            client.println("DS1307 read error!  Please check the circuitry.");
                        }
                        client.println("<br />");
                    }
                    client.println("</html>");
                    break;
                }
                if (c == '\n') {
                    // you're starting a new line
                    currentLineIsBlank = true;
                } else if (c != '\r') {
                    // you've gotten a character on the current line
                    currentLineIsBlank = false;
                }
            }
        }
        // give the web browser time to receive the data
        delay(1);
        // close the connection:
        client.stop();
        Serial.println("client disconnected");
    }
}

//////////////////////////////////////////////////////////////////
void printBME280Data(Stream *client) {
    readTemperature();
    client->print("Temp: ");
    client->print(bme280Temperature);
    client->print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
    client->print("\t\tHumidity: ");
    client->print(bme280Humidity);
    client->print("% RH");
    client->print("\t\tPressure: ");
    client->print(bme280Pressure);
    client->println(" Pa");
    delay(1000);
}

void readTemperature() {
    bme.read(bme280Pressure, bme280Temperature, bme280Humidity, tempUnit, presUnit);
    dht11Temperature = dht11Sensor.readTemperature();
    dht11Humidity = dht11Sensor.readHumidity();
    ds18b20Read();
}
void ds18b20Read(void) {
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];


    if ( !ds.search(addr)) {
        Serial.println("No more addresses.");
        Serial.println();
        ds.reset_search();
        return;
    }

    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0]) {
        case 0x10:
            Serial.println("  Chip = DS18S20");  // or old DS1820
            type_s = 1;
            break;
        case 0x28:
            Serial.println("  Chip = DS18B20");
            type_s = 0;
            break;
        case 0x22:
            Serial.println("  Chip = DS1822");
            type_s = 0;
            break;
        default:
            Serial.println("Device is not a DS18x20 family device.");
            return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    ds18b20ClockTemperature = (float)raw / 16.0;
    Serial.print("  Temperature = ");
    Serial.print(ds18b20ClockTemperature);
    Serial.print(" Celsius, ");
}
//////////////////////////////////////////////////////////////////


const int RELAY[6] = {8, 9, 10, 11, 12, 13};

void setupRelay() {

    for (int i = 0; i < 6; i++) {

        pinMode(RELAY[i], OUTPUT);

        digitalWrite(RELAY[i], HIGH);

    }
    Serial.begin(115200);
}

void print2digits(int number) {
    if (number >= 0 && number < 10) {
        Serial.write('0');
    }
    Serial.print(number);
}

void rtc_loop() {
    if (RTC.read(tm)) {
        Serial.print("Ok, Time = ");
        print2digits(tm.Hour);
        Serial.write(':');
        print2digits(tm.Minute);
        Serial.write(':');
        print2digits(tm.Second);
        Serial.print(", Date (D/M/Y) = ");
        Serial.print(tm.Day);
        Serial.write('/');
        Serial.print(tm.Month);
        Serial.write('/');
        Serial.print(tmYearToCalendar(tm.Year));
        Serial.println();
    } else {
        if (RTC.chipPresent()) {
            Serial.println("The DS1307 is stopped.  Please run the SetTime");
            Serial.println("example to initialize the time and begin running.");
            Serial.println();
        } else {
            Serial.println("DS1307 read error!  Please check the circuitry.");
            Serial.println();
        }
        delay(9000);
    }
    delay(1000);
}

void loop() {
    printBME280Data(&Serial);
    rtc_loop();
    ethernet_loop();
    readTemperature();
    delay(5000);
}
