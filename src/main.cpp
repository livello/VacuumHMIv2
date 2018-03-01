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
#include "SPI.h"
#include <TimeLib.h>
#include <DS1307RTC.h>
#include "Ethernet.h"

#define SERIAL_BAUD 115200
byte mac[] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 3, 177);
EthernetServer server(80);
tmElements_t tm;


BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
float temp(NAN), hum(NAN), pres(NAN);

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
                    client.println(temp);
                    client.println("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
                    client.println("\t\tHumidity: ");
                    client.println(hum);
                    client.println("% RH");
                    client.println("\t\tPressure: ");
                    client.println(pres);
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
void printBME280Data
        (
                Stream *client
        ) {


    bme.read(pres, temp, hum, tempUnit, presUnit);

    client->print("Temp: ");
    client->print(temp);
    client->print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
    client->print("\t\tHumidity: ");
    client->print(hum);
    client->print("% RH");
    client->print("\t\tPressure: ");
    client->print(pres);
    client->println(" Pa");

    delay(1000);
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
    delay(500);
}
