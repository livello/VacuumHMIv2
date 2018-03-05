#include "Arduino.h"
#include "HardwareSerial.h"
#include "ChRt.h"
#include <BME280I2C.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <DHT.h>
#include <OneWire.h>
#include <Dns.h>
#include <Time.h>
#include "Ethernet.h"
#include "personal_data.h"
#include "../.piolibdeps/ChRt_ID2986/src/rt/ch.h"

#ifndef PERSONAL_DATA_H
#define my_personal_mac_address {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}
#endif

#define SERIAL_BAUD 230400
#define RELAYS_NUM 6
#define DHT11_PIN 36
#define DS18B20_CLOCK_PIN 38
#define DS18B20_STEEL_PIN 34
#define SOIL_SENSOR_PIN1 A0
#define SOIL_SENSOR_PIN2 40
#define NTP_UDP_PORT 2390
#define TIME_ZONE 3
#define seventyYears 2208988800UL

DHT dht11Sensor(DHT11_PIN, DHT11);
OneWire ds(DS18B20_CLOCK_PIN);  // on pin 10 (a 4.7K resistor is necessary)

byte mac[] = my_personal_mac_address;
IPAddress ip(192, 168, 3, 177);
EthernetServer server(80);
tmElements_t tm_rtc, tm_ntp;
const int relayPins[] = {31, 33, 35, 37, 39, 41};

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
float bme280Temperature(NAN), bme280Humidity(NAN), bme280Pressure(NAN);
float dht11Temperature(NAN), dht11Humidity(NAN);
float ds18b20SteelTemperature(NAN), ds18b20ClockTemperature(NAN);
bool isReadingDS18B20 = false;


const char *ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
char packetBuffer2[NTP_PACKET_SIZE];
IPAddress ntpServerIP; // time.nist.gov NTP server address
EthernetUDP udp;
DNSClient my_dns;

void readTemperature();

void ds18b20Read(Stream *stream);

void chSetup();

void rtcPrint(Stream *stream);

void printBME280Data(Stream *stream);

void sendNTPpacket(IPAddress &address);

void getNtpTime();

//////////////////////////////////////////////////////////////////
void ethernet_setup() {
    Ethernet.begin(mac, 15000);
    server.begin();
    udp.begin(NTP_UDP_PORT);
    my_dns.begin(Ethernet.dnsServerIP());
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
        digitalWrite(relayPins[i], LOW);
    }
    dht11Sensor.begin();
    chBegin(chSetup);
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
                    client.println("Refresh: 15");
                    client.println();
                    client.println("<!DOCTYPE HTML>");
                    client.println("<html>");
                    // output the value of each analog input pin
                    rtcPrint(&client);
                    client.println("<br />");
                    printBME280Data(&client);
                    client.println("<br />");
                    ds18b20Read(&client);
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
        chThdSleepMilliseconds(10);
        // close the connection:
        client.stop();
        Serial.println("client disconnected");
    }
}

//////////////////////////////////////////////////////////////////
void printBME280Data(Stream *stream) {
    stream->print("Temp: ");
    stream->print(bme280Temperature);
    stream->print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
    stream->print("\t\tHumidity: ");
    stream->print(bme280Humidity);
    stream->print("% RH");
    stream->print("\t\tPressure: ");
    stream->print(bme280Pressure);
    stream->println(" Pa");
}

void readTemperature() {
    bme.read(bme280Pressure, bme280Temperature, bme280Humidity, tempUnit, presUnit);
    dht11Temperature = dht11Sensor.readTemperature();
    dht11Humidity = dht11Sensor.readHumidity();
    ds18b20Read(&Serial);
}

void ds18b20Read(Stream *stream) {
    if (isReadingDS18B20)
        return;
    isReadingDS18B20 = true;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    while (ds.search(addr)) {


        stream->print("ROM =");
        for (i = 0; i < 8; i++) {
            stream->write(' ');
            stream->print(addr[i], HEX);
        }

        if (OneWire::crc8(addr, 7) != addr[7]) {
            stream->println("CRC is not valid!");
            return;
        }

        // the first ROM byte indicates which chip
        switch (addr[0]) {
            case 0x10:
                stream->print("  Chip = DS18S20");  // or old DS1820
                type_s = 1;
                break;
            case 0x28:
                stream->print("  Chip = DS18B20");
                type_s = 0;
                break;
            case 0x22:
                stream->print("  Chip = DS1822");
                type_s = 0;
                break;
            default:
                stream->print("Device is not a DS18x20 family device.");
                return;
        }
        ds.reset();
        ds.select(addr);
        ds.write(0x44, 1);        // start conversion, with parasite power on at the end

        chThdSleepMilliseconds(1000);
        // we might do a ds.depower() here, but the reset will take care of it.

        present = ds.reset();
        ds.select(addr);
        ds.write(0xBE);         // Read Scratchpad

        stream->print("  Data = ");
        stream->print(present, HEX);
        stream->print(" ");
        for (i = 0; i < 9; i++) {           // we need 9 bytes
            data[i] = ds.read();
            stream->print(data[i], HEX);
            stream->print(" ");
        }
        stream->print(" CRC=");
        stream->print(OneWire::crc8(data, 8), HEX);

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
        ds18b20ClockTemperature = (float) raw / 16.0;
        stream->print("  DS18b20Temp = ");
        stream->print(ds18b20ClockTemperature);
        stream->print(" C, \n");
    }
    stream->println("No more addresses.");
    ds.reset_search();
    isReadingDS18B20 = false;
}
//////////////////////////////////////////////////////////////////

void print2digits(int number, Stream *stream) {
    if (number >= 0 && number < 10) {
        stream->write('0');
    }
    stream->print(number);
}

void rtcPrint(Stream *stream = &Serial) {
    if (RTC.read(tm_rtc)) {
        stream->print("Ok, Time = ");
        print2digits(tm_rtc.Hour, stream);
        stream->write(':');
        print2digits(tm_rtc.Minute, stream);
        stream->write(':');
        print2digits(tm_rtc.Second, stream);
        stream->print(", Date (D/M/Y) = ");
        stream->print(tm_rtc.Day);
        stream->write('/');
        stream->print(tm_rtc.Month);
        stream->write('/');
        stream->print(tmYearToCalendar(tm_rtc.Year));
        stream->println();
    } else {
        if (RTC.chipPresent()) {
            stream->println("The DS1307 is stopped.  Please run the SetTime");
            stream->println("example to initialize the time and begin running.");
            stream->println();
        } else {
            stream->println("DS1307 read error!  Please check the circuitry.");
            stream->println();
        }
    }
}

void loop() {
    getNtpTime();
    printBME280Data(&Serial);
    readTemperature();
    Serial.print("DHT11:");
    Serial.print(dht11Temperature);
    Serial.print(" Celsius, Humidity(%):");
    Serial.println(dht11Humidity);
    chThdSleepMilliseconds(10000);

}

SEMAPHORE_DECL(sem, 0);
static THD_WORKING_AREA(waThread1, 64);

static THD_FUNCTION(Thread1, arg) {
    (void) arg;
    while (!chThdShouldTerminateX()) {
//        chSemWait(&sem);
//        rtcPrint(&Serial);
        chThdSleepMilliseconds(1000);
    }
}

static THD_WORKING_AREA(waThread2, 64);

static THD_FUNCTION(Thread2, arg) {
    (void) arg;
    while (true) {
        ethernet_loop();
        chThdSleepMilliseconds(100);
    }
}

//------------------------------------------------------------------------------
// continue setup() after chBegin().
void chSetup() {
    // Start threads.
    chThdCreateStatic(waThread1, sizeof(waThread1),
                      NORMALPRIO + 2, Thread1, NULL);

    chThdCreateStatic(waThread2, sizeof(waThread2),
                      NORMALPRIO + 1, Thread2, NULL);
}

void getNtpTime() {

    int ret = my_dns.getHostByName(ntpServerName, ntpServerIP);
    if (ret == 1) {
        Serial.print("IP: ");
        Serial.print(ntpServerIP);
    } else {
        Serial.print("getHostByName Failed");
        Serial.print("ret = ");
        Serial.print(ret);
        return;
    }
    //get a random server from the pool
    sendNTPpacket(ntpServerIP); // send an NTP packet to a time server
    // wait to see if a reply is available
    time_t time_rtc = RTC.get();
    uint32_t send_time = millis();
    while (send_time + 1500 > millis() && !udp.parsePacket())
        chThdSleep(10);
    if (!udp.parsePacket()) {
        Serial.println("no answer was received");
        return;
    } else {
        udp.read(packetBuffer2, NTP_PACKET_SIZE); // read the packet into the buffer
        unsigned long highWord = word(packetBuffer2[40], packetBuffer2[41]);
        unsigned long lowWord = word(packetBuffer2[42], packetBuffer2[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;
        unsigned long epoch = secsSince1900 - seventyYears + TIME_ZONE * 3600;
        RTC.set(epoch);
        Serial.print("RTC seconds:");
        Serial.print(time_rtc);
        Serial.print("NTP seconds:");
        Serial.println(epoch);
        rtcPrint(&Serial);
    }
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
    Serial.println("sending NTP packet...");
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}