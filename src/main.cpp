#include "Arduino.h"
#include "HardwareSerial.h"
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

#ifndef PERSONAL_DATA_H
#define my_personal_mac_address {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}
#endif

#define SERIAL_BAUD 230400
#define RELAYS_NUM 6
#define DHT11_PIN 46
#define DS18B20_CLOCK_PIN 49
#define DS18B20_STEEL_PIN 34
#define SOIL_SENSOR_PIN_ANALOG A0
#define SOIL_SENSOR_PIN_DIGITAL 48
#define NTP_UDP_PORT 2390
#define TIME_ZONE 3
#define seventyYears 2208988800UL
#define NTP_PACKET_SIZE 48

#define NTP_SYNC_PERIOD 300000
#define NTP_RETRY_ON_ERROR_PERIOD 5000
#define TEMP_UPDATE_PERIOD 30000
DHT dht11Sensor(DHT11_PIN, DHT11);
OneWire ds(DS18B20_CLOCK_PIN);  // on pin 10 (a 4.7K resistor is necessary)

byte mac[] = my_personal_mac_address;
IPAddress ip(192, 168, 3, 177);
EthernetServer server(80);
tmElements_t tm_rtc;
const int relayPins[] = {31, 33, 35, 37, 39, 41};

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
float bme280Temperature(NAN), bme280Humidity(NAN), bme280Pressure(NAN);
float dht11Temperature(NAN), dht11Humidity(NAN);
float ds18b20SteelTemperature(NAN), ds18b20ClockTemperature(NAN);

bool nothing;

const char *ntpServerName = "time.nist.gov";

byte packetBuffer[NTP_PACKET_SIZE] = {0b11100011, 0, 6, 0xEC, 0, 0, 0, 0, 0, 0, 0, 0, 49, 0x4E, 49, 52, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0}; //buffer to hold incoming and outgoing packets
char packetBuffer2[NTP_PACKET_SIZE];
EthernetUDP udp;
DNSClient my_dns;

void readTemperature();

void ds18b20Read(Stream *stream);

void chSetup();

void rtcPrint(Stream *stream);

void printBME280Data(Stream *stream);

void sendNTPpacket(IPAddress &address);

bool getNtpTime();

void printDHT11(Stream *stream);

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
        if (i == 1 || i == 4)
            digitalWrite(relayPins[i], LOW);
        else
            digitalWrite(relayPins[i], HIGH);
    }
    dht11Sensor.begin();
    pinMode(SOIL_SENSOR_PIN_DIGITAL,INPUT_PULLUP);

}
    float soilHumidityUnits=0, soilHumidityRaw=0,soilHumidityVoltage=0;
    bool isSoilHumidityLevel = 0;
void readSoilSensor(Stream *stream){
    soilHumidityRaw = analogRead(SOIL_SENSOR_PIN_ANALOG);
    soilHumidityVoltage=soilHumidityRaw/1024.0*5.0;
    soilHumidityUnits = map(soilHumidityRaw,550,10,0,100);
    isSoilHumidityLevel = digitalRead(SOIL_SENSOR_PIN_DIGITAL);
    stream->print("    SoilHumidity:");
    stream->print(soilHumidityRaw);
    stream->print(" raw, ");
    stream->print(soilHumidityVoltage);
    stream->print(" Volts, ");
    stream->print(soilHumidityUnits);
    stream->print(" units, LEVEL:");
    stream->print(isSoilHumidityLevel);
    stream->print("<br />\n");
}

int pinState[] = {0, 0, 0, 0, 0, 0, 0, 0};  // Состояние пинов
void updateRelays() {
    for (int i = 0; i < RELAYS_NUM; i++)
        digitalWrite(relayPins[i], !pinState[i]);
}

void sendRelayControlForm(Stream *stream) {
    stream->println("<form method='post'> \n");
    for (int i = 0; i < RELAYS_NUM; i++) {
        stream->print("<div>Relay ");
        stream->print(i);
        stream->print(" <input type='checkbox' ");
        (pinState[i] == 1) ? stream->print("checked") : nothing;
        stream->print(" name='r");
        stream->print(i);
        stream->print("'></div>\n");
    }
    stream->print("<input type='submit' value='Refresh'>\n");
    stream->print("</form>\n");
}

void sendMainPage(EthernetClient &client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println(
            "Connection: close");  // the connection will be closed after completion of the response
    client.println();
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<meta http-equiv=\"refresh\" content=\"60\">");
    sendRelayControlForm(&client);
    rtcPrint(&client);
    client.println("<br />");
    printBME280Data(&client);
    client.println("<br />");
    printDHT11(&client);
    client.println("<br />");
    ds18b20Read(&client);
    readSoilSensor(&client);
    client.println("</html>");
}

void ethernet_loop() {
    EthernetClient client = server.available();
    if (client) {
        String webRequestType = client.readStringUntil('/');
        if (webRequestType.compareTo("GET ") == 0) {
            boolean currentLineIsBlank = true;
            while (client.connected()) {
                if (client.available()) {
                    char c = client.read();
                    if (c == '\n' && currentLineIsBlank) {
                        // send a standard http response header
                        sendMainPage(client);
                        break;
                    }
                    if (c == '\n') {
                        currentLineIsBlank = true;
                    } else if (c != '\r') {
                        currentLineIsBlank = false;
                    }
                }
            }
        } else if (webRequestType.compareTo("POST ") == 0) {
            String clientRequest = client.readString();
            Serial.println(clientRequest);
            char *rOnSequence = "r0=on";
            for (int i = 0; i < RELAYS_NUM; i++) {
                rOnSequence[1] = '0' + i;
                (clientRequest.indexOf(rOnSequence) > 0) ? pinState[i] = 1 : pinState[i] = 0;
            }
            updateRelays();
            sendMainPage(client);
        } else {
            Serial.println("." + webRequestType + ". - uknown webRequestType");
            Serial.println("" + client.readString());
            sendMainPage(client);
        }

        client.stop();
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

bool isReadingDS18B20 = false;

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
        delay(800);
        // we might do a ds.depower() here, but the reset will take care of it.

        present = ds.reset();
        ds.select(addr);
        ds.write(0xBE);         // Read Scratchpad

//        stream->print("  Data = ");
//        stream->print(present, HEX);
//        stream->print(" ");
        for (i = 0; i < 9; i++) {           // we need 9 bytes
            data[i] = ds.read();
//            stream->print(data[i], HEX);
//            stream->print(" ");
        }
//        stream->print(" CRC=");
//        stream->print(OneWire::crc8(data, 8), HEX);

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
        stream->print("  T: ");
        stream->print(ds18b20ClockTemperature);
        stream->print(" C, \n<br />\n");
    }
    stream->println("No more addresses.");
    ds.reset_search();
    isReadingDS18B20 = false;
}


bool isRTC_using = false;

void rtcPrint(Stream *stream) {
    if (isRTC_using)
        return;
    isRTC_using = true;
    tmElements_t tm_rtc_current;
    if (RTC.read(tm_rtc_current)) {
        stream->print("Ok, Time = ");
        (tm_rtc_current.Hour >= 0 && tm_rtc_current.Hour < 10) ? stream->write('0') : nothing;
        stream->print(tm_rtc_current.Hour);
        stream->write(':');
        (tm_rtc_current.Minute >= 0 && tm_rtc_current.Minute < 10) ? stream->write('0') : nothing;
        stream->print(tm_rtc_current.Minute);
        stream->write(':');
        (tm_rtc_current.Second >= 0 && tm_rtc_current.Second < 10) ? stream->write('0') : nothing;
        stream->print(tm_rtc_current.Second);
        stream->print(", Date (D/M/Y) = ");
        stream->print(tm_rtc_current.Day);
        stream->write('/');
        stream->print(tm_rtc_current.Month);
        stream->write('/');
        stream->print(tmYearToCalendar(tm_rtc_current.Year));
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
    isRTC_using = false;
}

void receiveUdpNtpPacket() {
    if (udp.parsePacket()) {
        Serial.println("received UDP packet");
        udp.read(packetBuffer2, NTP_PACKET_SIZE); // read the packet into the buffer
        unsigned long highWord = word(packetBuffer2[40], packetBuffer2[41]);
        unsigned long lowWord = word(packetBuffer2[42], packetBuffer2[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;
        unsigned long epoch = secsSince1900 - seventyYears + TIME_ZONE * 3600;
        if (epoch > RTC.get() + 5 || epoch + 5 < RTC.get()) {
            tmElements_t tm_ntp;
            breakTime(epoch, tm_ntp);
            RTC.write(tm_ntp);
            Serial.print("Time adjusted!!!");
        }
    }
}


unsigned long nextNtpSynchroTime = 5000;
unsigned long nextTempUpdateTime = 10000;
unsigned long nextPrintTime = 15000;

void loop() {
    ethernet_loop();
    receiveUdpNtpPacket();
    if (nextNtpSynchroTime < millis()) {
        if (getNtpTime())
            nextNtpSynchroTime = millis() + NTP_SYNC_PERIOD;
        else
            nextNtpSynchroTime = millis() + NTP_RETRY_ON_ERROR_PERIOD;
    }
    if (nextTempUpdateTime < millis()) {
        nextTempUpdateTime = millis() + TEMP_UPDATE_PERIOD;
        readTemperature();
    }
    if (nextPrintTime < millis()) {
        nextPrintTime = millis() + TEMP_UPDATE_PERIOD;
        printBME280Data(&Serial);
        printDHT11(&Serial);
        rtcPrint(&Serial);
        readSoilSensor(&Serial);

    }
}

void printDHT11(Stream *stream) {
    stream->print("DHT11:");
    stream->print(dht11Temperature);
    stream->print(" Celsius, Humidity(%):");
    stream->println(dht11Humidity);
}


bool isNtpTimeUsing = false;

bool getNtpTime() {
    bool result = false;
    if (isNtpTimeUsing)
        return result;
    isNtpTimeUsing = true;
    IPAddress ntpServerIP;
    int dnsResolveReturnCode = my_dns.getHostByName(ntpServerName, ntpServerIP);
    if (dnsResolveReturnCode == 1) {
        Serial.print("IP: ");
        Serial.print(ntpServerIP);
        sendNTPpacket(ntpServerIP); // send an NTP packet to a time server
        result = true;

    } else {
        Serial.print("getHostByName Failed");
        Serial.print("ret = ");
        Serial.print(dnsResolveReturnCode);
    }
    isNtpTimeUsing = false;
    return result;
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
    Serial.println("sending NTP packet...");
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}