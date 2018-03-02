
//#define DHTPIN 8    // пин для датчика DHT22
//#define DHTTYPE DHT11   // DHT 11
#define ONEWIRE_PIN 7
//

#include "Arduino.h"
#include <HardwareSerial.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <IPAddress.h>
#include <EthernetServer.h>
#include "OneWire.h"


const byte default_ip[] = { 10, 20, 191, 200 };
const byte default_gateway[] = { 10, 20, 191, 169 };
const byte default_dns1[] = { 185, 108, 192, 4 };
byte mac_address[] = {0xCA, 0xAF, 0x21, 0x9C, 0x11, 0x29}; //MAC-адрес Arduino
EthernetClient narodmonClient;

float temp_DS18B20;
float humidity_DHT22 = 0, temp_DHT22 = 0;
OneWire dallas18b20(ONEWIRE_PIN);

const unsigned long postingInterval = 330000;  // интервал между отправками данных в миллисекундах (10 минут)
uint32_t nextConnectionTime = 20000;           // время последней передачи данных
char replyBuffer[160];
const char *narodmon_host = "narodmon.ru";
const int narodmon_port = 8283;
char numberConversionBuffer[6];

void ftos(float float_number);

void sendNarodmonData();

void narodmonLoop();


void narodmonLoop() {
    if ((millis() > nextConnectionTime)) {
        char temp[3];
        //формирование HTTP-запроса
        memset(replyBuffer, 0, sizeof(replyBuffer));
        strcpy(replyBuffer, "#");

        //Конвертируем MAC-адрес
        for (int k = 0; k < 6; k++) {
            int b1 = mac_address[k] / 16;
            int b2 = mac_address[k] % 16;
            char c1[2], c2[2];

            if (b1 > 9) c1[0] = (char) (b1 - 10) + 'A';
            else c1[0] = (char) (b1) + '0';
            if (b2 > 9) c2[0] = (char) (b2 - 10) + 'A';
            else c2[0] = (char) (b2) + '0';

            c1[1] = '\0';
            c2[1] = '\0';

            strcat(replyBuffer, c1);
            strcat(replyBuffer, c2);
        }
        strcat(replyBuffer, "\n#T1#");
        ftos(temp_DHT22);
        strcat(replyBuffer, numberConversionBuffer);
        strcat(replyBuffer, "\n#H1#");
        ftos(humidity_DHT22);
        strcat(replyBuffer, numberConversionBuffer);
        strcat(replyBuffer, "\n#T2#");
        ftos(temp_DS18B20);
        strcat(replyBuffer, numberConversionBuffer);
        strcat(replyBuffer, "\n#LAT#56.14031\n#LNG#47.19248\n##\0");
        Serial.print("Prepared replyBuffer:");
        Serial.println(replyBuffer);
        sendNarodmonData();
        //lat=56.14031&lon=47.19248
    }

}

void sendNarodmonData() {
    if (narodmonClient.connect(narodmon_host, narodmon_port)) {
        narodmonClient.println(replyBuffer);
        narodmonClient.stop();
        nextConnectionTime = millis() + postingInterval;
        Serial.print(millis());
        Serial.println("Connection to NarodMon successfull!");
    } else
        Serial.println("Can not connect to NarodMon :((");
}

void ftos(float float_number) //int to string
{
    numberConversionBuffer[5] = '\0';
    int number = float_number * 10;
    int digits_processed = 0;
    int char_position = 4;
    if (number < 0) {
        number = -number;
        numberConversionBuffer[0] = '-';
    } else
        numberConversionBuffer[0] = '0';

    numberConversionBuffer[char_position] = (number % 10) + 48;
    char_position--;
    numberConversionBuffer[char_position] = '.';
    char_position--;
    while (digits_processed < 2) {
        number /= 10;
        numberConversionBuffer[char_position] = (number % 10) + 48;
        char_position--;
        digits_processed++;
    }
    while (char_position > 0) {
        numberConversionBuffer[char_position] = '0';
        char_position--;
    }



}
void show_ds18b20() {
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];


    if (!dallas18b20.search(addr)) {
        Serial.println("No more addresses.");
        Serial.println();
        dallas18b20.reset_search();
        delay(250);
        return;
    }

    Serial.print("ROM =");
    for (i = 0; i < 8; i++) {
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

    dallas18b20.reset();
    dallas18b20.select(addr);
    dallas18b20.write(0x44, 1);        // start conversion, with parasite power on at the end

    present = dallas18b20.reset();
    dallas18b20.select(addr);
    dallas18b20.write(0xBE);         // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = dallas18b20.read();
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
    temp_DS18B20 = (float) raw / 16.0;
    Serial.print("  Temperature ds18b20= ");
    Serial.print(temp_DS18B20);
    Serial.println(" Cels");

}
void setup() {
    Serial.begin(115200);
    delay(10);
    if(!Ethernet.begin(mac_address)){
        Serial.println("Can not assign DHCP address!!!!");
        Ethernet.begin(mac_address,default_ip,default_dns1,default_gateway);
    }

    Serial.print("Local IP is ");
    delay(2000);
    Serial.println(Ethernet.localIP());
    nextConnectionTime = millis() - postingInterval + 15000; //первое соединение через 15 секунд после запуска
}

void loop(){
    show_ds18b20();
//    narodmonLoop();
    delay(15000);
}