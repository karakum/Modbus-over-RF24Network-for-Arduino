#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <ModbusRtu.h>
#include <ModbusRtuRF24.h>

#define stlPin  13  // номер выхода индикатора работы (расположен на плате Arduino)

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(9, 10);

// Network uses that radio
RF24Network network(radio);

// Address of our node
const uint16_t this_node = 0;

//Задаём ведомому адрес, последовательный порт, выход управления TX
ModbusRF24 proxy(network, 0, 0);
int8_t state = 0;
unsigned long tempus;

void setup() {
    // настраиваем входы и выходы
    io_setup();
    // настраиваем последовательный порт ведомого

    proxy.begin(57600);

    SPI.begin();
    radio.begin();
    network.begin(/*channel*/ 90, /*node address*/ this_node);

    // зажигаем светодиод на 100 мс
    tempus = millis() + 100;
    digitalWrite(stlPin, HIGH);
}

void io_setup() {
    digitalWrite(stlPin, HIGH);
    pinMode(stlPin, OUTPUT);
}

void loop() {

    // Pump the network regularly
    network.update();

    // обработка сообщений
    state = proxy.proxy();

    // если получили пакет без ошибок - зажигаем светодиод на 10 мс 
    if (state > 4) {
        tempus = millis() + 10;
        digitalWrite(stlPin, HIGH);
    }
    if (millis() > tempus) digitalWrite(stlPin, LOW);
}
