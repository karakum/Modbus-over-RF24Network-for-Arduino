#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <ModbusRtu.h>
#include <ModbusRtuRF24.h>

#define ID   1      // адрес ведомого
#define btnPin  2   // номер входа, подключенный к кнопке
#define ledPin  7  // номер выхода светодиода

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(9, 10);

// Network uses that radio
RF24Network network(radio);

// Address of our node
const uint16_t this_node = ID;

//Задаём ведомому адрес
ModbusRF24 slave(network, ID);

// массив данных modbus
uint16_t au16data[11];

void io_setup() {
    digitalWrite(ledPin, LOW);
    pinMode(ledPin, OUTPUT);
    pinMode(btnPin, INPUT);
}

void io_poll() {
    //Копируем Coil[1] в Discrete[0]
    au16data[0] = au16data[1];
    //Выводим значение регистра 1.3 на светодиод 
    digitalWrite(ledPin, bitRead(au16data[1], 3));
    //Сохраняем состояние кнопки в регистр 0.3
    bitWrite(au16data[0], 3, digitalRead(btnPin));
    //Копируем Holding[5,6,7] в Input[2,3,4]
    au16data[2] = au16data[5];
    au16data[3] = au16data[6];
    au16data[4] = au16data[7];
    //Сохраняем в регистры отладочную информацию
    au16data[8] = slave.getInCnt();
    au16data[9] = slave.getOutCnt();
    au16data[10] = slave.getErrCnt();
}

void setup() {
    // настраиваем входы и выходы
    io_setup();

    Serial.begin(57600);
    Serial.println("RF24Network/examples/modbus_slave/");

    SPI.begin();
    radio.begin();
    network.begin(/*channel*/ 90, /*node address*/ this_node);
}

void loop() {

    // Pump the network regularly
    network.update();

    if (network.available()) {
        slave.poll(au16data, 11);
    }
    //обновляем данные в регистрах Modbus и в пользовательской программе
    io_poll();
}

