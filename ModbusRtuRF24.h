/* 
 * File:   ModbusRtuRF24.h
 * Author: Andrey Shertsinger <andrey@shertsinger.ru>
 *
 * Created on August 4, 2015, 10:33 PM
 */

#ifndef MODBUSRTURF24_H
#define MODBUSRTURF24_H

#include <ModbusRtu.h>
#include <RF24Network.h>
#include <RF24.h>


#ifdef DEBUG
#define PRINTHEX(d,l)  printHex(d,l)
#if defined(UBRR1H)
#define PRINT(...)     Serial1.print(__VA_ARGS__)
#define PRINTLN(...)   Serial1.println(__VA_ARGS__)
#else
#define PRINT(...)     Serial.print(__VA_ARGS__)
#define PRINTLN(...)   Serial.println(__VA_ARGS__)
#endif
#else
#define PRINTHEX(d,l)
#define PRINT(...)
#define PRINTLN(...)
#endif

/**
 * @class ModbusRF24 
 * @brief
 * Arduino class library for communicating with Modbus devices over RF24Network
 */
class ModbusRF24 : public Modbus {
protected:
	RF24Network& network;

	void sendTxBuffer();
	int8_t getRxBuffer();
	int getRxBufferAvailable();

#ifdef DEBUG
    void printHex(unsigned char* data, int len);
#endif

public:
	// Slave over RF mode
	ModbusRF24(RF24Network& _network, uint8_t _u8id);

	// Proxy mode
	ModbusRF24(RF24Network& _network, uint8_t u8serno, uint8_t u8txenpin);

	int8_t proxy(); //cyclic poll for slave

private:
	uint16_t masterNode;
	char ser;

	void shiftData();
	void unShiftData();
	uint8_t validateProxyRequest(uint8_t len, uint8_t offset = 0);

};

#endif  /* MODBUSRTURF24_H */
