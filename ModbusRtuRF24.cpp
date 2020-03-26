/* 
 * File:   ModbusRtuRF24.cpp
 * Author: Andrey Shertsinger <andrey@shertsinger.ru>
 *
 * Created on August 4, 2015, 10:33 PM
 */


#include "ModbusRtuRF24.h"

ModbusRF24::ModbusRF24(RF24Network& _network, uint8_t _u8id) : Modbus(_u8id, 0, 0), network(_network) {
	ser = 0;
}

ModbusRF24::ModbusRF24(RF24Network& _network, uint8_t _u8serno, uint8_t _u8txenpin) : Modbus(0, _u8serno, _u8txenpin), network(_network) {
	ser = 'A';
	setTimeOut(3000);
}

#ifdef DEBUG
void printHex(unsigned char* data, int len) {
	for (int i = 0; i < len; i++) {
		Serial.print(data[i], HEX);
		Serial.print(" ");
	}
}
#if defined(UBRR1H)
void printHex1(unsigned char* data, int len) {
	for (int i = 0; i < len; i++) {
		Serial1.print(data[i], HEX);
		Serial1.print(" ");
	}
}
#endif
#endif

void ModbusRF24::shiftData() {
	// move data to 1 position. First byte - data length.
	// Because RF24 frame has minimum frame size more than modbus.
	for (int i = u8BufferSize - 1; i >= 0; i--) {
		au8Buffer[i + 1] = au8Buffer[i];
	}
	au8Buffer[0] = u8BufferSize;

}

void ModbusRF24::unShiftData() {
	u8BufferSize = au8Buffer[0];
	for (int i = 0; i < u8BufferSize; i++) {
		au8Buffer[i] = au8Buffer[i + 1];
	}

}

int ModbusRF24::getRxBufferAvailable() {
	network.update();
	return network.available() ? 1 : 0;
}

/**
 *
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 * @ingroup buffer
 */
int8_t ModbusRF24::getRxBuffer() {
	boolean bBuffOverflow = false;

	u8BufferSize = 0;
	RF24NetworkHeader headerRequest;
	if (network.available()) {
		network.peek(headerRequest);
		if (headerRequest.type != ser) {
			ser = headerRequest.type;
			masterNode = headerRequest.from_node;
			if (network.read(headerRequest, au8Buffer, MAX_BUFFER) > 0) {
				unShiftData();
#ifdef DEBUG
				printHex(au8Buffer, u8BufferSize);
				Serial.print(" -> ");
#endif
				if (u8BufferSize >= MAX_BUFFER) bBuffOverflow = true;
			}
		} else {
#ifdef DEBUG
			Serial.println("repeat");
#endif
			network.read(headerRequest, NULL, 0);
		}
	}
	u16InCnt++;

	if (bBuffOverflow) {
		u16errCnt++;
		return ERR_BUFF_OVERFLOW;
	}
	return u8BufferSize;
}

void ModbusRF24::sendTxBuffer() {
	// append CRC to message
	uint16_t u16crc = calcCRC(u8BufferSize);
	au8Buffer[ u8BufferSize ] = u16crc >> 8;
	u8BufferSize++;
	au8Buffer[ u8BufferSize ] = u16crc & 0x00ff;
	u8BufferSize++;

#ifdef DEBUG
	printHex(au8Buffer, u8BufferSize);
	Serial.println();
#endif

	shiftData();

	RF24NetworkHeader headerAnswer(masterNode, ser);
	if (!network.write(headerAnswer, au8Buffer, u8BufferSize + 1)) {
#ifdef DEBUG
		Serial.println("fail");
#endif
	}
	u8BufferSize = 0;

	// set time-out for master
	u32timeOut = millis() + (unsigned long) u16timeOut;

	// increase message counter
	u16OutCnt++;
}

int8_t ModbusRF24::proxy() {

	// check if there is any incoming frame
	uint8_t u8current = Modbus::getRxBufferAvailable();
	if (u8current == 0) return 0;

	// check T35 after frame end or still no frame end
	if (u8current != u8lastRec) {
		u8lastRec = u8current;
		u32time = millis() + T35;
		return 0;
	}
	if (millis() < u32time) return 0;

	u8lastRec = 0;
	int8_t i8state = Modbus::getRxBuffer();
	u8lastError = i8state;
	if (i8state < 7) return i8state;
#ifdef DEBUG
#if defined(UBRR1H)
	printHex1(au8Buffer, u8BufferSize);
	Serial1.print(" -> ");
#endif
#endif
	u8id = au8Buffer[ ID ];

	// validate message: CRC, FCT, address and size
	uint8_t u8exception = validateProxyRequest(i8state);
	if (u8exception > 0) {
		if (u8exception != NO_REPLY) {
			buildException(u8exception);
			sendTxBuffer();
		}
		u8lastError = u8exception;
#ifdef DEBUG
#if defined(UBRR1H)
		Serial1.println("not valid");
#endif
#endif
		return u8exception;
	}

	u32timeOut = millis() + long(u16timeOut);
	u8lastError = 0;

	RF24NetworkHeader headerRequest(u8id, ser);

	shiftData();
	if (network.write(headerRequest, au8Buffer, u8BufferSize + 1)) {
		ser++;
		if (ser > 'Z') ser = 'A';

#ifdef DEBUG
#if defined(UBRR1H)
		Serial1.print((char) headerRequest.type);
		Serial1.print(":");
#endif
#endif

		while (!network.available()) {
			network.update();

			if (millis() > u32timeOut) {
				u8state = COM_IDLE;
				u8lastError = NO_REPLY;
				u16errCnt++;
#ifdef DEBUG
#if defined(UBRR1H)
				Serial1.print("timeout:");
				Serial1.println(u16timeOut);
#endif
#endif
				return 0;
			}
		}

		while (network.available()) {
			RF24NetworkHeader headerAnswer;
			network.peek(headerAnswer);
#ifdef DEBUG
#if defined(UBRR1H)
			Serial1.print((char) headerAnswer.type);
			Serial1.print(" ");
#endif
#endif
			if (headerAnswer.type == headerRequest.type && headerAnswer.from_node == u8id) {
				size_t r = network.read(headerRequest, au8Buffer, MAX_BUFFER);
				if (r > 2) {
					unShiftData();
#ifdef DEBUG
#if defined(UBRR1H)
					printHex1(au8Buffer, u8BufferSize);
					Serial1.println();
#endif
#endif
					uint16_t u16MsgCRC =
							((au8Buffer[u8BufferSize - 2] << 8)
							| au8Buffer[u8BufferSize - 1]); // combine the crc Low & High bytes
					if (calcCRC(u8BufferSize - 2) != u16MsgCRC) {
						u16errCnt++;
						return NO_REPLY;
					} else {
						u8BufferSize -= 2; // next call sendTxBuffer will calc and add CRC
					}

					Modbus::sendTxBuffer();
					return r;
				} else {
					return 0;
				}

			} else {
#ifdef DEBUG
#if defined(UBRR1H)
				Serial1.println(" lost packet");
#endif
#endif
				while (network.available()) {
					network.update();
					network.read(headerRequest, NULL, 0);
				}
				port->flush();
				ser = 'A';
			}
		}

#ifdef DEBUG
#if defined(UBRR1H)
	} else {
		Serial1.println("send error");
#endif
#endif
	}


	return i8state;
}

/**
 * @brief
 * This method validates slave incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
uint8_t ModbusRF24::validateProxyRequest(uint8_t len, uint8_t offset = 0) {
	// check message crc vs calculated crc
	uint16_t u16MsgCRC =
			((au8Buffer[offset + len - 2] << 8)
			| au8Buffer[offset + len - 1]); // combine the crc Low & High bytes
	if (calcCRC(len - 2, offset) != u16MsgCRC) {
		u16errCnt++;
		return NO_REPLY;
	}

	// check fct code
	boolean isSupported = false;
	for (uint8_t i = 0; i< sizeof ( fctsupported); i++) {
		if (fctsupported[i] == au8Buffer[FUNC]) {
			isSupported = 1;
			break;
		}
	}
	if (!isSupported) {
		u16errCnt++;
		return EXC_FUNC_CODE;
	}

	return 0; // OK, no exception code thrown
}

