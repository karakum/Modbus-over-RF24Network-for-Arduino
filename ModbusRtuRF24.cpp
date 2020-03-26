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
void ModbusRF24::printHex(unsigned char* data, int len) {
	for (int i = 0; i < len; i++) {
		PRINT(data[i], HEX);
		PRINT(" ");
	}
}
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

				PRINTHEX(au8Buffer, u8BufferSize);
				PRINT(" -> ");

				if (u8BufferSize >= MAX_BUFFER) bBuffOverflow = true;
			}
		} else {
			PRINTLN("repeat");

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

	PRINTHEX(au8Buffer, u8BufferSize);
	PRINTLN();

	shiftData();

	RF24NetworkHeader headerAnswer(masterNode, ser);
	if (!network.write(headerAnswer, au8Buffer, u8BufferSize + 1)) {
		PRINTLN("fail");
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

	PRINT("Available: ");
	PRINTLN(u8current);

	u8lastRec = 0;
	int8_t i8state = Modbus::getRxBuffer();
	u8lastError = i8state;
	if (i8state < 7) return i8state;

	PRINTLN(u8BufferSize);
	PRINTLN(i8state);
	PRINTHEX(au8Buffer, i8state);
	PRINT(" -> ");

	u8id = au8Buffer[ ID ];

	// validate message: CRC, FCT, address and size
	uint8_t u8exception = validateProxyRequest(i8state);
	if (u8exception > 0) {
		if (u8exception != NO_REPLY) {
			buildException(u8exception);
			sendTxBuffer();
		}
		u8lastError = u8exception;

		PRINTLN("not valid");

		return u8exception;
	}

	u32timeOut = millis() + long(u16timeOut);
	u8lastError = 0;

	RF24NetworkHeader headerRequest(u8id, ser);

	shiftData();
	if (network.write(headerRequest, au8Buffer, i8state + 1)) {
		ser++;
		if (ser > 'Z') ser = 'A';

		PRINT((char) headerRequest.type);
		PRINT(":");

		while (!network.available()) {
			network.update();

			if (millis() > u32timeOut) {
				u8state = COM_IDLE;
				u8lastError = NO_REPLY;
				u16errCnt++;

				PRINT("timeout:");
				PRINTLN(u16timeOut);

				return 0;
			}
		}

		while (network.available()) {
			RF24NetworkHeader headerAnswer;
			network.peek(headerAnswer);

			PRINT((char) headerAnswer.type);
			PRINT(" ");

			if (headerAnswer.type == headerRequest.type && headerAnswer.from_node == u8id) {
				size_t r = network.read(headerRequest, au8Buffer, MAX_BUFFER);
				if (r > 2) {
					unShiftData();

					PRINTHEX(au8Buffer, u8BufferSize);
					PRINTLN();

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
				PRINTLN(" lost packet");

				while (network.available()) {
					network.update();
					network.read(headerRequest, NULL, 0);
				}
				port->flush();
				ser = 'A';
			}
		}

#ifdef DEBUG
	} else {
		PRINTLN("send error");
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

