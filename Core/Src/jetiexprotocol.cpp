/*
 * jetiexprotocol.cpp
 *
 *  Created on: 25 Jan 2020
 *      Author: gvaradi
 */

#include <jetiexprotocol.h>
#include "crc.h"

#include <memory>
//#include <string.h>

#define CRC8_POLYNOMIAL 0x07

void JetiExProtocol::readByte(uint8_t byte) {
	static const uint8_t startChar1 = 0x3E;
	static const uint8_t startChar2 = 0x3D;
	static const uint8_t releaseBusFlag = 0x01;
	static const uint8_t telemetryRequest = 0x3A;
	static const uint8_t jetiboxRequest = 0x3B;

	switch (state) {
	case Start:
		if (byte == startChar1 || byte == startChar2) {
			packet.clear();
			packet.push_back(byte);
			state++;
		}
		break;
	case ReleaseBusFlag:
		if (byte == releaseBusFlag) {
			packet.push_back(byte);
			state++;
		} else {
			state = Start;
		}
		break;
	case Length:
		packetLength = byte - 3; // 2 (byte header) + 1 (zero based)
		if (packetLength >= 3) {
			packet.push_back(byte);
			state++;
		} else {
			state = Start;
		}
		break;
	case Packet:
		packetLength--;
		packet.push_back(byte);
		if (packetLength <= 2) {
			state++;
		}
		break;

	case ChecksumChar1:
		parsedChecksum = byte;
		state++;
		break;
	case ChecksumChar2: {
		parsedChecksum += byte << 8;
		uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)packet.c_str(), packet.size());
		if (crc == parsedChecksum) {
			state++;
		} else {
			state = Start;
		}
	}
		break;
	case Done:
		for (int i = 0; i < 500; i++) { }
		state = Start;

		if (releaseBusFlag) {
			if (packet[4] == jetiboxRequest) {
			} else if (packet[4] == telemetryRequest) {
				std::string packet = createExTelemetryPacket();
				onPacketSend((uint8_t *)packet.c_str(), packet.size());

//				TelemetryData data(1, "Current", "A");
//				uint8_t *packet = createTelemetryTextPacket(data);
				uint8_t *cucc = (uint8_t *)packet.c_str();
				for (int i = 0; i < 1000; i++) { }
//				"\x9F\x10\xA1\xA4\x5D\x55\x00\x01\x39\x43\x75\x72\x72\x65\x6E\x74\x41\x00"; // Current
			} else {
				std::string packet = createExDataPacket();
				onPacketSend((uint8_t *)packet.c_str(), packet.size());
				uint8_t *cucc = (uint8_t *)packet.c_str();
				for (int i = 0; i < 1000; i++) { }

//	uint8_t cucc[] = "\x9F\x54\xA1\xA4\x5D\x55\x00\x11\xE8\x23\x21\x1A\x00\x31\x1A\x00\x41\x1A\x00\x00";
			}
		}

//		size_t maxSize = 16;
//		std::unique_ptr<char[]> formattedChecksum;
//		formattedChecksum.reset(new char[maxSize]);
//		snprintf(formattedChecksum.get(), maxSize, checksumFormat, checksum);
//		std::string calculatedChecksum = std::string(formattedChecksum.get());
//
//		if (byte == lineFeed && calculatedChecksum.compare(parsedChecksum) == 0) {
//			readSentence(command, params);
//		} else {
//			restart();
//		}
		break;
	}
}

void JetiExProtocol::readBuffer(uint8_t *buffer, size_t size) {
	for (size_t i = 0; i < size; ++i) {
		readByte(buffer[i]);
	}
}

//
// Private
//
std::string JetiExProtocol::createExDataPacket() {
	if (telemetryDataArray.size() <= 0) {
		return "";
	}

	std::string dataPacket = createTelemetryDataPacket();

	uint8_t length = dataPacket.size() + 8;
	std::string buffer;

	buffer[0] = 0x3B;
	buffer[1] = 0x01;
	buffer[2] = length;
	buffer[3] = packet[3];
	buffer[4] = 0x3A;
	buffer[5] = dataPacket.size();

//	memcpy(&buffer[6], dataPacket.c_str(), dataPacket.size());
	std::copy(std::begin(dataPacket), std::end(dataPacket), &buffer[6]);
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buffer.c_str(), dataPacket.size() + 5);
	buffer[dataPacket.size() + 6] = (uint8_t) crc;
	buffer[dataPacket.size() + 7] = (uint8_t) (crc >> 8);

	return buffer;
}

std::string JetiExProtocol::createExTelemetryPacket() {
	if (telemetryDataArray.size() <= 0) {
		return "";
	}

	auto const telemetryData = telemetryDataArray[currentTextPacketPosition];
	if (currentTextPacketPosition >= telemetryDataArray.size()) {
		currentTextPacketPosition = 0;
	}
	std::string textPacket = createTelemetryTextPacket(telemetryData);

	uint8_t length = textPacket.size() + 8;
	std::string buffer;

	buffer[0] = 0x3B;
	buffer[1] = 0x01;
	buffer[2] = length;
	buffer[3] = packet[3];
	buffer[4] = 0x3B;
	buffer[5] = textPacket.size();

//	memcpy(&buffer[6], textPacket.c_str(), textPacket.size());
	std::copy(std::begin(textPacket), std::end(textPacket), &buffer[6]);
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buffer.c_str(), textPacket.size() + 5);
	buffer[textPacket.size() + 6] = (uint8_t) crc;
	buffer[textPacket.size() + 7] = (uint8_t) (crc >> 8);

	return buffer;
}

std::string JetiExProtocol::createTelemetryDataPacket() {
	uint8_t length = 3 * telemetryDataArray.size() + 8;
	std::string buffer;

	buffer[0] = 0x9F;
	buffer[1] = (length - 2) & 0x3F;
	buffer[2] = manufacturerId;
	buffer[3] = manufacturerId >> 8;
	buffer[4] = deviceId;
	buffer[5] = deviceId >> 8;
	buffer[6] = 0; // reserved

	uint8_t i = 7;
	for (auto&& telemetryData : telemetryDataArray) {
		buffer[i++] = telemetryData->position;
		buffer[i++] = telemetryData->value;
		buffer[i++] = (telemetryData->value > 0 ? 0x00 : 0x80) |
				((telemetryData->decimalPointPosition & 0x03) << 6) |
				((telemetryData->value >> 8) & 0x1F);
	}

	buffer[length - 1] = calculateCrc8((uint8_t *)buffer.c_str() + 1, length - 2);

	return buffer;
}

std::string JetiExProtocol::createTelemetryTextPacket(const TelemetryData *data) {
	uint8_t length = data->description.size() + data->unit.size() + 10;
	std::string buffer;

	buffer[0] = 0x9F;
	buffer[1] = (length - 2) & 0x3F;
	buffer[2] = manufacturerId;
	buffer[3] = manufacturerId >> 8;
	buffer[4] = deviceId;
	buffer[5] = deviceId >> 8;
	buffer[6] = 0; // reserved
	buffer[7] = data->position;
	buffer[8] = ((data->description.size() & 0x1F) << 3) | (data->unit.size() & 0x03);

//	memcpy(&buffer[9], data->description.c_str(), data->description.size());
	std::copy(std::begin(data->description), std::end(data->description), &buffer[9]);
//	memcpy(&buffer[9 + data->description.size()], data->unit.c_str(), data->unit.size());
	std::copy(std::begin(data->unit), std::end(data->unit), &buffer[9 + data->description.size()]);

	buffer[length - 1] = calculateCrc8((uint8_t *)buffer.c_str() + 1, length - 2);

	return buffer;
}

//
// Copy from `JETI Telemetry communication protocol`. OMG those variable names :-)
//
uint8_t JetiExProtocol::updateCrc(uint8_t crc, uint8_t crc_seed) {
	uint8_t crc_u;

	crc_u = crc;
	crc_u ^= crc_seed;

	for (uint8_t i = 0; i < 8; i++) {
		crc_u = (crc_u & 0x80) ? CRC8_POLYNOMIAL ^ (crc_u << 1) : (crc_u << 1);
	}
	return crc_u;
}

uint8_t JetiExProtocol::calculateCrc8(uint8_t *crc, uint8_t crc_lenght) {
	uint8_t crc_up = 0;

	for (uint8_t c = 0; c < crc_lenght; c++) {
		crc_up = updateCrc(crc[c], crc_up);
	}

	return crc_up;
}

