/**
 * @file jetiexprotocol.cpp
 * @brief JETI EX protocol handler.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include <jetiexprotocol.h>
#include "crc.h"

#include <memory>
#include <numeric>

#define CRC8_POLYNOMIAL 0x07

bool JetiExProtocol::readByte(uint8_t byte) {
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
	case Done:
		parsedChecksum += byte << 8;
		uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)packet.data(), packet.size());
		if (crc == parsedChecksum) {
			for (int i = 0; i < 500; i++) { }

			if (releaseBusFlag) {
				if (packet[4] == jetiboxRequest) {
				} else if (packet[4] == telemetryRequest) {
					std::string packet = createExTelemetryPacket();
					onPacketSend((uint8_t *)packet.data(), packet.size());
				} else {
					std::string packet = createExDataPacket();
					onPacketSend((uint8_t *)packet.c_str(), packet.size());
				}
			}
		}

		state = Start;
		return true;
		break;
	}

	return false;
}

bool JetiExProtocol::readBuffer(uint8_t *buffer, size_t size) {
	uint32_t receivedValidPacket { 0 };
	for (size_t i = 0; i < size; ++i) {
		receivedValidPacket += readByte(buffer[i]);
	}
	return receivedValidPacket > 0;
}

//
// Private
//
std::string JetiExProtocol::createExDataPacket() {
	if (telemetryDataArray.size() <= 0) {
		return "";
	}

	std::string dataPacket = createTelemetryDataPacket();

	std::string buffer;
	uint8_t length = dataPacket.size() + 8;
	buffer.resize(length);

	buffer[0] = 0x3B;
	buffer[1] = 0x01;
	buffer[2] = length;
	buffer[3] = packet[3];
	buffer[4] = 0x3A;
	buffer[5] = dataPacket.size();

	std::copy(std::begin(dataPacket), std::end(dataPacket), std::begin(buffer) + 6);

	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buffer.data(), length - 2);
	buffer[length - 2] = (uint8_t) crc;
	buffer[length - 1] = (uint8_t) (crc >> 8);

	return buffer;
}

std::string JetiExProtocol::createExTelemetryPacket() {
	if (telemetryDataArray.size() <= 0) {
		return "";
	}

	const auto telemetryData = telemetryDataArray[currentTextPacketPosition++];
	if (currentTextPacketPosition >= telemetryDataArray.size()) {
		currentTextPacketPosition = 0;
	}
	std::string textPacket = createTelemetryTextPacket(telemetryData);

	std::string buffer;
	uint8_t length = textPacket.length() + 8;
	buffer.resize(length);

	buffer[0] = 0x3B;
	buffer[1] = 0x01;
	buffer[2] = length;
	buffer[3] = packet[3];
	buffer[4] = 0x3A;
	buffer[5] = textPacket.size();

	std::copy(std::begin(textPacket), std::end(textPacket), std::begin(buffer) + 6);

	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buffer.data(), length - 2);
	buffer[length - 2] = (uint8_t)crc;
	buffer[length - 1] = (uint8_t)(crc >> 8);

	return buffer;
}

void JetiExProtocol::createJetiboxPacket(void) {
//	  uint8_t cucc[] = "\x43\x65\x6E\x74\x72\x61\x6C\x20\x42\x6F\x78\x20\x31\x30\x30\x3E\x20\x20\x20\x34\x2E\x38\x56\x20\x20\x31\x30\x34\x30\x6D\x41\x00";
//	  uint8_t data[128];
//	  data[0] = 0x3B;
//	  data[1] = 0x01;
//	  data[2] = sizeof(cucc) + 7; //len
//	  data[3] = packetId; //0x08 packetId
//	  data[4] = 0x3B;
//	  data[5] = sizeof(cucc) - 1;
//
//	  uint8_t crc8 = calculateCrc8(cucc + 1, sizeof(cucc) - 3);
//	  cucc[sizeof(cucc) - 2] = crc8;
//
//	  memcpy(&data[6], cucc, sizeof(cucc));
//	  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data, sizeof(cucc) + 5);
//	  data[sizeof(cucc) + 5] = (uint8_t)crc;
//	  data[sizeof(cucc) + 6] = (uint8_t)(crc >> 8);
}

std::string JetiExProtocol::createTelemetryDataPacket() {
	std::string buffer;
	uint8_t length = std::accumulate(telemetryDataArray.begin(), telemetryDataArray.end(), 8,
			[](int n, auto const& telemetryData) { return n + telemetryData->numberOfValueBytes() + 1; });
	buffer.resize(length);

	buffer[0] = 0x9F;
	buffer[1] = 0x40 | ((length - 2) & 0x3F);
	buffer[2] = manufacturerId;
	buffer[3] = manufacturerId >> 8;
	buffer[4] = deviceId;
	buffer[5] = deviceId >> 8;
	buffer[6] = 0; // reserved

	uint8_t i = 7;
	for (auto const& telemetryData : telemetryDataArray) {
		auto bytes = telemetryData->numberOfValueBytes();
		if (bytes > 0) {
			buffer[i++] = ((telemetryData->position & 0x0F) << 4) | (telemetryData->dataType & 0x0F);
			buffer[i] = telemetryData->value;
			if (bytes > 1) {
				buffer[++i] = telemetryData->value >> 8;
			}
			if (bytes > 2) {
				buffer[++i] = telemetryData->value >> 16;
			}
			if (bytes > 3) {
				buffer[++i] = telemetryData->value >> 24;
			}
			buffer[i] = (telemetryData->value >= 0 ? 0x00 : 0x80)
					| ((telemetryData->decimalPointPosition & 0x03) << 5)
					| (buffer[i] & 0x1F);
		}
	}

	buffer[length - 1] = calculateCrc8((uint8_t *)buffer.data() + 1, length - 2);
	return buffer;
}

std::string JetiExProtocol::createTelemetryTextPacket(const TelemetryData *data) {
	std::string buffer;
	uint8_t length = data->description.length() + data->unit.length() + 10;
	buffer.resize(length);

	buffer[0] = 0x9F;
	buffer[1] = (length - 2) & 0x3F;
	buffer[2] = manufacturerId;
	buffer[3] = manufacturerId >> 8;
	buffer[4] = deviceId;
	buffer[5] = deviceId >> 8;
	buffer[6] = 0; // reserved
	buffer[7] = data->position;
	buffer[8] = ((data->description.size() & 0x1F) << 3) | (data->unit.length() & 0x03);

	std::copy(std::begin(data->description), std::end(data->description), std::begin(buffer) + 9);
	std::copy(std::begin(data->unit), std::end(data->unit), std::begin(buffer) + 9 + data->description.size());

	buffer[length - 1] = calculateCrc8((uint8_t *)buffer.data() + 1, length - 2);
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

