/**
 * @file jetiexprotocol.cpp
 * @brief JETI EX protocol handler.
 *
 * @author Varadi, Gyorgy aka bodri
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
#include <algorithm>

#define CRC8_POLYNOMIAL 0x07

bool sortByPosition(const TelemetryData *lhs, const TelemetryData *rhs) {
	return lhs->position < rhs->position;
}

JetiExProtocol::JetiExProtocol(uint16_t manufacturerId, uint16_t deviceId, std::vector<TelemetryData *> telemetryDataArray) :
	manufacturerId(manufacturerId),
	deviceId(deviceId),
	telemetryDataArray(telemetryDataArray) {
    std::sort(this->telemetryDataArray.begin(), this->telemetryDataArray.end(), sortByPosition);

    std::vector<TelemetryData *>::iterator it = this->telemetryDataArray.begin();
    if ((*it)->position != 0) {
        it = this->telemetryDataArray.insert(it, new TelemetryData(0, "Noname sensor", "", zero, 0));
    }
}

bool JetiExProtocol::readByte(uint8_t byte) {
	static const uint8_t startChar1 = 0x3E;
	static const uint8_t startChar2 = 0x3D;
	static const uint8_t releaseBusFlag = 0x01;
	static const uint8_t channelData = 0x31;
	static const uint8_t telemetryDataRequest = 0x3A;
	static const uint8_t jetiboxRequest = 0x3B;

	switch (state) {
	case Start:
		packet.clear();
		if (byte == startChar1 || byte == startChar2) {
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
			if (packet[1] == releaseBusFlag) {
				uint8_t dataIdentifier = packet[4];

				if (dataIdentifier == channelData) {
					decodeChannelData();
				} else if (dataIdentifier == jetiboxRequest) {
					std::string jetiboxPacket = createJetiboxPacket(packet[6]);
					onPacketSend((uint8_t *)jetiboxPacket.c_str(), jetiboxPacket.size());
				} else if (dataIdentifier == telemetryDataRequest) {
					std::string telemetryDataPacket = createExDataPacket();
					onPacketSend((uint8_t *)telemetryDataPacket.c_str(), telemetryDataPacket.size());
				} else {
					std::string telemetryTextPacket = createExTextPacket();
					onPacketSend((uint8_t *)telemetryTextPacket.data(), telemetryTextPacket.size());
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
void JetiExProtocol::decodeChannelData() {
	uint8_t numberOfChannels = packet[5] / 2; // channel data on 2 bytes

	for (int i = 0; i < numberOfChannels; i++) {
		uint16_t channelData = packet[6 + i * 2] | packet[7 + i * 2] << 8;
		if (channelDataObservers[i] != nullptr) {
			channelDataObservers[i](channelData);
		}
	}
}

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

std::string JetiExProtocol::createExTextPacket() {
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

std::string JetiExProtocol::createJetiboxPacket(uint8_t buttonStatus) {
	std::string buffer;
	uint8_t length = 40; // screen size + 8
	buffer.resize(length);
	buffer[0] = 0x3B;
	buffer[1] = 0x01;
	buffer[2] = length;
	buffer[3] = packet[3];
	buffer[4] = 0x3B;
	buffer[5] = 32;

	std::string text;
	if (onDisplayScreen != nullptr) {
		text = onDisplayScreen(buttonStatus);
	}
	text.resize(32);
	std::copy(std::begin(text), std::end(text), std::begin(buffer) + 6);

	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) buffer.data(), length - 2);
	buffer[length - 2] = (uint8_t)crc;
	buffer[length - 1] = (uint8_t)(crc >> 8);

	return buffer;
}

std::string JetiExProtocol::createTelemetryDataPacket() {
	std::string buffer;
	uint8_t length = std::accumulate(telemetryDataArray.begin() + 1, telemetryDataArray.end(), 8,
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
		if (telemetryData->position > 0 && bytes > 0) {
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
			i++;
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

