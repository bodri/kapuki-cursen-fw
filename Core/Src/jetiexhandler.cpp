/*
 * jetiexparser.cpp
 *
 *  Created on: 25 Jan 2020
 *      Author: gvaradi
 */

#include <jetiexhandler.h>
#include "crc.h"

#include <memory>
#include <string.h>

#define CRC8_POLYNOMIAL 0x07

//bool JetiExParser::valid() const {
//	return isValid;
//}
//
//void JetiExParser::setSentenceHandler(std::string command, std::function<void(const NmeaSentence&)> handler){
//	sentenceHandlers.erase(command);
//	sentenceHandlers.insert({ command, handler });
//}

void JetiExHandler::readByte(uint8_t byte) {
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
		uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) packet.c_str(), packet.size());
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
				const char *packet = createExTelemetryPacket().c_str();
//				TelemetryData data(1, "Current", "A");
//				uint8_t *packet = createTelemetryTextPacket(data);
				for (int i = 0; i < 1000; i++) { }
//				"\x9F\x10\xA1\xA4\x5D\x55\x00\x01\x39\x43\x75\x72\x72\x65\x6E\x74\x41\x00"; // Current
			} else {

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

void JetiExHandler::readBuffer(uint8_t *buffer, size_t size) {
	for (size_t i = 0; i < size; ++i) {
		readByte(buffer[i]);
	}
}

//
// Private
//
//void JetiExParser::restart(void) {
//	command.clear();
//	params.clear();
//	checksum = 0;
//	parsedChecksum.clear();
//}
//
//void JetiExParser::readSentence(std::string command, std::string params) {
//	NmeaSentence sentence;
//
//	if (command.size() < 5 || !validCommand(command)) {
//		sentence.isValid = false;
//		return;
//	}
//
//	sentence.command = command.substr(2, 3);
//
//    size_t start = 0;
//    size_t stop = start;
//
//    while (stop < params.length()) {
//        if (params[stop] == ',') {
//        	sentence.parameters.push_back(params.substr(start, stop - start));
//        	start = ++stop;
//        } else {
//            stop++;
//        }
//    }
//
//    sentence.parameters.push_back(params.substr(start, stop));
//
//	for (size_t i = 0; i < sentence.parameters.size(); i++) {
//		if (!validParamChars(sentence.parameters[i])) {
//			sentence.isValid = false;
//			return;
//		}
//	}
//
//	std::function<void(const NmeaSentence&)> handler = sentenceHandlers[sentence.command];
//	if (handler) {
//		handler(sentence);
//	}
//}
//
//bool JetiExParser::validCommand(std::string txt) {
//	for (size_t i = 0; i < txt.size(); i++) {
//		if (isalnum(txt[i])) {
//			return true;
//		}
//	}
//
//	return false;
//}
//
//bool JetiExParser::validParamChars(std::string txt) {
//	for (size_t i = 0; i < txt.size(); i++) {
//		if (!isalnum(txt[i])) {
//			if (txt[i] != '-' && txt[i] != '.') {
//				return false;
//			}
//		}
//	}
//
//	return true;
//}
//
//
//

std::string JetiExHandler::createExTelemetryPacket() {

	if (telemetryDataArray.size() <= 0) {
		return NULL;
	}

	auto const& telemetryData = telemetryDataArray[currentTextPacketPosition];
	if (currentTextPacketPosition >= telemetryDataArray.size()) {
		currentTextPacketPosition = 0;
	}
	std::string textPacket = createTelemetryTextPacket(telemetryData);

	uint8_t length = sizeof(textPacket) + 7;
	std::string buffer;

	buffer[0] = 0x3B;
	buffer[1] = 0x01;
	buffer[2] = length;
	buffer[3] = packet[3];
	buffer[4] = 0x3A;
	buffer[5] = length - 1;

	memcpy(&buffer[6], textPacket.c_str(), textPacket.size());
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buffer.c_str(), textPacket.size() + 5);
	buffer[textPacket.size() + 5] = (uint8_t) crc;
	buffer[textPacket.size() + 6] = (uint8_t) (crc >> 8);

	return buffer;
}

std::string JetiExHandler::createTelemetryTextPacket(const TelemetryData& data) {
	uint8_t length = data.description.size() + data.unit.size() + 10;
	std::string buffer;

	buffer[0] = 0x9F;
	buffer[1] = (length - 2) & 0x3F;

	buffer[2] = manufacturerId;
	buffer[3] = manufacturerId >> 8;
	buffer[4] = deviceId;
	buffer[5] = deviceId >> 8;

	buffer[6] = 0; // reserved
	buffer[7] = data.position;
	buffer[8] = ((data.description.size() & 0x1F) << 3) | (data.unit.size() & 0x03);

	memcpy(&buffer[9], data.description.c_str(), data.description.size());
	memcpy(&buffer[9 + data.description.size()], data.unit.c_str(), data.unit.size());

	buffer[length - 1] = calculateCrc8((uint8_t *)buffer.c_str() + 1, length - 2);

	return buffer;
}


//
// Copy from `JETI Telemetry communication protocol`. OMG those variable names :-)
//
uint8_t JetiExHandler::updateCrc(uint8_t crc, uint8_t crc_seed) {
	uint8_t crc_u;

	crc_u = crc;
	crc_u ^= crc_seed;

	for (uint8_t i = 0; i < 8; i++) {
		crc_u = (crc_u & 0x80) ? CRC8_POLYNOMIAL ^ (crc_u << 1) : (crc_u << 1);
	}
	return crc_u;
}

uint8_t JetiExHandler::calculateCrc8(uint8_t *crc, uint8_t crc_lenght) {
	uint8_t crc_up = 0;

	for (uint8_t c = 0; c < crc_lenght; c++) {
		crc_up = updateCrc(crc[c], crc_up);
	}

	return crc_up;
}

