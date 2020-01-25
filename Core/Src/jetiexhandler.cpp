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
		parsedChecksum = byte << 8;
		state++;
		break;
	case ChecksumChar2: {
		parsedChecksum += byte;
		uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) packet.c_str(), packet.size());
		if (crc == parsedChecksum) {
			state++;
		} else {
			state = Start;
		}
	}
		break;
	case Done:
		for (int i = 0; i < 1000; i++) { }
		state = Start;

		if (releaseBusFlag) {
			if (packet[4] == telemetryRequest) {
				TelemetryData data(1, "Current", "A");
				uint8_t *packet = createTelemetryTextPacket(data);

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

uint8_t *JetiExHandler::createTelemetryTextPacket(const TelemetryData& data) {
//	  uint8_t cucc[] = "\x9F\x0F\xA1\xA4\x5D\x55\x00\x02\x2ATemp.\xB0\x43\x00";
	uint8_t length = data.description.size() + data.unit.size() + 9;
	std::unique_ptr<uint8_t[]> buffer;
	buffer.reset(new uint8_t[length]);

	buffer[0] = 0x9F;
	buffer[1] = (length - 2) & 0x3F;

	buffer[2] = manufacturerId;
	buffer[4] = deviceId;

	buffer[6] = 0; // reserved
	buffer[7] = data.position;
	buffer[8] = ((data.description.size() & 0x1F) << 3) | (data.unit.size() & 0x03);

	memcpy(&buffer[9], data.description.c_str(), data.description.size());
	memcpy(&buffer[9 + data.description.size()], data.unit.c_str(), data.unit.size());
	uint8_t crc8 = calculateCrc8(buffer.get() + 1, length - 2);
	buffer[length - 2] = crc8;

	return buffer.get();
}

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

