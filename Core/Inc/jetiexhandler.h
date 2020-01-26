/*
 * jetiexparser.h
 *
 *  Created on: 25 Jan 2020
 *      Author: gvaradi
 */

#ifndef __JETIEXPARSER__
#define __JETIEXPARSER__

#include <stdint.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

class JetiExHandler;

class TelemetryData {
	friend JetiExHandler;
public:
	TelemetryData(uint8_t position, std::string description, std::string unit, uint8_t decimalPointPosition) :
			position(position), description(description), unit(unit), decimalPointPosition(decimalPointPosition), value(0) {
	}
	~TelemetryData() { }

private:
	uint8_t position;
	std::string description;
	std::string unit;
	uint8_t decimalPointPosition;
	int16_t value;
};

enum ParserState {
	Start = 1,
	ReleaseBusFlag,
	Length,
	Packet,
	ChecksumChar1,
	ChecksumChar2,
	Done
};

inline ParserState& operator++(ParserState& s, int) {
	ParserState& temp = s;
    s = (s == Done ? Start : static_cast<ParserState>((static_cast<int>(s) + 1)));
    return temp;
}

class JetiExHandler {
public:
	JetiExHandler() { }
	~JetiExHandler() { }

//	void setSentenceHandler(std::string command, std::function<void(const NmeaSentence&)> handler);

	void readByte(uint8_t byte);
	void readBuffer(uint8_t *buffer, size_t size);

	std::vector<TelemetryData> telemetryDataArray { };


private:
	uint16_t manufacturerId { 0xA4A1 };
	uint16_t deviceId { 0x555D } ;

//	std::unordered_map<std::string, std::function<void(NmeaSentence)>> sentenceHandlers;
	ParserState state { Start };
	std::string packet;
	uint8_t packetLength { 0 };
	uint16_t parsedChecksum;
	uint8_t currentTextPacketPosition { 0 };

//	static const int maxSentenceParamLength { NMEA_MAX_SENTENCE_PARAM_LENGTH };

	std::string createExDataPacket();
	std::string createExTelemetryPacket();
	std::string createTelemetryDataPacket();
	std::string createTelemetryTextPacket(const TelemetryData& data);
	uint8_t updateCrc(uint8_t crc, uint8_t crc_seed);
	uint8_t calculateCrc8(uint8_t *crc, uint8_t crc_lenght);
};

#endif /* __JETIEXPARSER__ */
