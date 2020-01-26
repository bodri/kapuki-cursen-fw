/*
 * jetiexprotocol.h
 *
 *  Created on: 25 Jan 2020
 *      Author: gvaradi
 */

#ifndef __JETIEXPROTOCOL__
#define __JETIEXPROTOCOL__

#include <stdint.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

class JetiExProtocol;

class TelemetryData {
	friend JetiExProtocol;
public:
	TelemetryData(uint8_t position, std::string description, std::string unit, uint8_t decimalPointPosition) :
			position(position),
			description(description),
			unit(unit),
			decimalPointPosition(decimalPointPosition),
			value(0) { }
	~TelemetryData() { }

	void setValue(int16_t value) {
		this->value = value;
	}

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

class JetiExProtocol {
public:
	JetiExProtocol(uint16_t manufacturerId, uint16_t deviceId, std::vector<TelemetryData *> telemetryDataArray) :
		manufacturerId(manufacturerId),
		deviceId(deviceId),
		telemetryDataArray(telemetryDataArray) { }
	~JetiExProtocol() { }

	std::function<void(const uint8_t *packet, size_t size)> onPacketSend;

	void readByte(uint8_t byte);
	void readBuffer(uint8_t *buffer, size_t size);

private:
	uint16_t manufacturerId;
	uint16_t deviceId;

	std::vector<TelemetryData *> telemetryDataArray;

	ParserState state { Start };
	std::string packet;
	uint8_t packetLength { 0 };
	uint16_t parsedChecksum;
	uint8_t currentTextPacketPosition { 0 };

//	static const int maxSentenceParamLength { NMEA_MAX_SENTENCE_PARAM_LENGTH };

	std::string createExDataPacket();
	std::string createExTelemetryPacket();
	std::string createTelemetryDataPacket();
	std::string createTelemetryTextPacket(const TelemetryData *data);
	uint8_t updateCrc(uint8_t crc, uint8_t crc_seed);
	uint8_t calculateCrc8(uint8_t *crc, uint8_t crc_lenght);
};

#endif /* __JETIEXPROTOCOL__ */
