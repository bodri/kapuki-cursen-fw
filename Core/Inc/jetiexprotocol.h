/**
 * @file jetiexprotocol.h
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

#ifndef __JETIEXPROTOCOL_H__
#define __JETIEXPROTOCOL_H__

#include <stdint.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <functional>

class JetiExProtocol;

enum TelemetryDataType {
	int6_t = 0,
	int14_t,
	int22_t = 4,
	int30_t = 8,
	zero = 100
};

class TelemetryData {
	friend JetiExProtocol;
	friend bool sortByPosition(const TelemetryData *lhs, const TelemetryData *rhs);

public:

	TelemetryData(uint8_t position, std::string description, std::string unit, TelemetryDataType dataType, uint8_t decimalPointPosition) :
			position(position),
			description(description),
			unit(unit),
			dataType(dataType),
			decimalPointPosition(decimalPointPosition),
			value(0) { }
	~TelemetryData() { }

	void setValue(int8_t value) {
		this->value = value;
	}

	void setValue(int16_t value) {
		this->value = value;
	}

	void setValue(int32_t value) {
		this->value = value;
	}

	uint8_t numberOfValueBytes() {
		switch (dataType) {
			case int6_t: return 1;
			case int14_t: return 2;
			case int22_t: return 3;
			case int30_t: return 4;
			default: return 0;
		}
	}

private:
	uint8_t position;
	std::string description;
	std::string unit;
	TelemetryDataType dataType;
	uint8_t decimalPointPosition;
	int32_t value;
};

enum ParserState {
	Start = 1,
	ReleaseBusFlag,
	Length,
	Packet,
	ChecksumChar1,
	Done
};

inline ParserState& operator++(ParserState& s, int) {
	ParserState& temp = s;
    s = (s == Done ? Start : static_cast<ParserState>((static_cast<int>(s) + 1)));
    return temp;
}

class JetiExProtocol {
public:
	JetiExProtocol(uint16_t manufacturerId, uint16_t deviceId, std::vector<TelemetryData *> telemetryDataArray);
	~JetiExProtocol() { }

	std::function<void(const uint8_t *packet, size_t size)> onPacketSend;

	// JetiBox functions
	std::function<std::string(const uint8_t buttonStatus)> onDisplayScreen;

	void addChannelObserver(uint8_t channel, std::function<void(uint16_t channelData)> callback) { channelDataObservers[channel - 1] = callback; }
	void removeChannelObserver(uint8_t channel) { channelDataObservers.erase(channel); };

	bool readByte(uint8_t byte);
	bool readBuffer(uint8_t *buffer, size_t size);

private:
	uint16_t manufacturerId;
	uint16_t deviceId;

	std::vector<TelemetryData *> telemetryDataArray;
	std::map<uint8_t, std::function<void(uint16_t value)>> channelDataObservers;

	ParserState state { Start };
	std::string packet;
	uint8_t packetLength { 0 };
	uint16_t parsedChecksum;
	uint8_t currentTextPacketPosition { 0 };

	void decodeChannelData();
	std::string createExDataPacket();
	std::string createExTextPacket();
	std::string createJetiboxPacket(uint8_t buttonStatus);
	std::string createTelemetryDataPacket();
	std::string createTelemetryTextPacket(const TelemetryData *data);
	uint8_t updateCrc(uint8_t crc, uint8_t crc_seed);
	uint8_t calculateCrc8(uint8_t *crc, uint8_t crc_lenght);
};

#endif /* __JETIEXPROTOCOL_H__ */
