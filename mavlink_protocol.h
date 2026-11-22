#pragma once

// MAVLink/MSP framing: pure byte-level code, no Qt event loop, so tests/test_protocol.cpp can exercise it.

#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QString>

#include <cstdint>
#include <cstring>


constexpr uint8_t MavStxV1 = 0xFE;
constexpr uint8_t MavStxV2 = 0xFD;

constexpr uint8_t MavTypeGcs = 6;
constexpr uint8_t MavAutopilotInvalid = 8;
constexpr uint8_t MavStateActive = 4;

constexpr uint8_t MavFrameGlobalRelativeAltInt = 6;
constexpr uint8_t MavModeFlagCustomModeEnabled = 1;
constexpr uint8_t MavModeFlagSafetyArmed = 128;

constexpr uint8_t MavDataStreamExtendedStatus = 2;
constexpr uint8_t MavDataStreamPosition = 6;
constexpr uint8_t MavDataStreamExtra1 = 10;
constexpr uint8_t MavDataStreamExtra2 = 11;
constexpr uint8_t MavDataStreamExtra3 = 12;

constexpr uint8_t MspApiVersion = 1;
constexpr uint8_t MspFcVariant = 2;
constexpr uint8_t MspFcVersion = 3;
constexpr uint8_t MspStatus = 101;
constexpr uint8_t MspRawImu = 102;
constexpr uint8_t MspRawGps = 106;
constexpr uint8_t MspAttitude = 108;
constexpr uint8_t MspAltitude = 109;
constexpr uint8_t MspAnalog = 110;
constexpr uint8_t MspBatteryState = 130;

constexpr uint16_t MavCmdNavLand = 21;
constexpr uint16_t MavCmdNavTakeoff = 22;
constexpr uint16_t MavCmdNavReturnToLaunch = 20;
constexpr uint16_t MavCmdComponentArmDisarm = 400;

constexpr uint32_t ArduModeGuided = 4;
constexpr uint32_t ArduModeRtl = 6;

template <typename T>
T readLe(const QByteArray &data, int offset)
{
    T value{};
    if (offset < 0 || offset + int(sizeof(T)) > data.size()) {
        return value;
    }
    std::memcpy(&value, data.constData() + offset, sizeof(T));
    return value;
}

template <typename T>
void writeLe(QByteArray &data, int offset, T value)
{
    if (offset < 0 || offset + int(sizeof(T)) > data.size()) {
        return;
    }
    std::memcpy(data.data() + offset, &value, sizeof(T));
}

inline void crcAccumulate(uint8_t value, uint16_t &crc)
{
    value ^= uint8_t(crc & 0xFF);
    value ^= uint8_t(value << 4);
    crc = uint16_t((crc >> 8) ^ (uint16_t(value) << 8) ^ (uint16_t(value) << 3) ^ (uint16_t(value) >> 4));
}

inline uint16_t mavCrc(const QByteArray &headerWithoutStx, const QByteArray &payload, uint8_t crcExtra)
{
    uint16_t crc = 0xFFFF;
    for (char b : headerWithoutStx) {
        crcAccumulate(uint8_t(b), crc);
    }
    for (char b : payload) {
        crcAccumulate(uint8_t(b), crc);
    }
    crcAccumulate(crcExtra, crc);
    return crc;
}

inline uint8_t crcExtra(uint32_t msgId)
{
    switch (msgId) {
    case 0: return 50;   // HEARTBEAT
    case 1: return 124;  // SYS_STATUS
    case 11: return 89;  // SET_MODE
    case 24: return 24;  // GPS_RAW_INT
    case 30: return 39;  // ATTITUDE
    case 33: return 104; // GLOBAL_POSITION_INT
    case 66: return 148; // REQUEST_DATA_STREAM
    case 76: return 152; // COMMAND_LONG
    case 77: return 143; // COMMAND_ACK
    case 86: return 5;   // SET_POSITION_TARGET_GLOBAL_INT
    case 109: return 185; // RADIO_STATUS
    case 253: return 83; // STATUSTEXT
    default: return 0;
    }
}

inline QString arduModeName(uint32_t customMode)
{
    switch (customMode) {
    case 0: return "STABILIZE";
    case 1: return "ACRO";
    case 2: return "ALT_HOLD";
    case 3: return "AUTO";
    case 4: return "GUIDED";
    case 5: return "LOITER";
    case 6: return "RTL";
    case 7: return "CIRCLE";
    case 9: return "LAND";
    case 11: return "DRIFT";
    case 13: return "SPORT";
    case 14: return "FLIP";
    case 15: return "AUTOTUNE";
    case 16: return "POSHOLD";
    case 17: return "BRAKE";
    case 18: return "THROW";
    case 19: return "AVOID_ADSB";
    case 20: return "GUIDED_NOGPS";
    case 21: return "SMART_RTL";
    default: return QString("MODE %1").arg(customMode);
    }
}

struct MavPacket {
    uint32_t msgId = 0;
    uint8_t sysId = 0;
    uint8_t compId = 0;
    QByteArray payload;
};

struct MspPacket {
    uint8_t cmd = 0;
    bool error = false;
    QByteArray payload;
};

class MavParser {
public:
    QList<MavPacket> push(const QByteArray &bytes)
    {
        buffer_.append(bytes);
        if (buffer_.size() > maxBufferSize_) {
            buffer_.remove(0, buffer_.size() - maxBufferSize_);
        }

        QList<MavPacket> packets;

        while (!buffer_.isEmpty()) {
            const uint8_t stx = uint8_t(buffer_.at(0));
            if (stx != MavStxV1 && stx != MavStxV2) {
                buffer_.remove(0, 1);
                continue;
            }

            if (stx == MavStxV1) {
                if (buffer_.size() < 8) {
                    break;
                }
                const int payloadLen = uint8_t(buffer_.at(1));
                const int frameLen = 6 + payloadLen + 2;
                if (buffer_.size() < frameLen) {
                    break;
                }

                const QByteArray frame = buffer_.left(frameLen);
                const uint32_t msgId = uint8_t(frame.at(5));
                const QByteArray header = frame.mid(1, 5);
                const QByteArray payload = frame.mid(6, payloadLen);
                const uint16_t expected = readLe<uint16_t>(frame, 6 + payloadLen);
                if (mavCrc(header, payload, crcExtra(msgId)) != expected) {
                    // False STX, corrupt or unsupported frame: drop one byte and resync
                    // instead of discarding frameLen bytes that may hold real packets.
                    buffer_.remove(0, 1);
                    continue;
                }
                buffer_.remove(0, frameLen);

                packets.append({msgId, uint8_t(frame.at(3)), uint8_t(frame.at(4)), payload});
                continue;
            }

            if (buffer_.size() < 12) {
                break;
            }
            const int payloadLen = uint8_t(buffer_.at(1));
            const bool signedFrame = (uint8_t(buffer_.at(2)) & 0x01) != 0;
            const int signatureLen = signedFrame ? 13 : 0;
            const int frameLen = 10 + payloadLen + 2 + signatureLen;
            if (buffer_.size() < frameLen) {
                break;
            }

            const QByteArray frame = buffer_.left(frameLen);
            const uint32_t msgId = uint8_t(frame.at(7)) | (uint32_t(uint8_t(frame.at(8))) << 8) | (uint32_t(uint8_t(frame.at(9))) << 16);
            const QByteArray header = frame.mid(1, 9);
            QByteArray payload = frame.mid(10, payloadLen);
            const uint16_t expected = readLe<uint16_t>(frame, 10 + payloadLen);
            if (mavCrc(header, payload, crcExtra(msgId)) != expected) {
                buffer_.remove(0, 1);
                continue;
            }
            buffer_.remove(0, frameLen);

            // MAVLink v2 strips trailing zero bytes; restore them so fixed-size field reads work.
            payload.append(QByteArray(255 - payload.size(), '\0'));
            packets.append({msgId, uint8_t(frame.at(5)), uint8_t(frame.at(6)), payload});
        }

        return packets;
    }

    void clear()
    {
        buffer_.clear();
    }

private:
    static constexpr int maxBufferSize_ = 8192;
    QByteArray buffer_;
};

class MspParser {
public:
    QList<MspPacket> push(const QByteArray &bytes)
    {
        buffer_.append(bytes);
        if (buffer_.size() > maxBufferSize_) {
            buffer_.remove(0, buffer_.size() - maxBufferSize_);
        }

        QList<MspPacket> packets;

        while (buffer_.size() >= 6) {
            if (uint8_t(buffer_.at(0)) != '$') {
                buffer_.remove(0, 1);
                continue;
            }

            if (buffer_.size() < 3) {
                break;
            }

            if (uint8_t(buffer_.at(1)) != 'M') {
                buffer_.remove(0, 1);
                continue;
            }

            const uint8_t direction = uint8_t(buffer_.at(2));
            if (direction != '>' && direction != '!') {
                buffer_.remove(0, 1);
                continue;
            }

            const int payloadLen = uint8_t(buffer_.at(3));
            const int frameLen = 6 + payloadLen;
            if (buffer_.size() < frameLen) {
                break;
            }

            const QByteArray frame = buffer_.left(frameLen);
            uint8_t checksum = uint8_t(frame.at(3)) ^ uint8_t(frame.at(4));
            const QByteArray payload = frame.mid(5, payloadLen);
            for (char b : payload) {
                checksum ^= uint8_t(b);
            }

            if (checksum != uint8_t(frame.at(5 + payloadLen))) {
                buffer_.remove(0, 1);
                continue;
            }
            buffer_.remove(0, frameLen);

            packets.append({uint8_t(frame.at(4)), direction == '!', payload});
        }

        return packets;
    }

    void clear()
    {
        buffer_.clear();
    }

private:
    static constexpr int maxBufferSize_ = 8192;
    QByteArray buffer_;
};

inline QByteArray makeV1Packet(uint8_t &seq, uint8_t sysId, uint8_t compId, uint8_t msgId, const QByteArray &payload)
{
    QByteArray header;
    header.append(char(payload.size()));
    header.append(char(seq++));
    header.append(char(sysId));
    header.append(char(compId));
    header.append(char(msgId));

    const uint16_t crc = mavCrc(header, payload, crcExtra(msgId));

    QByteArray packet;
    packet.append(char(MavStxV1));
    packet.append(header);
    packet.append(payload);
    packet.append(char(crc & 0xFF));
    packet.append(char((crc >> 8) & 0xFF));
    return packet;
}

inline QByteArray makeMspRequest(uint8_t cmd, const QByteArray &payload = QByteArray())
{
    QByteArray packet;
    packet.append('$');
    packet.append('M');
    packet.append('<');
    packet.append(char(payload.size()));
    packet.append(char(cmd));

    uint8_t checksum = uint8_t(payload.size()) ^ cmd;
    for (char b : payload) {
        checksum ^= uint8_t(b);
        packet.append(b);
    }

    packet.append(char(checksum));
    return packet;
}

