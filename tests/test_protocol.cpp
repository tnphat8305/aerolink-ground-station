// Self-check for the MAVLink/MSP framing. Reference frames were generated with pymavlink 2.x.
#undef NDEBUG // keep asserts active in Release builds
#include "../mavlink_protocol.h"

#include <cassert>
#include <cstdio>

int main()
{
    // HEARTBEAT v1: type=QUADROTOR, autopilot=ARDUPILOTMEGA, base_mode=ARMED|CUSTOM, custom_mode=4 (GUIDED)
    const QByteArray heartbeatV1 = QByteArray::fromHex("fe09000101000400000002038104039995");
    // ATTITUDE v2, roll=0.5 pitch=-0.25 yaw=1.0, rates=0 -> payload truncated to 16 of 28 bytes on the wire
    const QByteArray attitudeV2 = QByteArray::fromHex("fd1000000001011e0000e80300000000003f000080be0000803f9670");

    {
        MavParser parser;
        const auto packets = parser.push(heartbeatV1);
        assert(packets.size() == 1);
        assert(packets[0].msgId == 0 && packets[0].sysId == 1 && packets[0].compId == 1);
        assert(readLe<uint32_t>(packets[0].payload, 0) == 4);
        assert(uint8_t(packets[0].payload.at(5)) != MavAutopilotInvalid);
        assert(uint8_t(packets[0].payload.at(6)) & MavModeFlagSafetyArmed);
    }

    {
        // Truncated v2 payload must be zero-extended so the 28-byte ATTITUDE handler accepts it.
        MavParser parser;
        const auto packets = parser.push(attitudeV2);
        assert(packets.size() == 1);
        assert(packets[0].msgId == 30);
        assert(packets[0].payload.size() >= 28);
        assert(readLe<float>(packets[0].payload, 4) == 0.5f);
        assert(readLe<float>(packets[0].payload, 8) == -0.25f);
        assert(readLe<float>(packets[0].payload, 24) == 0.0f);
    }

    {
        // Noise containing false STX bytes, then a frame split across two reads: nothing real is lost.
        MavParser parser;
        QByteArray noisy = QByteArray::fromHex("00fd05fe2011223344") + heartbeatV1 + attitudeV2.left(10);
        auto packets = parser.push(noisy);
        packets += parser.push(attitudeV2.mid(10) + heartbeatV1);
        assert(packets.size() == 3);
        assert(packets[0].msgId == 0 && packets[1].msgId == 30 && packets[2].msgId == 0);
    }

    {
        // Corrupted CRC is rejected, the following good frame still parses.
        MavParser parser;
        QByteArray bad = heartbeatV1;
        bad[bad.size() - 1] = char(bad.at(bad.size() - 1) ^ 0xFF);
        const auto packets = parser.push(bad + heartbeatV1);
        assert(packets.size() == 1);
    }

    {
        // Our encoder round-trips through our parser.
        uint8_t seq = 0;
        const QByteArray payload = QByteArray::fromHex("000000000608000403");
        const auto packets = MavParser().push(makeV1Packet(seq, 255, 190, 0, payload));
        assert(packets.size() == 1 && packets[0].sysId == 255 && packets[0].payload == payload);
        assert(seq == 1);
    }

    {
        // MSP: response frame "$M>" with XOR checksum, preceded by garbage and a bad-checksum frame.
        QByteArray response("$M>", 3);
        response.append(char(3)).append(char(MspApiVersion)).append(char(0)).append(char(1)).append(char(46));
        response.append(char(3 ^ MspApiVersion ^ 0 ^ 1 ^ 46));
        QByteArray badFrame = response;
        badFrame[badFrame.size() - 1] = char(0);
        const auto packets = MspParser().push(QByteArray("xx") + badFrame + response);
        assert(packets.size() == 1);
        assert(packets[0].cmd == MspApiVersion && !packets[0].error && packets[0].payload.size() == 3);

        const QByteArray request = makeMspRequest(MspStatus);
        assert(request == QByteArray::fromHex("244d3c006565"));
    }

    std::puts("test_protocol: all checks passed");
    return 0;
}
