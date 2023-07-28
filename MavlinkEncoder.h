//
// Created by Yarema Dzulynsky on 2023-06-29.
//

#ifndef WARGDRONEKITSUPPORTV2_MAVLINKENCODER_H
#define WARGDRONEKITSUPPORTV2_MAVLINKENCODER_H

#include "c_library_v2/common/mavlink.h"
#include "iostream"


class MavlinkEncoder {

public:

    MavlinkEncoder();
    //the current message being decoded
    mavlink_message_t currentMessage;
    //The decoding status of the current message being decoded
    mavlink_status_t currentMessageDecodingStatus;


    std::unordered_map<int, std::function<void(mavlink_message_t &, std::tuple<float...> &args...)>> encodingFunctions;

    uint16_t msgToBuffer(mavlink_message_t &msg, uint8_t *buffer);

    uint8_t *encodeAttitude(mavlink_message_t &msg, size_t &len, uint8_t *buffer, float roll, float pitch, float yaw,
                            float rollSpeed, float pitchSpeed, float yawSpeed);

    static uint8_t *combineBuffers(uint8_t *buffer1, uint8_t *buffer2, size_t len1, size_t len2);

    uint8_t *
    encodeGlobalPositionInt(mavlink_message_t &msg, size_t &len, uint8_t *buffer, int velocity, int heading,
                            int latitude,
                            int longitude, int altitude, int relativeAltitude);


    uint8_t * createUniqueMavlinkMsgBuffer(){
        mavlink_message_t msg;

//    mavlinkDecoder->decode(mavlinkEncoder->encodeGlobalPositionInt(msg, 1201, 43, 0, 12, 143, 0), MAVLINK_MAX_PACKET_LEN);

//        std::size_t attitudeBufferLen = 0;
//        std::size_t globalPositionIntLen = 0;
//
//        uint8_t attitudeBuffer[MAVLINK_MAX_PACKET_LEN] = {0};
//        uint8_t globalPositionIntBuffer[MAVLINK_MAX_PACKET_LEN] = {0};
//
//        mavlinkEncoder->encodeAttitude(msg, attitudeBufferLen, attitudeBuffer, 1, 2, 3, 4, 5, 6);
//
//        mavlinkEncoder->encodeGlobalPositionInt(msg, globalPositionIntLen, globalPositionIntBuffer, 1, 2, 3, 4, 5, 6);
//
//        std::size_t combinedBufferLength = attitudeBufferLen + globalPositionIntLen;
//        uint8_t *combinedBuffer = MavlinkEncoder::combineBuffers(attitudeBuffer, globalPositionIntBuffer, attitudeBufferLen, globalPositionIntLen);


    }
};


#endif //WARGDRONEKITSUPPORTV2_MAVLINKENCODER_H
