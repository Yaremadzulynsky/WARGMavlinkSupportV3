// Created by Yarema Dzulynsky on 2023-07-24.

#include "BasicMavlinkEncoder.h"

BasicMavlinkEncoder::BasicMavlinkEncoder() = default;

std::size_t
BasicMavlinkEncoder::encodeAttitude(mavlink_message_t &msg, uint8_t *buffer, uint32_t time_boot_ms, float roll,
                                    float pitch, float yaw, float rollSpeed, float pitchSpeed, float yawSpeed) {
    return ENCODE_MESSAGE(attitude, time_boot_ms, roll, pitch, yaw, rollSpeed, pitchSpeed, yawSpeed)(msg, buffer);
}

std::size_t BasicMavlinkEncoder::encodeGlobalPositionInt(mavlink_message_t &msg, uint8_t *buffer, uint32_t time_boot_ms,
                                                         int32_t lat, int32_t lon, int32_t alt, int32_t relativeAlt,
                                                         int16_t vx, int16_t vy, int16_t vz, uint16_t hdg) {
    return ENCODE_MESSAGE(global_position_int, time_boot_ms, lat, lon, alt, relativeAlt, vx, vy, vz, hdg)(msg, buffer);
}

/**
 * Add the contents of a buffer to the end of a vector
 * @param bufferLen - the length of the buffer
 * @param vectorBuffer - the vector to add the buffer to
 * @param buffer - the buffer to add to the vector
 */
void packIntoVector(size_t bufferLen, std::vector<uint8_t> &vectorBuffer, const uint8_t *bufferToAdd) {

    // Add the contents of the buffer to the vector with a for loop
    for (int i = 0; i < bufferLen; i++) {
        vectorBuffer.push_back(bufferToAdd[i]);
    }
}


std::vector<uint8_t> BasicMavlinkEncoder::findPackingFunction(IncomingData &data) {

    std::vector<uint8_t> vectorBuffer;
    std::size_t tempSize;

    auto *tempBuffer = new uint8_t[MAVLINK_MAX_PACKET_LEN];

    if(data.isLatitudeInitialized || data.isLongitudeInitialized || data.isAltitudeInitialized) {
        tempSize = ENCODE_MESSAGE(global_position_int, 0, data.latitude, data.longitude, data.altitude, 0, data.vx, data.vy, data.vz, 0)(currentMessage, tempBuffer);
        packIntoVector(tempSize, vectorBuffer, tempBuffer);
    }
    if(data.isRollInitialized || data.isPitchInitialized || data.isYawInitialized) {
        tempSize = ENCODE_MESSAGE(attitude, 0, data.roll, data.pitch, data.yaw, 0, 0, 0)(currentMessage, tempBuffer);
        packIntoVector(tempSize, vectorBuffer, tempBuffer);
    }

    delete[] tempBuffer;

    return vectorBuffer;
}

