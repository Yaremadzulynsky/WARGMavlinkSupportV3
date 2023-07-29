// Created by Yarema Dzulynsky on 2023-07-24.

#include "BasicMavlinkEncoder.h"

BasicMavlinkEncoder::BasicMavlinkEncoder() = default;

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

    // Create a vector to store the encoded message
    std::vector<uint8_t> vectorBuffer;

    // Create a temporary variable to store the size of the encoded message in the buffer
    std::size_t tempSize;

    // Create a temporary buffer to store the encoded message before adding it to the vector.
    auto *tempBuffer = new uint8_t[MAVLINK_MAX_PACKET_LEN];

    /**
     * This is currently very primitive and only checks if the data is initialized not if it is valid.
     * It also only (Barely) supports the global_position_int and attitude messages. To create a more
     * robust system, I need to know exactly what kinds of messages I will be receiving as this part
     * of the code is very specific to the messages I am receiving and involves a lot of hard coding.
     */
    if(data.isLatitudeInitialized || data.isLongitudeInitialized || data.isAltitudeInitialized) {
        tempSize = ENCODE_MESSAGE(global_position_int, 0, data.latitude, data.longitude, data.altitude, 0, data.vx, data.vy, data.vz, 0)(currentMessage, tempBuffer);
        packIntoVector(tempSize, vectorBuffer, tempBuffer);
    }
    if(data.isRollInitialized || data.isPitchInitialized || data.isYawInitialized) {
        tempSize = ENCODE_MESSAGE(attitude, 0, data.roll, data.pitch, data.yaw, 0, 0, 0)(currentMessage, tempBuffer);
        packIntoVector(tempSize, vectorBuffer, tempBuffer);
    }

    // Delete the temporary buffer
    delete[] tempBuffer;

    return vectorBuffer;
}

