//
// Created by Yarema Dzulynsky on 2023-07-24.
//

#ifndef WARGMAVLINKSUPPORT_BASICMAVLINKENCODER_H
#define WARGMAVLINKSUPPORT_BASICMAVLINKENCODER_H

#include "c_library_v2/common/mavlink.h"
#include "iostream"


struct IncomingData {

//    IncomingData() {
//            //init all fields
//
//    };
    float latitude = 0;
    bool isLatitudeInitialized = false;

    float longitude = 0;
    bool isLongitudeInitialized = false;

    float vx = 0;
    bool isVxInitialized = false;

    float vy = 0;
    bool isVyInitialized = false;

    float vz = 0;
    bool isVzInitialized = false;

    int altitude = 0;
    bool isAltitudeInitialized = false;

    float pitch = 0;
    bool isPitchInitialized = false;

    float roll = 0;
    bool isRollInitialized = false;

    float yaw = 0;
    bool isYawInitialized = false;
};

/**
 * @brief ENCODE_MESSAGE macro for MAVLink message packing
 *
 * This macro is used to pack MAVLink messages of various types. It takes the base name of the MAVLink
 * message and a variable number of arguments specific to the message type. It creates an anonymous lambda
 * function that packs the provided arguments into a MAVLink message and converts it to a byte buffer.
 *
 * @param baseName The base name of the MAVLink message (e.g., HEARTBEAT, COMMAND_LONG, etc.).
 * @param ...      Variable number of arguments specific to the MAVLink message type. These arguments
 *                 are used for populating the fields of the message.
 *
 * @return std::size_t The length of the byte buffer containing the packed MAVLink message.
 *
 * @note The ENCODE_MESSAGE macro uses lambda functions to encapsulate the packing process for different
 * MAVLink message types. The resulting lambda function can be used to efficiently pack MAVLink messages
 * and obtain the length of the packed buffer.
 *
 * @note The ENCODE_MESSAGE macro is used in the following way:
 *      ENCODE_MESSAGE(HEARTBEAT, 1, 2, 3, 4, 5, 6, 7, 8, 9)(msg, buffer);
 *      where HEARTBEAT is the base name of the MAVLink message and 1, 2, 3, 4, 5, 6, 7, 8, 9 are the
 *      arguments specific to the HEARTBEAT message type.
 *      The parameters msg and buffer are passed by reference to the lambda function and are used to
 *      store the packed MAVLink message and the packed buffer, respectively.
 *
 */
#define ENCODE_MESSAGE(baseName, ...) \
    ([&](mavlink_message_t& msg, uint8_t* buffer) -> std::size_t { \
        mavlink_msg_##baseName##_pack(0, 0, &msg, __VA_ARGS__); \
        std::size_t bufferLen = mavlink_msg_to_send_buffer(buffer, &msg); \
        return bufferLen; \
    })



class BasicMavlinkEncoder {
public:
    //The current message being encoded
    mavlink_message_t currentMessage;



    BasicMavlinkEncoder();

    /**
     * Encode an attitude message into the buffer
     * @param msg - the message object to pass by reference
     * @param buffer - the buffer to encode the message into
     * @param time_boot_ms - the time in milliseconds since boot
     * @param roll - the roll angle
     * @param pitch - the pitch angle
     * @param yaw - the yaw angle
     * @param rollspeed - the roll speed
     * @param pitchspeed - the pitch speed
     * @param yawspeed - the yaw speed
     * @return The length of the buffer containing the encoded message.
     */
    std::size_t encodeAttitude(mavlink_message_t &msg, uint8_t *buffer, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);


    /**
     * Encode a global position int message into the buffer
     * @param msg - the message object to pass by reference
     * @param buffer - the buffer to encode the message into
     * @param time_boot_ms - the time in milliseconds since boot
     * @param lat - the latitude
     * @param lon - the longitude
     * @param alt - the altitude
     * @param relativeAlt - the relative altitude
     * @param vx - the velocity in the x direction
     * @param vy - the velocity in the y direction
     * @param vz - the velocity in the z direction
     * @param hdg - the heading
     * @return The length of the buffer containing the encoded message.
     */
    size_t encodeGlobalPositionInt(mavlink_message_t &msg, uint8_t *buffer, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relativeAlt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);


    /**
     * Find the correct combination of mavlink message types and pack the data structure into mavlink format.
     * @param data - the data structure to pack which contains the data to pack.
     * @return The packed buffer within a vector.
     */
    std::vector<uint8_t> findPackingFunction(IncomingData &data);

    /**
     * Combine two buffers into one
     * @param buffer1 - the first buffer to combine
     * @param buffer2 - the buffer to combine with buffer1
     * @param len1 - the length of the first buffer
     * @param len2 - the length of the second buffer
     * @return The combined buffer.
     */
    static uint8_t *combineBuffers(uint8_t *buffer1, uint8_t *buffer2, size_t len1, size_t len2) {

        //Allocate a new buffer to hold the combined buffers
        auto *combinedBuffer = new uint8_t[len1 + len2];

        //Copy the first buffer into the combined buffer
        std::copy(buffer1, buffer1 + len1, combinedBuffer);
        std::copy(buffer2, buffer2 + len2, combinedBuffer + len1);

        //Return the combined buffer
        return combinedBuffer;
    }
};


#endif //WARGMAVLINKSUPPORT_BASICMAVLINKENCODER_H
