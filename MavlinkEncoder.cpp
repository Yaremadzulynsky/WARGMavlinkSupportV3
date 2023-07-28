//
// Created by Yarema Dzulynsky on 2023-06-29.
//
//#define REGISTER_ENCODER(msgId, baseName, ...) encodingFunctions[msgId] = [&](mavlink_message_t &msg, auto... args) { \
////    mavlink_ ## baseName ## _t msgType;                         \
////    mavlink_msg_ ## baseName ## _init(&msgType);                \
//    mavlink_msg_ ## baseName ## _pack(1, 200, &msg, args...); \
//};

#include "MavlinkEncoder.h"
#define REGISTER_ENCODER(msgId, baseName, ...) \
    encodingFunctions[msgId] = [&](mavlink_message_t& msg, auto... args) { \
        mavlink_ ## baseName ## _t msgType; \
        auto argsTuple = std::make_tuple(args...); \
        unpackAndCall( \
            [&](auto&&... unpackedArgs) { \
                mavlink_msg_ ## baseName ## _pack(1, 200, &msg, 0, std::forward<decltype(unpackedArgs)>(unpackedArgs)...); \
            }, \
            argsTuple \
        ); \
    };


template<typename Func, typename Container, std::size_t... Indices>
void unpackAndCallHelper(Func &&func, const Container &container, std::index_sequence<Indices...>) {
    std::apply(std::forward<Func>(func), std::make_tuple(container[Indices]...));
}

// Helper function to unpack the arguments and call the target function
template<typename Func, typename... Args>
void unpackAndCall(Func &&func, const std::tuple<Args...> &args) {
    std::apply(std::forward<Func>(func), args);
}


MavlinkEncoder::MavlinkEncoder() {
//    REGISTER_ENCODER(MAVLINK_MSG_ID_ATTITUDE, attitude, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);

//encodingFunctions[MAVLINK_MSG_ID_ATTITUDE] = [&](mavlink_message_t &msg, auto... args) { \
//    mavlink_attitude_t msgType;
//    mavlink_msg_attitude_init(&msgType);
//    mavlink_msg_attitude_pack(1, 200, &msg, args...);

    encodingFunctions[MAVLINK_MSG_ID_ATTITUDE] = [&](mavlink_message_t &msg, auto... args) {
       /* auto argsTuple = std::make_tuple(args...);
        unpackAndCall(
                [&](auto &&... unpackedArgs) {
                    //sig uint8_t system_id, uint8_t component_id, __mavlink_message *msg, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed

                    mavlink_msg_attitude_pack(1, 200, msg, 0, std::forward<decltype(unpackedArgs)>(unpackedArgs)...);

                }, argsTuple);*/
    };

    encodingFunctions[MAVLINK_MSG_ID_ATTITUDE](currentMessage, 1, 2, 3, 4, 5, 6);



//    REGISTER_ENCODER(MAVLINK_MSG_ID_ATTITUDE, attitude, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);




    // Handle the packed message or perform any additional operations

    // Call the lambda function explicitly with the desired arguments
    mavlink_message_t message;
    float roll = 1.0f;
    float pitch = 2.0f;
    float yaw = 3.0f;
    float rollspeed = 4.0f;
    float pitchspeed = 5.0f;
    float yawspeed = 6.0f;

//    encodingFunctions[MAVLINK_MSG_ID_ATTITUDE](roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, 5);
//    encodingFunctions[MAVLINK_MSG_ID_ATTITUDE](, 1, 2, 3, 4, 5, 6);
}


uint8_t *
MavlinkEncoder::encodeGlobalPositionInt(mavlink_message_t &msg, std::size_t &len, uint8_t *buffer, int velocity,
                                        int heading, int latitude,
                                        int longitude,
                                        int altitude, int relativeAltitude) {
    mavlink_global_position_int_t globalPositionInt;
    globalPositionInt.time_boot_ms = 0;
    globalPositionInt.lat = latitude;
    globalPositionInt.lon = longitude;
    globalPositionInt.alt = altitude;
    globalPositionInt.relative_alt = relativeAltitude;
    globalPositionInt.vx = velocity;
    globalPositionInt.vy = velocity;
    globalPositionInt.vz = velocity;
    globalPositionInt.hdg = heading;

    mavlink_msg_global_position_int_encode(1, 1, &msg, &globalPositionInt);
    //make a variable buffer that holds the msg in bytes

    len = msgToBuffer(msg, buffer);

    return buffer;
}

uint16_t MavlinkEncoder::msgToBuffer(mavlink_message_t &msg, uint8_t *buffer) {

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return len;

}

uint8_t *
MavlinkEncoder::encodeAttitude(mavlink_message_t &msg, std::size_t &len, uint8_t *buffer, float roll, float pitch,
                               float yaw,
                               float rollSpeed,
                               float pitchSpeed, float yawSpeed) {
    mavlink_attitude_t attitude;
    attitude.time_boot_ms = 0;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    attitude.rollspeed = rollSpeed;
    attitude.pitchspeed = pitchSpeed;
    attitude.yawspeed = yawSpeed;

    mavlink_msg_attitude_encode(1, 1, &msg, &attitude);

    len = msgToBuffer(msg, buffer);

    return buffer;
}

uint8_t *MavlinkEncoder::combineBuffers(uint8_t *buffer1, uint8_t *buffer2, size_t len1, size_t len2) {
    auto *combinedBuffer = new uint8_t[len1 + len2];

    std::copy(buffer1, buffer1 + len1, combinedBuffer);
    std::copy(buffer2, buffer2 + len2, combinedBuffer + len1);
    return combinedBuffer;
}
