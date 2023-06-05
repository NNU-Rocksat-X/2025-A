#ifndef COMMON_COMMS_H_
#define COMMON_COMMS_H_

#define NUM_JOINTS (8) // 6 DOF + Gripper


inline unsigned short crc16(const unsigned char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}

typedef struct cmd_packet
{
    // Header 
    uint8_t FRAME_SYNC_LSB;
    uint8_t FRAME_SYNC_MSB;
    uint8_t seq;
    uint8_t led;
    int32_t joint_velocity_cmd[NUM_JOINTS];
    uint16_t crc;
} CMDPacket;

typedef struct response_packet
{
    // Header
    uint8_t seq;
    uint8_t reserved;
    int32_t joint_step_position[NUM_JOINTS];
    uint16_t crc;
} RESPacket;

#endif /* COMMON_COMMS_H_ */