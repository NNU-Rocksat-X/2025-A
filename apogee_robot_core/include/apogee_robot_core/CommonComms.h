#ifndef COMMON_COMMS_H_
#define COMMON_COMMS_H_

#define NUM_JOINTS (7) // 6 DOF + Gripper


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

    float joint_velocity_cmd[NUM_JOINTS];

    // Header 
    // (which is placed last because of struct packing and so the crc is last)
    uint8_t seq;
    uint8_t reserved;
    uint16_t crc;

} CMDPacket;

typedef struct response_packet
{
    uint32_t joint_step_position[NUM_JOINTS]; // will be uint32_t

    // Header
    uint8_t seq;
    uint8_t reserved;
    uint16_t crc;
} RESPacket;

#endif /* COMMON_COMMS_H_ */