#ifndef IMUPACKET_H
#define IMUPACKET_H


#pragma pack(push,1)
typedef struct
{
    uint8_t sync[2];
    uint8_t pkt_class;
    uint8_t pkt_id;
    uint16_t length;
    uint32_t time;
    int16_t temp;
    int16_t imu[6];
    uint16_t csum;
} imuPacket_t;
#pragma pack(pop)

#define CLASS_LOG 0x50
#define ID_ICM42688 0xe3

#endif // IMUPACKET_H
