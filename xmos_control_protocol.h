/**
 * @brief Protocol definition for controlling xmos sensors
 */
#ifndef XMOS_CONTROL_PROTOCOL_H_
#define XMOS_CONTROL_PROTOCOL_H_

struct ControlPacket
{
    char data[20];
};


#endif // XMOS_CONTROL_PROTOCOL_H_