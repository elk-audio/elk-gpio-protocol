/**
 * @brief Protocol definition for controlling xmos sensors
 */
#ifndef XMOS_CONTROL_PROTOCOL_H_
#define XMOS_CONTROL_PROTOCOL_H_

// Main commands
#define XMOS_CMD_SYSTEM_CNTRL       1
#define XMOS_CMD_CONFIGURE_CNTRLR   100
#define XMOS_CMD_GET_VALUE          101
#define XMOS_CMD_SET_VALUE          102
#define XMOS_ACK                    250

// System sub commands
#define XMOS_SUB_CMD_STOP_RESET_SYSTEM  0
#define XMOS_SUB_CMD_START_SYSTEM       1
#define XMOS_SUB_CMD_STOP_SYSTEM        2
#define XMOS_SUB_CMD_SET_TICK_RATE      3
#define XMOS_SUB_CMD_GET_BOARD_INFO     4

// Configure sub commmands
#define XMOS_SUB_CMD_RESET_ALL_CNTRLRS  0
#define XMOS_SUB_CMD_RESET_CNTRLR       1
#define XMOS_SUB_CMD_ADD_DIGITAL_OUTPUT 2
#define XMOS_SUB_CMD_ADD_DIGITAL_INPUT  3
#define XMOS_SUB_CMD_ADD_ANALOG_INPUT   4
#define XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR 5
#define XMOS_SUB_CMD_MUTE_CNTRLR        6
#define XMOS_SUB_CMD_REMOVE_CNTRLR      7
#define XMOS_SUB_CMD_SET_CNTRLR_RANGE   8

// Base message structure
typedef struct XmosControlPacket
{
    char     command;
    char     sub_command;
    char     continuation;
    char     reserved;
    char     payload[20];
    uint32_t timestamp;
    uint32_t sequence_no;
} XmosControlPacket;

// Payload is on of the following
typedef struct ResetControllerData
{
    char controller_id;
    char reserved[3];
} ResetControllerData;

typedef struct DigitalOutputData
{
    char controller_id;
    char output_hw_type;
    char mux_inverted;
    char mux_controller_id;
    char mux_controller_pin;
    char delta_tick_rate;
    char num_pins;
    char pins[13];
} DigitalOutputData;

typedef struct DigitalInputData
{
    char controller_id;
    char input_hw_type;
    char mux_inverted;
    char mux_controller_id;
    char mux_controller_pin;
    char delta_tick_rate;
    char notification_mode;
    char num_pins;
    char pins[12];
} DigitalInputData;

typedef struct AnalogInputData
{
    char controller_id;
    char pin_number;
} AnalogInputData;

typedef struct ValueData
{
    char controller_id;
    char reserved[3];
    uint32_t value;
} ValueData;


#endif // XMOS_CONTROL_PROTOCOL_H_