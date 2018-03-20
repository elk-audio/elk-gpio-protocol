/**
 * @brief Protocol definition for controlling xmos sensors
 */
#ifndef XMOS_CONTROL_PROTOCOL_H_
#define XMOS_CONTROL_PROTOCOL_H_

#include "stdint.h"

enum XmosCommand
{
    XMOS_CMD_SYSTEM_CNTRL     = 1,
    XMOS_CMD_CONFIGURE_CNTRLR = 100,
    XMOS_CMD_GET_VALUE        = 101,
    XMOS_CMD_SET_VALUE        = 102,
    XMOS_ACK                  = 250
};

// System sub commands
enum XmosSystemSubCommand
{
    XMOS_SUB_CMD_STOP_RESET_SYSTEM = 0,
    XMOS_SUB_CMD_START_SYSTEM,
    XMOS_SUB_CMD_STOP_SYSTEM,
    XMOS_SUB_CMD_SET_TICK_RATE,
    XMOS_SUB_CMD_GET_BOARD_INFO
};

// Configure sub commmands
enum XmosConfigSubCommand
{
    XMOS_SUB_CMD_RESET_ALL_CNTRLRS  = 0,
    XMOS_SUB_CMD_RESET_CNTRLR,
    XMOS_SUB_CMD_ADD_DIGITAL_OUTPUT,
    XMOS_SUB_CMD_ADD_DIGITAL_INPUT,
    XMOS_SUB_CMD_ADD_ANALOG_INPUT,
    XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR,
    XMOS_SUB_CMD_MUTE_CNTRLR,
    XMOS_SUB_CMD_REMOVE_CNTRLR,
    XMOS_SUB_CMD_SET_CNTRLR_RANGE
};

enum MuxInvertMode
{
    NOT_MUXED_ACTIVE_HIGH = 0,
    NOT_MUXED_ACTIVE_LOW,
    MUXED_ACTIVE_HIGH,
    MUXED_ACTIVE_LOW
};

enum NotificationMode
{
    ON_VALUE_CHANGE = 0,
    EVERY_SYSTEM_TICK,
    EVERY_CNTRLR_TICK,
    WHEN_TOGGLED,
    WHEN_TOGGLED_ON,
    WHEN_TOGGLED_OFF
};

enum InputHwType
{
    BINARY_INPUT = 0,
    N_WAY_SWITCH,
    ROTARY_ENCODER,
    TAP_BUTTON
};

enum OutputHwType
{
    BINARY_OUTPUT = 0,
    STEP_FROM_BOTTOM,
    STEP_FROM_UP,
};

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

typedef struct PinData
{
    char controller_id;
    char num_pins;
    char pins[18];
} PinData;

typedef struct ValueData
{
    char controller_id;
    char reserved[3];
    uint32_t value;
} ValueData;


#endif // XMOS_CONTROL_PROTOCOL_H_