/**
 * @brief Protocol definition for controlling xmos sensors
 */
#ifndef XMOS_GPIO_PROTOCOL_H_
#define XMOS_GPIO_PROTOCOL_H_

#include "stdint.h"

typedef enum XmosCommand
{
    XMOS_CMD_SYSTEM_CNTRL     = 1,
    XMOS_CMD_CONFIGURE_CNTRLR = 100,
    XMOS_CMD_GET_VALUE        = 101,
    XMOS_CMD_SET_VALUE        = 102,
    XMOS_ACK                  = 250
} XmosCommand;

/*==========================================================
=          XMOS_CMD_SYSTEM_CNTRL Sub commands             =
===========================================================*/

typedef enum XmosSystemSubCommand
{
    XMOS_SUB_CMD_STOP_RESET_SYSTEM = 0,
    XMOS_SUB_CMD_START_SYSTEM,
    XMOS_SUB_CMD_STOP_SYSTEM,
    XMOS_SUB_CMD_SET_TICK_RATE,
    XMOS_SUB_CMD_GET_BOARD_INFO
} XmosSystemSubCommand;

/*----------  XMOS_SUB_CMD_SET_TICK_RATE payload data structure  ----------*/
typedef enum SystemTickRate
{
    TICK_100_HZ = 0,
    TICK_500_HZ,
    TICK_1000_HZ,
    TICK_5000_HZ
} SystemTickRate;

typedef struct TickRateData
{
    uint8_t system_tick_rate;
} TickRateData;

/*----------  XMOS_SUB_CMD_GET_BOARD_INFO payload data structure  ----------*/
typedef struct BoardInfoData
{
    uint8_t num_digital_input_pins;
    uint8_t num_digital_output_pins;
    uint8_t num_analog_pins;
    uint8_t reserved;
    uint32_t adc_resolution;
} BoardInfoData;

/*---------------------------------------------------*/

/*==========================================================
=          XMOS_CMD_CONFIGURE_CNTRLR Sub commands             =
===========================================================*/

typedef enum XmosConfigSubCommand
{
    XMOS_SUB_CMD_RESET_ALL_CNTRLRS = 0,
    XMOS_SUB_CMD_RESET_CNTRLR,
    XMOS_SUB_CMD_ADD_CNTRLR,
    XMOS_SUB_CMD_ADD_CNTRLR_TO_MUX,
    XMOS_SUB_CMD_SET_CNTRLR_POLARITY,
    XMOS_SUB_CMD_SET_INPUT_CNTRLR_TICK_RATE,
    XMOS_SUB_CMD_SET_INPUT_CNTRLR_NOTIF_MODE,
    XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR,
    XMOS_SUB_CMD_MUTE_UNMUTE_CNTRLR,
    XMOS_SUB_CMD_REMOVE_CNTRLR,
    XMOS_SUB_CMD_SET_CNTRLR_RANGE
} XmosConfigSubCommand;

/*----------  XMOS_SUB_CMD_RESET_CNTRLR payload data structure  ----------*/
typedef struct ResetCntrlrData
{
    uint8_t controller_id;
} ResetCntrlrData;

/*----------  XMOS_SUB_CMD_ADD_CNTRLR payload data structure  ----------*/
typedef enum HwType
{
    BINARY_OUTPUT = 0,
    BINARY_INPUT,
    ANALOG_INPUT,
    STEPPED_OUTPUT,
    MUX_OUTPUT,
    N_WAY_SWITCH,
    ROTARY_ENCODER,
    TAP_BUTTON
} HwType;

typedef struct CntrlrData
{
    uint8_t controller_id;
    uint8_t hw_type;
} AddCntrlrData;

/*----------  XMOS_SUB_CMD_ADD_CNTRLR_TO_MUX payload data structure  ----------*/
typedef struct ControllerToMuxData
{
    uint8_t controller_id;
    uint8_t mux_controller_id;
    uint8_t mux_controller_pin;
} ControllerToMuxData;

/*----------  XMOS_SUB_CMD_SET_CNTRLR_POLARITY payload data structure  ----------*/

typedef enum CntrlrPolarity
{
    ACTIVE_HIGH,
    ACTIVE_LOW
} CntrlrPolarity;

typedef struct CntrlrPolarityData
{
    uint8_t controller_id;
    uint8_t polarity;
} CntrlrPolarityData;

/*----------  XMOS_SUB_CMD_SET_INPUT_CNTRLR_TICK_RATE payload data structure  ----------*/
typedef struct CntrlrTickRateData
{
    uint8_t controller_id;
    uint8_t delta_tick_rate;
} CntrlrTickRateData;

/*----------  XMOS_SUB_CMD_SET_INPUT_CNTRLR_NOTIF_MODE payload data structure  ----------*/
typedef enum NotificationMode
{
    ON_VALUE_CHANGE = 0,
    EVERY_CNTRLR_TICK,
    WHEN_TOGGLED,
    WHEN_TOGGLED_ON,
    WHEN_TOGGLED_OFF
} NotificationMode;

typedef struct NotificationModeData
{
    uint8_t controller_id;
    uint8_t notif_mode;
} NotificationModeData;

/*----------  XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR payload data structure  ----------*/
typedef struct PinsData
{
    uint8_t controller_id;
    uint8_t num_pins;
    uint8_t pins[18];
} PinsData;

/*----------  XMOS_SUB_CMD_MUTE_UNMUTE_CNTRLR payload data structure  ----------*/
typedef enum MuteStatus
{
    UNMUTED = 0,
    MUTED
} MuteStatus;

typedef struct MuteCommandData
{
    uint8_t controller_id;
    uint8_t mute_status;
} MuteCommandData;

/*----------  XMOS_SUB_CMD_REMOVE_CNTRLR payload data structure  ----------*/
typedef struct RemoveCntrlrData
{
    uint8_t controller_id;
} RemoveCntrlrData;

/*----------  XMOS_SUB_CMD_SET_CNTRLR_RANGE payload data structure  ----------*/
typedef struct SetCntrlrRangeData
{
    uint8_t controller_id;
    uint8_t reserved[3];
    uint32_t min_value;
    uint32_t max_value;  
} SetCntrlrRangeData;

/*---------------------------------------------------*/

/*======================================================
=          Raspa and XMOS value data structures        =
========================================================*/

//Raspa -> XMOS
typedef struct ValueRequest
{
    uint8_t controller_id;
} ValueRequest;

// XMOS -> Raspa
typedef struct ValueSend
{
    uint8_t controller_id;
    uint8_t reserved[3];
    uint32_t controller_val;
} ValueSend;

/*---------------------------------------------------*/

// Payloads are a union of the following
typedef union PayloadData
{
    TickRateData tick_rate_data;
    BoardInfoData board_info_data;

    ResetCntrlrData reset_cntrlr_data;
    CntrlrData cntrlr_data;
    ControllerToMuxData cntrlr_to_mux_data;
    CntrlrPolarityData cntrlr_polarity_data;
    CntrlrTickRateData cntrlr_tick_rate;
    NotificationModeData notif_mode_data;
    PinsData pin_data;
    MuteCommandData mute_cmnd_data;
    RemoveCntrlrData remove_cntrlr_data;
    SetCntrlrRangeData set_cntrlr_range_data;

    ValueRequest value_request_data;
    ValueSend value_send_data;

    uint8_t raw_data[20];
} PayloadData;


// Base message structure
typedef struct XmosGpioPacket
{
    uint8_t     command;
    uint8_t     sub_command;
    uint8_t     continuation;
    uint8_t     reserved;
    PayloadData payload;
    uint32_t    timestamp;
    uint32_t    sequence_no;
} XmosGpioPacket;

#endif // XMOS_GPIO_PROTOCOL_H_