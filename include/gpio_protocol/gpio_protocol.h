/**
 * @brief Protocol definition for controlling GPIO sensors
 */
#ifndef GPIO_PROTOCOL_H_
#define GPIO_PROTOCOL_H_

#include "stdint.h"

#ifdef __cplusplus

#include <cassert>
namespace gpio {
#endif

typedef enum GpioCommand
{
    GPIO_CMD_SYSTEM_CONTROL = 1,
    GPIO_CMD_CONFIG_CONTROLLER = 100,
    GPIO_CMD_GET_VALUE = 101,
    GPIO_CMD_SET_VALUE = 102,
    GPIO_ACK = 250
} GpioCommand;

/*==========================================================
=          GPIO_CMD_SYSTEM_CONTROL Sub commands            =
===========================================================*/

typedef enum GpioSystemSubCommand
{
    GPIO_SUB_CMD_STOP_RESET_SYSTEM = 0,
    GPIO_SUB_CMD_START_SYSTEM,
    GPIO_SUB_CMD_STOP_SYSTEM,
    GPIO_SUB_CMD_SET_SYSTEM_TICK_RATE,
    GPIO_SUB_CMD_GET_BOARD_INFO
} GpioSystemSubCommand;

/*----------  GPIO_SUB_CMD_SET_TICK_RATE payload data structure  ----------*/
typedef enum GpioSystemTickRate
{
    GPIO_SYSTEM_TICK_100_HZ = 0,
    GPIO_SYSTEM_TICK_500_HZ,
    GPIO_SYSTEM_TICK_1000_HZ,
    GPIO_SYSTEM_TICK_5000_HZ
} GpioSystemTickRate;

typedef struct SystemTickRateData
{
    uint8_t gpio_system_tick_rate;
} SystemTickRateData;

typedef struct GpioBoardInfoData
{
    uint8_t num_digital_input_pins;
    uint8_t num_digital_output_pins;
    uint8_t num_analog_input_pins;
    uint8_t adc_res_in_bits;
} GpioBoardInfoData;

/*---------------------------------------------------*/

/*==========================================================
=          GPIO_CMD_CONFIG_CONTROLLER Sub commands         =
===========================================================*/

typedef enum GpioConfigSubCommand
{
    GPIO_SUB_CMD_RESET_ALL_CONTROLLERS = 0,
    GPIO_SUB_CMD_RESET_CONTROLLER,
    GPIO_SUB_CMD_ADD_CONTROLLER,
    GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX,
    GPIO_SUB_CMD_SET_CONTROLLER_POLARITY,
    GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE,
    GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE,
    GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER,
    GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER,
    GPIO_SUB_CMD_REMOVE_CONTROLLER,
    GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES,
    GPIO_SUB_CMD_SET_CONTROLLER_RANGE,
    GPIO_SUB_CMD_SET_CONTROLLER_DEBOUNCE_MODE
} GpioConfigSubCommand;

/*----------  GPIO_SUB_CMD_RESET_CONTROLLER payload data structure  ----------*/
typedef struct ResetControllerData
{
    uint8_t controller_id;
} ResetControllerData;

/*----------  GPIO_SUB_CMD_ADD_CONTROLLER payload data structure  ----------*/
typedef enum GpioHwType
{
    GPIO_BINARY_OUTPUT = 0,
    GPIO_BINARY_INPUT,
    GPIO_ANALOG_INPUT,
    GPIO_STEPPED_OUTPUT,
    GPIO_MUX_OUTPUT,
    GPIO_N_WAY_SWITCH,
    GPIO_ROTARY_ENCODER,
    GPIO_AUDIO_MUTE_BUTTON
} GpioHwType;

typedef struct AddControllerData
{
    uint8_t controller_id;
    uint8_t gpio_hw_type;
} AddControllerData;

/*----------  GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX payload data structure  ----------*/
typedef struct ControllerToMuxData
{
    uint8_t controller_id;
    uint8_t mux_controller_id;
    uint8_t mux_controller_pin;
} ControllerToMuxData;

/*----------  GPIO_SUB_CMD_SET_CONTROLLER_POLARITY payload data structure  ----------*/
typedef enum ControllerPolarity
{
    GPIO_ACTIVE_HIGH,
    GPIO_ACTIVE_LOW
} ControllerPolarity;

typedef struct ControllerPolarityData
{
    uint8_t controller_id;
    uint8_t polarity;
} ControllerPolarityData;

/*----------  GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE payload data structure  ----------*/
typedef struct ControllerTickRateData
{
    uint8_t controller_id;
    uint8_t delta_tick_rate;
} ControllerTickRateData;

/*----------  GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE payload data structure  ----------*/
typedef enum ControllerNotifMode
{
    GPIO_ON_VALUE_CHANGE = 0,
    GPIO_EVERY_CONTROLLER_TICK,
    GPIO_WHEN_TOGGLED_ON,    // Only for 1 pin binary input device
    GPIO_WHEN_TOGGLED_OFF    // Only for 1 pin binary input device
} ControllerNotifMode;

typedef struct ControllerNotifData
{
    uint8_t controller_id;
    uint8_t notif_mode;
} ControllerNotifData;

/*----------  GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER payload data structure  ----------*/
typedef struct ControllerPinsData
{
    uint8_t controller_id;
    uint8_t num_pins;
    uint8_t pins[18];
} ControllerPinsData;

/*----------  GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER payload data structure  ----------*/
typedef enum ControllerMuteStatus
{
    GPIO_CONTROLLER_UNMUTED = 0,
    GPIO_CONTROLLER_MUTED
} ControllerMuteStatus;

typedef struct ControllerMuteData
{
    uint8_t controller_id;
    uint8_t mute_status;
} ControllerMuteData;

/*----------  GPIO_SUB_CMD_REMOVE_CONTROLLER payload data structure  ----------*/
typedef struct RemoveControllerData
{
    uint8_t controller_id;
} RemoveControllerData;

/*----------  GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES payload data structure  ----------*/
typedef struct AnalogControllerResData
{
    uint8_t controller_id;
    uint8_t res_in_bits;
} AnalogControllerResData;

/*----------  GPIO_SUB_CMD_SET_CONTROLLER_RANGE payload data structure  ----------*/
typedef struct ControllerRangeData
{
    uint8_t controller_id;
    uint8_t reserved[3];
    uint32_t min_val;
    uint32_t max_val;
} ControllerRangeData;

typedef enum ControllerDebounceMode
{
    GPIO_CONTROLLER_DEBOUNCE_ENABLED,
    GPIO_CONTROLLER_DEBOUNCE_DISABLED,
} ControllerDebounceMode;

typedef struct ControllerDebounceData
{
    uint8_t controller_id;
    uint8_t controller_debounce_mode;
} ControllerDebounceData;

/*=====================================
=          GPIO_ACK layout            =
=======================================*/

typedef enum GpioReturnStatus
{
    GPIO_OK = 0,
    GPIO_ERROR,
    GPIO_INVALID_CMD,
    GPIO_INVALID_SUB_CMD,
    GPIO_NO_CONTROLLERS_ADDED,
    GPIO_UNITIALIZED_CONTROLLERS,
    GPIO_INVALID_TICK_RATE,
    GPIO_INVALID_CONTROLLER_ID,
    GPIO_INVALID_HW_TYPE,
    GPIO_INVALID_MUX_CONTROLLER,
    GPIO_INVALID_CONTROLLER_POLARITY,
    GPIO_NO_PINS_AVAILABLE,
    GPIO_INVALID_SHARING_OF_PINS,
    GPIO_RES_OUT_OF_RANGE,
    GPIO_UNRECOGNIZED_CMD,
    GPIO_PARAMETER_ERROR,
    GPIO_INVALID_COMMAND_FOR_CONTROLLER
} GpioReturnStatus;

typedef struct GpioAckData
{
    uint32_t returned_seq_no;
    uint8_t gpio_return_status;
} GpioAckData;

/*---------------------------------------------------*/

/*======================================================
=     Controller value exchange data structures        =
========================================================*/
/* Master to Slave */
typedef struct GpioValueRequest
{
    uint8_t controller_id;
} GpioValueRequest;

/* Slave to Master */
typedef struct GpioValueData
{
    uint8_t controller_id;
    uint8_t reserved[3];
    uint32_t controller_val;
} GpioValueData;

/*---------------------------------------------------*/

// Payloads are a union of the following
typedef union PayloadData
{
    SystemTickRateData system_tick_rate_data;
    GpioBoardInfoData gpio_board_info_data;

    ResetControllerData reset_controller_data;
    AddControllerData add_controller_data;
    ControllerToMuxData controller_to_mux_data;
    ControllerPolarityData controller_polarity_data;
    ControllerTickRateData controller_tick_rate;
    ControllerNotifData controller_notif_data;
    ControllerPinsData controller_pins_data;
    ControllerMuteData controller_mute_data;
    RemoveControllerData remove_controller_data;
    AnalogControllerResData analog_controller_res_data;
    ControllerRangeData controller_range_data;
    ControllerDebounceData controller_debounce_data;

    GpioValueRequest gpio_value_request;
    GpioValueData gpio_value_data;

    GpioAckData gpio_ack_data;

    uint8_t raw_data[20];
} PayloadData;

// Base message structure
typedef struct GpioPacket
{
    uint8_t     command;
    uint8_t     sub_command;
    uint8_t     continuation;
    uint8_t     reserved;
    PayloadData payload;
    uint32_t    timestamp;
    uint32_t    sequence_no;
} GpioPacket;

#ifdef __cplusplus
static_assert(sizeof(GpioPacket) == 32);
} // end namespace gpio
#endif

#endif // GPIO_PROTOCOL_H_