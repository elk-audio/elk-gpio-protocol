/**
 * @brief Contains the definition and behavior logic of the gpio protocol client.
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
 *
 * Defines the gpio client library which contains the logic and behaviour to
 * control gpio controllers. This can run on multiple architectures which follow
 * the gpio protocol specification. It is meant to be used in any platform where
 * gpio protocol logic is needed and interacts.
 */
#ifndef GPIO_CLIENT_H_
#define GPIO_CLIENT_H_

#include <cstdint>
#include <tuple>
#include <array>
#include <chrono>

#include "gpio_protocol/gpio_protocol.h"
#include "ctrlr_set.h"
#include "gpio_logger_interface.h"
#include "gpio_fifo.h"

namespace gpio {

// Default system task period and system tick rate
constexpr int DEFAULT_TASK_PERIOD_NS = 1000000; // 1KHz
constexpr int DEFAULT_SYSTEM_TICK_RATE = 1000; // 1KHz

/**
 * @brief Cross platform and architecture gpio protocol client which handles all
 *        the logic necessary for a gpio protocol based client and the physical
 *        controllers connected on a target board. It can be configured for any
 *        combination of the number of input, output and analog pins. It does
 *        not use any kind of dynamic allocation and is real time safe.
 *        It is responsible for:
 *        for:
 *        -> Handling of gpio protocol packets sent by the host(sensei) and
 *           making the respective changes on the physical controller behavior.
 *        -> Processing of all the various controllers as specified in the gpio
 *           protocol hw types.
 *        -> Manage the handling of packets sent to the host(sensei) which are generated
 *           by controllers notifying the host of new controller values or as
 *           responses to the the packets sent by the host(sensei).
 *        -> Its a state machine and can be reset to initial state. All previous
 *           config can be reset. This is useful for running it on micro-controllers.
 *        -> All packets sent to the host are timestamped to the system tick rate.
 *        -> Handling of the log msg fifo and the packet fifo.
 *
 * @tparam NumInputPins  The number of Digital Input pins of the system. Can be
 *                        zero.
 * @tparam NumOutputPins The number of Digital Output pins of the system. Can be
 *                       zero.
 * @tparam NumAnalogPins The number of Analog Input pins of the system. Can be
 *                       zero.
 * @tparam AdcRes        The resolution of the adc. Should be non zero if
 *                        NumAnalogPins is non zero.
 */
template<uint32_t NumInputPins,
        uint32_t NumOutputPins,
        uint32_t NumAnalogPins,
        uint32_t AdcRes>
class GpioClient
{
public:

    /**
     * @brief Initializes all the internal data and ctrlrs and checks the
     *        validity of the pin data memory. Also checks that the arguments
     *        match with that of the template parameters. Should be run first
     *        after instantiation.
     * @param input_pin_data     The pointer to the data containing the input
     *                           pin data. Can be nullptr if there are no
     *                           input pins.
     * @param output_pin_data    The pointer to the data containing the output
     *                           pin data.Can be nullptr if there are no output
     *                           pins.
     * @param analog_pin_data    The pointer to the data containing the analog
     *                           pin data. Can be nullptr if there are no analog
     *                           pins.
     * @param adc_chans_per_tick The number of adc channels (or pins) sampled
     *                           per tick. This is for supporting platforms
     *                           where all adc channels cannot be sampled in one
     *                           system tick. Can be 0 if there are no analog
     *                           pins.
     *
     * @return true              All parameters are valid and have been
     *                           initialized.
     * @return false             Error in initialization. Check the log msgs for
     *                           details.
     */
    inline bool init(uint32_t* input_pin_data,
                     uint32_t* output_pin_data,
                     uint32_t* analog_pin_data,
                     int adc_chans_per_tick)
    {
        if (NumInputPins > 0 && input_pin_data == nullptr)
        {
            GPIO_LOG_ERROR("Cannot init gpio client, invalid input pin data");
            return false;
        }

        if (NumOutputPins > 0 && output_pin_data == nullptr)
        {
            GPIO_LOG_ERROR("Cannot init gpio client, invalid output pin data");
            return false;
        }

        if (NumAnalogPins > 0)
        {
            static_assert(AdcRes != 0);

            if (analog_pin_data == nullptr)
            {
                GPIO_LOG_ERROR(
                        "Cannot init gpio client, invalid analog pin data");
                return false;
            }

            if (adc_chans_per_tick == 0)
            {
                GPIO_LOG_ERROR("Cannot init gpio client, cannot accept %d" \
                " adc_chans_per_tick", adc_chans_per_tick);
                return false;
            }
        }

        _output_pin_data = output_pin_data;
        _digital_outputs.set_info(output_pin_data, &_current_system_tick);

        _input_pin_data = input_pin_data;
        _digital_inputs.set_info(input_pin_data, &_current_system_tick,
                                 &_tx_packet_fifo);

        _analog_pin_data = analog_pin_data;
        _adc_chans_per_tick = adc_chans_per_tick;
        _analog_inputs.set_info(analog_pin_data, &_current_system_tick,
                                &_tx_packet_fifo, adc_chans_per_tick);

        return true;
    }

    /**
     * @brief Reset the gpio client to its initial state. This will clear all
     *        the pin data, remove any previously configured controllers and
     *        reset the log msg fifo and the packet fifos.
     */
    inline void reset()
    {
        _current_system_tick = 0;
        _is_running = false;

        _system_tick_rate = DEFAULT_SYSTEM_TICK_RATE;
        _system_tick_period_ns = DEFAULT_TASK_PERIOD_NS;

        for (int i = 0; i < NumInputPins; i++)
        {
            _input_pin_data[i] = 0;
        }

        for (int i = 0; i < NumOutputPins; i++)
        {
            _output_pin_data[i] = 0;
        }

        for (int i = 0; i < NumAnalogPins; i++)
        {
            _analog_pin_data[i] = 0;
        }

        _digital_inputs.init();
        _digital_outputs.init();
        _analog_inputs.init();
        _analog_inputs.calc_and_set_adc_tick_rate(_system_tick_rate,
                                                  _adc_chans_per_tick);

        _tx_packet_fifo.reset();
        GPIO_LOG_RESET;
    }

    /**
     * @brief Get the current system tick period in nanosecs.
     * @return The tick period in nanoseconds.
     */
    inline int get_tick_period_ns()
    {
        return _system_tick_period_ns;
    }

    /**
     * @brief Get the running status if the client,
     * @return true If the client is already configured and running, false if not
     */
    inline bool is_running()
    {
        return _is_running;
    }

    /**
     * @brief Handle a gpio protocol packet sent by the host.
     * @param rx_packet
     */
    inline void handle_rx_packet(const gpio::GpioPacket &rx_packet)
    {
        switch (rx_packet.command)
        {
        case GPIO_CMD_SYSTEM_CONTROL:
            GPIO_LOG_INFO("Received system ctrl cmd; seq = %d",
                          rx_packet.sequence_no);
            _handle_system_ctrl_cmd(rx_packet);
            break;

        case GPIO_CMD_CONFIG_CONTROLLER:
            GPIO_LOG_INFO("Received config ctrl cmd; seq = %d",
                          rx_packet.sequence_no);
            _handle_config_ctrlr_cmd(rx_packet);
            break;

        case GPIO_CMD_GET_VALUE:
            GPIO_LOG_INFO("Received get val cmd; seq = %d",
                          rx_packet.sequence_no);
            _handle_get_val_cmd(rx_packet);
            break;

        case GPIO_CMD_SET_VALUE:
            GPIO_LOG_INFO("Received set val cmd; seq = %d",
                          rx_packet.sequence_no);
            _handle_set_val_cmd(rx_packet);
            break;

        default:
            GPIO_LOG_ERROR("Unknown cmd; seq = %d", rx_packet.sequence_no);
            _tx_packet_fifo.send_ack(GPIO_INVALID_CMD, _current_system_tick,
                                     rx_packet.sequence_no);
            break;
        }
    }

    inline void clear_packet(GpioPacket &packet)
    {
        _tx_packet_fifo.clear_packet(packet);
    }

    void process()
    {
        if (!_is_running)
        {
            return;
        }

        _digital_inputs.process();
        _digital_outputs.process();
        _analog_inputs.process();

        _current_system_tick++;
    }

    inline bool has_new_tx_packet()
    {
        return _tx_packet_fifo.has_new_elements();
    }

    inline GpioPacket* get_next_tx_packet()
    {
        return _tx_packet_fifo.get_packet();
    }

    inline bool has_new_log_msg()
    {
        return GPIO_LOG_HAS_NEW_MSG;
    }

    inline GpioLogMsg* get_log_msg()
    {
        return GPIO_LOG_GET_MSG;
    }

private:
    inline GpioReturnStatus _start()
    {
        if (_is_running)
        {
            GPIO_LOG_ERROR("Cannot start. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        // check if atleast one ctrlr is added
        if (_digital_inputs.get_num_ctrlrs() == 0 &&
            _digital_outputs.get_num_ctrlrs() == 0 &&
            _analog_inputs.get_num_ctrlrs() == 0)
        {
            GPIO_LOG_ERROR("Cannot start. No ctrlrs were added");
            return GPIO_NO_CONTROLLERS_ADDED;
        }

        if (!_digital_inputs.check_ctrlr_init_status() ||
            !_digital_outputs.check_ctrlr_init_status() ||
            !_analog_inputs.check_ctrlr_init_status())
        {
            GPIO_LOG_ERROR("Cannot start. One or more ctrlrs were" \
                           " not initialized properly");
            return GPIO_UNITIALIZED_CONTROLLERS;
        }

        if (_digital_inputs.check_duplicated_pins() ||
            _digital_outputs.check_duplicated_pins() ||
            _analog_inputs.check_duplicated_pins())
        {
            GPIO_LOG_ERROR("Cannot start. One or more ctrlrs are" \
                            " using the same pins");
            return GPIO_INVALID_SHARING_OF_PINS;
        }

        _is_running = true;
        _current_system_tick = 0;

        GPIO_LOG_INFO("Client is started");
        return GPIO_OK;
    }

    inline GpioReturnStatus
    _set_system_tick_rate(GpioSystemTickRate system_tick_rate)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot set system tick rate. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        switch (system_tick_rate)
        {
        case GPIO_SYSTEM_TICK_100_HZ:
            _system_tick_rate = 100; // 10 ms
            _system_tick_period_ns = 10000000;
            break;

        case GPIO_SYSTEM_TICK_500_HZ:
            _system_tick_rate = 500; // 2 ms
            _system_tick_period_ns = 2000000;
            break;

        case GPIO_SYSTEM_TICK_1000_HZ:
            _system_tick_rate = 1000; // 1 ms
            _system_tick_period_ns = 1000000;
            break;

        case GPIO_SYSTEM_TICK_5000_HZ:
            _system_tick_rate = 5000; //200 us
            _system_tick_period_ns = 200000;
            break;

        default:
            GPIO_LOG_ERROR("Cannot set system tick rate. Invalid tick rate %d",
                           system_tick_rate);
            return GPIO_INVALID_TICK_RATE;
            break;
        }

        _analog_inputs.calc_and_set_adc_tick_rate(_system_tick_rate,
                                                  _adc_chans_per_tick);

        GPIO_LOG_INFO("System Tick rate set to %d Hz", _system_tick_rate);
        return GPIO_OK; // to supress warning
    }

    inline void _reset_all_ctrlrs()
    {
        _digital_inputs.reset_all_ctrlrs();
        _digital_outputs.reset_all_ctrlrs();
        _analog_inputs.reset_all_ctrlrs();

        GPIO_LOG_INFO("All ctrlrs are reset to initial value");
    }

    inline GpioReturnStatus _reset_ctrlr(int id)
    {
        if (_digital_inputs.reset_ctrlr(id) ||
            _digital_outputs.reset_ctrlr(id) ||
            _analog_inputs.reset_ctrlr(id))
        {
            GPIO_LOG_INFO("ctrlr id %d is reset to initial value", id);
            return GPIO_OK;
        }

        GPIO_LOG_ERROR("Cannot reset ctrlr,Invalid id %d", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _add_ctrlr(int id, GpioHwType type)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot add new ctrlr. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        if (_digital_inputs.does_id_exist(id) ||
            _digital_outputs.does_id_exist(id) ||
            _analog_inputs.does_id_exist(id))
        {
            GPIO_LOG_ERROR("Cannot add new ctrlr. id %d already exists", id);
            return GPIO_INVALID_CONTROLLER_ID;
        }

        switch (type)
        {
        case GPIO_BINARY_INPUT:
        case GPIO_N_WAY_SWITCH:
        case GPIO_ROTARY_ENCODER:
            return _digital_inputs.add_ctrlr(id, type);
            break;

        case GPIO_BINARY_OUTPUT:
        case GPIO_STEPPED_OUTPUT:
        case GPIO_MUX_OUTPUT:
            return _digital_outputs.add_ctrlr(id, type);
            break;

        case GPIO_ANALOG_INPUT:
            return _analog_inputs.add_ctrlr(id);
            break;

        default:
            GPIO_LOG_WARNING("Cannot add new ctrlr. Invalid hw type");
            return GPIO_INVALID_HW_TYPE;
            break;
        }

        // to suppress warnings
        return GPIO_OK;
    }

    inline GpioReturnStatus _attach_ctrlr_to_mux(int ctrlr_id,
                                                 int mux_id,
                                                 uint32_t mux_pin)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot attach ctrlr to mux. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        auto mux_ctrlr = static_cast<MuxCtrlrInterface*>(_digital_outputs.get_ctrlr(
                mux_id));
        if (mux_ctrlr == nullptr)
        {
            GPIO_LOG_ERROR(
                    "Cannot attach ctrlr to mux. Mux id %d does not exists",
                    mux_id);
            return GPIO_INVALID_MUX_CONTROLLER;
        }

        auto status = _digital_inputs.attach_to_mux(ctrlr_id, mux_pin,
                                                    mux_ctrlr);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _digital_outputs.attach_to_mux(ctrlr_id, mux_pin, mux_ctrlr);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_analog_inputs.does_id_exist(ctrlr_id))
        {
            GPIO_LOG_ERROR(
                    "Cannot attach ctrlr to mux. id %d is an analog ctrlr",
                    ctrlr_id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot attach ctrlr to mux. Id %d does not exists",
                       ctrlr_id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_ctrlr_pol(int id, ControllerPolarity pol)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr polarity. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        auto status = _digital_inputs.set_pol(id, pol);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _digital_outputs.set_pol(id, pol);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_analog_inputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr polarity. id %d is an analog ctrlr", id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot set ctrlr polarity. Id %d not found", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_ctrlr_tick_rate(int id, int tick_rate)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr tick rate. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        if (tick_rate == 0)
        {
            GPIO_LOG_ERROR("Cannot set ctrlr tick rate. Tick rate = 0");
            return GPIO_INVALID_TICK_RATE;
        }

        auto status = _digital_inputs.set_tick_rate(id, tick_rate);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _analog_inputs.set_tick_rate(id, tick_rate);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_digital_outputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr tick rate. Id %d is an Output Ctrlr", id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot set ctrlr tick rate. Invalid id %d\n", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_ctrlr_notif_mode(int id,
                                                  ControllerNotifMode notif_mode)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr notif mode. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        auto status = _digital_inputs.set_notif_mode(id, notif_mode);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _analog_inputs.set_notif_mode(id, notif_mode);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_digital_outputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr notif mode. Id %d is a digital output",
                    id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot set ctrlr notif mode. Invalid id %d\n", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _add_pins_to_ctrlr(int id,
                                               int num_pins,
                                               const uint8_t* const pin_list)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot add pins to ctrlr. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        auto status = _digital_inputs.add_pins(id, num_pins, pin_list);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _digital_outputs.add_pins(id, num_pins, pin_list);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _analog_inputs.add_pins(id, num_pins, pin_list);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        GPIO_LOG_ERROR("Cannot add pins. Invalid id %d", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    // mux cannot be muted
    inline GpioReturnStatus _set_ctrlr_mute_status(int id,
                                                   ControllerMuteStatus mute_status)
    {
        auto status = _digital_inputs.set_mute_status(id, mute_status);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _digital_outputs.set_mute_status(id, mute_status);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        status = _analog_inputs.set_mute_status(id, mute_status);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        GPIO_LOG_ERROR("Cannot set mute status. Invalid id %d", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_analog_input_res(int id, uint32_t res_in_bits)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot set analog input res. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        if (res_in_bits > AdcRes || res_in_bits == 0)
        {
            GPIO_LOG_ERROR("Cannot set analog input res. Invalid res %d",
                           res_in_bits);
            return GPIO_RES_OUT_OF_RANGE;
        }

        auto status = _analog_inputs.set_res_diff(id, (AdcRes - res_in_bits));
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_digital_outputs.does_id_exist(id) ||
            _digital_inputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING("Cannot set analog res for non analog ctrlr id %d",
                             id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot set analog input res. Invalid id %d", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_ctrlr_range(int id,
                                             uint32_t min_val,
                                             uint32_t max_val)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING(
                    "Cannot set ctrlr range. Client has already started");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        if (max_val <= min_val)
        {
            GPIO_LOG_ERROR("Cannot set ctrlr range. Min val >= Max val");
            return GPIO_PARAMETER_ERROR;
        }

        auto status = _digital_inputs.set_range(id, min_val, max_val);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_digital_outputs.does_id_exist(id) ||
            _analog_inputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING("Cannot set range for non input ctrlr id %d", id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot set range. Invalid id %d", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_ctrlr_debounce_mode(int id,
                                                     ControllerDebounceMode debounce_mode)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING("Cannot set debounce mode, Client is running");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        auto status = _digital_inputs.set_debounce_mode(id, debounce_mode);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_digital_outputs.does_id_exist(id) ||
            _analog_inputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING("Cannot set debounce for non input ctrlr id %d",
                             id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        GPIO_LOG_ERROR("Cannot set debounce. Invalid id %d", id);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline GpioReturnStatus _set_time_constant(int id, float time_constant)
    {
        if (_is_running)
        {
            GPIO_LOG_WARNING("Cannot set debounce mode, Client is running");
            return GPIO_INVALID_RUNTIME_CONFIG;
        }

        auto status = _analog_inputs.set_time_constant(id, time_constant);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            return status;
        }

        if (_digital_outputs.does_id_exist(id) ||
            _digital_inputs.does_id_exist(id))
        {
            GPIO_LOG_WARNING(
                    "Cannot set filter time constant non analog ctrlr id %d",
                    id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        return GPIO_INVALID_CONTROLLER_ID;
    }

    inline void _handle_system_ctrl_cmd(const GpioPacket &packet)
    {
        switch (packet.sub_command)
        {
        case GPIO_SUB_CMD_STOP_RESET_SYSTEM:
            GPIO_LOG_INFO("Got a stop reset cmd, resetting..");
            reset();
            _tx_packet_fifo.send_ack(GPIO_OK, _current_system_tick,
                                     packet.sequence_no);
            break;

        case GPIO_SUB_CMD_START_SYSTEM:
            GPIO_LOG_INFO("Got a start cmd");
            _tx_packet_fifo.send_ack(_start(), _current_system_tick,
                                     packet.sequence_no);
            break;

        case GPIO_SUB_CMD_SET_SYSTEM_TICK_RATE:
            GPIO_LOG_INFO("Got a set tick rate cmd");
            {
                const auto &new_tick_rate_hz =
                        packet.payload.system_tick_rate_data.gpio_system_tick_rate;
                auto status = _set_system_tick_rate(
                        static_cast<GpioSystemTickRate>(new_tick_rate_hz));
                _tx_packet_fifo.send_ack(status, _current_system_tick,
                                         packet.sequence_no);
            }
            break;

        case GPIO_SUB_CMD_GET_BOARD_INFO:
            GPIO_LOG_INFO("Got a board info cmd");
            _tx_packet_fifo.send_ack(GPIO_OK, _current_system_tick,
                                     packet.sequence_no);
            _tx_packet_fifo.send_board_info(_current_system_tick,
                                            NumInputPins,
                                            NumOutputPins,
                                            NumAnalogPins,
                                            AdcRes);
            break;

        default:
            GPIO_LOG_WARNING("Invalid command by packet %d",
                             packet.sequence_no);
            _tx_packet_fifo.send_ack(GPIO_INVALID_CMD, _current_system_tick,
                                     packet.sequence_no);
        }
    }

    inline void _handle_config_ctrlr_cmd(const GpioPacket &packet)
    {
        GpioReturnStatus status;
        switch (packet.sub_command)
        {
        case GPIO_SUB_CMD_RESET_ALL_CONTROLLERS:
            GPIO_LOG_INFO("Got a reset all ctrlrs cmd");
            _reset_all_ctrlrs();
            status = GPIO_OK;
            break;

        case GPIO_SUB_CMD_RESET_CONTROLLER:
            GPIO_LOG_INFO("Got a reset ctrlr cmd");
            status = _reset_ctrlr(
                    packet.payload.reset_controller_data.controller_id);
            break;

        case GPIO_SUB_CMD_ADD_CONTROLLER:
            GPIO_LOG_INFO("Got an add ctrlr cmd");
            status = _add_ctrlr(
                    packet.payload.add_controller_data.controller_id,
                    static_cast<GpioHwType>(packet.payload.add_controller_data.gpio_hw_type));
            break;

        case GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX:
            GPIO_LOG_INFO("Got an attach ctrlr to mux cmd");
            status = _attach_ctrlr_to_mux(
                    packet.payload.controller_to_mux_data.controller_id,
                    packet.payload.controller_to_mux_data.mux_controller_id,
                    packet.payload.controller_to_mux_data.mux_controller_pin);
            break;

        case GPIO_SUB_CMD_SET_CONTROLLER_POLARITY:
            GPIO_LOG_INFO("Got a set ctrlr polarity cmd");
            status = _set_ctrlr_pol(
                    packet.payload.controller_polarity_data.controller_id,
                    static_cast<ControllerPolarity>(packet.payload.controller_polarity_data.polarity));
            break;

        case GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE:
            GPIO_LOG_INFO("Got a set ctrlr tick rate cmd");
            status = _set_ctrlr_tick_rate(
                    packet.payload.controller_tick_rate.controller_id,
                    packet.payload.controller_tick_rate.delta_tick_rate);
            break;

        case GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE:
            GPIO_LOG_INFO("Got a set ctrlr notif mode cmd");
            status = _set_ctrlr_notif_mode(
                    packet.payload.controller_notif_data.controller_id,
                    static_cast<ControllerNotifMode>(packet.payload.controller_notif_data.notif_mode));
            break;

        case GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER:
            GPIO_LOG_INFO("Got an add pins to ctrlr cmd");
            status = _add_pins_to_ctrlr(
                    packet.payload.controller_pins_data.controller_id,
                    packet.payload.controller_pins_data.num_pins,
                    packet.payload.controller_pins_data.pins);
            break;

        case GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER:
            GPIO_LOG_INFO("Got a mute ctrlr cmd");
            status = _set_ctrlr_mute_status(
                    packet.payload.controller_mute_data.controller_id,
                    static_cast<ControllerMuteStatus>(packet.payload.controller_mute_data.mute_status));
            break;

        case GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES:
            GPIO_LOG_INFO("Got a set ctrlr res cmd");
            status = _set_analog_input_res(
                    packet.payload.analog_controller_res_data.controller_id,
                    packet.payload.analog_controller_res_data.res_in_bits);
            break;

        case GPIO_SUB_CMD_SET_CONTROLLER_RANGE:
            GPIO_LOG_INFO("Got a set ctrlr range cmd");
            status = _set_ctrlr_range(
                    packet.payload.controller_range_data.controller_id,
                    packet.payload.controller_range_data.min_val,
                    packet.payload.controller_range_data.max_val);
            break;

        case GPIO_SUB_CMD_SET_CONTROLLER_DEBOUNCE_MODE:
            GPIO_LOG_INFO("Got a set ctrlr debounce mode cmd");
            status = _set_ctrlr_debounce_mode(
                    packet.payload.controller_debounce_data.controller_id,
                    static_cast<ControllerDebounceMode>(packet.payload.controller_debounce_data.controller_debounce_mode));
            break;

        case GPIO_SUB_CMD_SET_ANALOG_TIME_CONSTANT:
            GPIO_LOG_INFO("Got a set time constant cmd");
            status = _set_time_constant(
                    packet.payload.time_constant_data.controller_id,
                    packet.payload.time_constant_data.time_constant);
            break;

        default:
            GPIO_LOG_WARNING("Invalid sub cmd by packet %d",
                             packet.sequence_no);
            status = GPIO_INVALID_SUB_CMD;
            break;
        }

        _tx_packet_fifo.send_ack(status,
                                 _current_system_tick,
                                 packet.sequence_no);
    }

    inline void _handle_get_val_cmd(const GpioPacket &packet)
    {
        const auto &id = packet.payload.gpio_value_request.controller_id;

        auto result = _digital_inputs.get_val(id);
        if (result.first)
        {
            _tx_packet_fifo.send_ack(GPIO_OK,
                                     _current_system_tick,
                                     packet.sequence_no);
            _tx_packet_fifo.send_val(id,
                                     result.second,
                                     _current_system_tick);
            return;
        }

        result = _digital_outputs.get_val(id);
        if (result.first)
        {
            _tx_packet_fifo.send_ack(GPIO_OK,
                                     _current_system_tick,
                                     packet.sequence_no);
            _tx_packet_fifo.send_val(id,
                                     result.second,
                                     _current_system_tick);
            return;
        }

        result = _analog_inputs.get_val(id);
        if (result.first)
        {
            _tx_packet_fifo.send_ack(GPIO_OK,
                                     _current_system_tick,
                                     packet.sequence_no);
            _tx_packet_fifo.send_val(id,
                                     result.second,
                                     _current_system_tick);
            return;
        }

        GPIO_LOG_ERROR("Cannot get val. Invalid id %d", id);
        _tx_packet_fifo.send_ack(GPIO_INVALID_CONTROLLER_ID,
                                 _current_system_tick,
                                 packet.sequence_no);
    }

    inline void _handle_set_val_cmd(const GpioPacket &packet)
    {
        const auto &id = packet.payload.gpio_value_data.controller_id;
        const auto &val = packet.payload.gpio_value_data.controller_val;

        auto status = _digital_outputs.set_val(id, val);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            _tx_packet_fifo.send_ack(status,
                                     _current_system_tick,
                                     packet.sequence_no);
            return;
        }

        /* Digital inputs and analog inputs can have their value set before
           the gpio process is started. This becomes their initial value */
        if (_is_running)
        {
            GPIO_LOG_ERROR("Cannot set val of a non Output Ctrlr id %d", id);
            _tx_packet_fifo.send_ack(GPIO_INVALID_CONTROLLER_ID,
                                     _current_system_tick,
                                     packet.sequence_no);
            return;
        }

        status = _digital_inputs.set_val(id, val);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            _tx_packet_fifo.send_ack(status,
                                     _current_system_tick,
                                     packet.sequence_no);
            return;
        }

        status = _analog_inputs.set_val(id, val);
        if (status != GPIO_INVALID_CONTROLLER_ID)
        {
            _tx_packet_fifo.send_ack(status,
                                     _current_system_tick,
                                     packet.sequence_no);
            return;
        }

        GPIO_LOG_ERROR("Cannot set val. Invalid id %d", id);
        _tx_packet_fifo.send_ack(GPIO_INVALID_CONTROLLER_ID,
                                 _current_system_tick,
                                 packet.sequence_no);

        return;
    }

    uint32_t _current_system_tick;
    bool _is_running;
    int _system_tick_rate;
    int _system_tick_period_ns;
    int _adc_tick_rate;
    int _adc_chans_per_tick;

    uint32_t* _input_pin_data;
    uint32_t* _output_pin_data;
    uint32_t* _analog_pin_data;

    CtrlrSet<InputCtrlr<NumInputPins>, NumInputPins> _digital_inputs;
    CtrlrSet<OutputCtrlr<NumOutputPins>, NumOutputPins> _digital_outputs;
    CtrlrSet<AnalogCtrlr<NumAnalogPins>, NumAnalogPins> _analog_inputs;

    GpioTxPacketFifo _tx_packet_fifo;
};


} // gpio

#endif // GPIO_CLIENT_H_