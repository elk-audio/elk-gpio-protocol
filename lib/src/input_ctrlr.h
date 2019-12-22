/*
 * Copyright 2019 Modern Ancient Instruments Networked AB, dba Elk
 * Gpio Protocol is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * Gpio Protocol is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Gpio Protocol. If not, see http://www.gnu.org/licenses/ .
 */

/**
 * @brief File which contains the logic for handling digital input controllers.
 *
 * @copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */

#ifndef INPUT_CTRLR_H_
#define INPUT_CTLRL_H_

#include <variant>

#include "gpio_protocol/gpio_protocol.h"
#include "gpio_fifo.h"
#include "gpio_system_interface.h"

namespace gpio {

constexpr uint32_t DEBOUNCE_BUFFER_ON_VAL_ACTIVE_HIGH = 0xFFFFFFFF;
constexpr uint32_t DEBOUNCE_BUFFER_OFF_VAL_ACTIVE_HIGH = 0;
constexpr uint32_t DEBOUNCE_BUFFER_ON_VAL_ACTIVE_LOW = DEBOUNCE_BUFFER_OFF_VAL_ACTIVE_HIGH;
constexpr uint32_t DEBOUNCE_BUFFER_OFF_VAL_ACTIVE_LOW = DEBOUNCE_BUFFER_ON_VAL_ACTIVE_HIGH;
constexpr uint32_t ENC_CW_TRANSITION_SNAPSHOT_ACTIVE_HIGH = 0x87;
constexpr uint32_t ENC_CCW_TRANSITION_SNAPSHOT_ACTIVE_HIGH = 0x4B;
constexpr uint32_t ENC_CW_TRANSITION_SNAPSHOT_ACTIVE_LOW =  0x4B;
constexpr uint32_t ENC_CCW_TRANSITION_SNAPSHOT_ACTIVE_LOW = 0x87;

/**
 * @brief Class which contains the logic for digital input controllers.
 * @tparam NumInputPins the number of digital input pins on the board
 */
template <int NumInputPins>
class InputCtrlr
{
public:
    InputCtrlr()
    {
        static_assert(NumInputPins != 0);
        reset_to_initial_state();
    }

    ~InputCtrlr() = default;

    /**
     * @brief Initialize this digital input controller.
     * @param gpio_sys_interface The pointer to an instance of the
     *        GpioSysInterface class needed for run time info of the gpio system
     * @param tx_packet_fifo Pointer to an instance of the tx packet fifo.
     */
    inline void init(GpioSysInterface* gpio_sys_interface,
                     GpioTxPacketFifo* tx_packet_fifo)
    {
        _gpio_sys_interface = gpio_sys_interface;
        _tx_packet_fifo = tx_packet_fifo;
    }

    /**
     * @brief Reset this digital input controller to its initial state.
     */
    inline void reset_to_initial_state()
    {
        _id = 0;
        _is_active = false;
        _mux_ctrlr_intf = nullptr;
        _num_pins_used = 0;
        _pin_nums.fill(0);
        _is_muxed = false;
        _is_muted = false;
        _is_debounced = true;
        _pin_on_val = 1;
        _pin_off_val = 0;
        _debounce_buffer_on_val = DEBOUNCE_BUFFER_ON_VAL_ACTIVE_HIGH;
        _debounce_buffer_off_val = DEBOUNCE_BUFFER_OFF_VAL_ACTIVE_HIGH;
        _debounce_buffer.fill(0);
        _notif_mode = GPIO_ON_VALUE_CHANGE;
        _min_val = 0;
        _max_val = 0;
        _tick_rate = 1;
        _enc_cw_transition_val = ENC_CW_TRANSITION_SNAPSHOT_ACTIVE_HIGH;
        _enc_ccw_transition_val = ENC_CCW_TRANSITION_SNAPSHOT_ACTIVE_HIGH;
    }

    /**
     * @brief Get the id of this digital input controller
     * @return The id of this digital input controller
     */
    inline int get_id()
    {
        return _id;
    }

    /**
     * @brief Check if this digital input controller is active
     * @return True if active, false if not
     */
    inline bool is_active()
    {
        return _is_active;
    }

    /**
     * @brief Check if this digital input controller has been initialized, i.e if it
     *        has received all necessary information.
     * @return True if ok, false if not.
     */
    inline bool is_init()
    {
        switch(_hw_type)
        {
            case GPIO_BINARY_INPUT:
            case GPIO_N_WAY_SWITCH:
                if(_num_pins_used == 0)
                {
                    GPIO_LOG_ERROR("ID %d has no pins", _id);
                    return false;
                }

                if(_num_pins_used > 1)
                {
                    if(_notif_mode == GPIO_WHEN_TOGGLED_ON ||
                       _notif_mode == GPIO_WHEN_TOGGLED_OFF)
                       {
                           GPIO_LOG_WARNING("Input ID %d has invalid notif mode", _id);
                           GPIO_LOG_WARNING("Allowed modes for it are when toggled on and toggled off", _id);

                           _notif_mode = GPIO_ON_VALUE_CHANGE;
                           return true;
                       }
                }
                break;

            case GPIO_ROTARY_ENCODER:
                if(_num_pins_used != 2 ||
                   _max_val == 0)
                {
                    GPIO_LOG_ERROR("Input ID %d of type encoder uses does not use only 2 pins", _id);
                    return false;
                }
                break;

            default:
                break;
        }

        return true;
    }

    /**
     * @brief Iterate through a list of used digital input pins and check if this
     *        digital input controller's pin is duplicated. If not, then add it
     *        to the used pin list.
     * @param used_input_pin_list A list of all used digital input pins.
     * @param total_num_used_pins The total number of digital input pins used in
     *        the list.
     * @return True if duplicated, false if not.
     */
    inline bool check_duplicated_pins(std::array<uint32_t, NumInputPins>& used_input_pin_list,
                                      int& total_num_used_pins)
    {
        // ignore if ctrlr is muxed as they can share the same pin
        if(_is_muxed)
        {
            return false;
        }

        if(_num_pins_used + total_num_used_pins > NumInputPins)
        {
            return true;
        }

        for(int i = 0; i < _num_pins_used; i++)
        {
            for(int j = 0; j < total_num_used_pins; j++)
            {
                if(_pin_nums[i] == used_input_pin_list[j])
                {
                    return true;
                }
            }

            used_input_pin_list[total_num_used_pins] = _pin_nums[i];
            total_num_used_pins++;
        }

        return false;
    }

    /**
     * @brief Activate this digital input controller
     * @param id The new id of this controller
     * @param type The hw type of this digital input controller
     */
    inline void activate(int id, GpioHwType type)
    {
        reset_to_initial_state();

        _id = id;
        _hw_type = type;
        _is_active = true;

        reset_ctrlr_val();
    }

    /**
     * @brief Reset the value of this digital input controller to its default.
     */
    inline void reset_ctrlr_val()
    {
        _next_ctrlr_tick = _gpio_sys_interface->get_current_system_tick() +
                           _tick_rate;

        switch (_hw_type)
        {
        case GPIO_BINARY_INPUT:
        case GPIO_N_WAY_SWITCH:
            _debounce_buffer.fill(_debounce_buffer_off_val);
            _val = 0;
            _previous_val = 0;
            break;

        case GPIO_ROTARY_ENCODER:
            _debounce_buffer[0] = 0;
            _debounce_buffer[1] = 0;
            _val = _min_val;
            _previous_val = _min_val;

        default:
            break;
        }
    }

    /**
     * @brief Attach this digital controller to a mux controller.
     * @param mux_ctrlr_intf The mux controller's interface
     * @param mux_pin The pin of the mux controller which is associated with this
     *        digital input controller
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER if this digital input controller
     *         is of type rotary encoder
     *         GPIO_INVALID_MUX_CONTROLLER if mux controller if it is an invalid
     *         mux controller or mux pin.
     *         GPIO_OK otherwise.
     *
     */
    inline GpioReturnStatus attach_to_mux(MuxCtrlrInterface* const mux_ctrlr_intf, int mux_pin)
    {
        if(_hw_type == GPIO_ROTARY_ENCODER)
        {
            GPIO_LOG_ERROR("Cannot attach Input ctrlr to mux, Id %d is of type encoder", _id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        if(mux_ctrlr_intf == nullptr)
        {
            return GPIO_INVALID_MUX_CONTROLLER;
        }

        if(!mux_ctrlr_intf->attach_ctrlr(_id, mux_pin))
        {
            GPIO_LOG_ERROR("Cannot attach Input id %d to mux", _id);
            return GPIO_INVALID_MUX_CONTROLLER;
        }

        _mux_ctrlr_intf = mux_ctrlr_intf;
        _is_muxed = true;

        GPIO_LOG_INFO("Input id %d attached to a mux", _id);
        return GPIO_OK;
    }

    /**
     * @brief Set the polarity of this digital input controller.
     * @param pol The new polarity.
     * @return GPIO_OK always
     */
    inline GpioReturnStatus set_pol(ControllerPolarity pol)
    {
        if(pol == GPIO_ACTIVE_LOW)
        {
            _pin_on_val = 0;
            _pin_off_val = 1;
            _debounce_buffer_on_val = DEBOUNCE_BUFFER_ON_VAL_ACTIVE_LOW;
            _debounce_buffer_off_val = DEBOUNCE_BUFFER_OFF_VAL_ACTIVE_LOW;
            _debounce_buffer.fill(DEBOUNCE_BUFFER_OFF_VAL_ACTIVE_LOW);
            _enc_cw_transition_val = ENC_CW_TRANSITION_SNAPSHOT_ACTIVE_LOW;
            _enc_ccw_transition_val = ENC_CCW_TRANSITION_SNAPSHOT_ACTIVE_LOW;
        }

        GPIO_LOG_INFO("Input id %d pol changed", _id);
        return GPIO_OK;
    }

    /**
     * @brief Set the tick rate of this digital input controller.
     * @param tick_rate The new tick rate.
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER if this digital input controller
     *         is of type rotary encoder
     *         GPIO_OK otherwise
     */
    inline GpioReturnStatus set_tick_rate(int tick_rate)
    {
        if(_hw_type == GPIO_ROTARY_ENCODER)
        {
            GPIO_LOG_WARNING("Cannot set tick rate for Input id %d. It is of type encoder", _id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        _tick_rate = tick_rate;
        _next_ctrlr_tick = _gpio_sys_interface->get_current_system_tick() +
                           _tick_rate;

        GPIO_LOG_INFO("Input id %d tick rate set to %d", _id, _tick_rate);
        return GPIO_OK;
    }

    /**
     * @brief Set the notification mode of this digital input controller
     * @param notif_mode the new notification mode
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER if hw type is either
     *         GPIO_ROTARY_ENCODER or GPIO_N_WAY_SWITCH and notif mode is either
     *         GPIO_WHEN_TOGGLED_ON or GPIO_WHEN_TOGGLED_OFF
     *
     *         GPIO_OK otherwise
     */
    inline GpioReturnStatus set_notif_mode(ControllerNotifMode notif_mode)
    {
        switch (_hw_type)
        {
        case GPIO_ROTARY_ENCODER:
        case GPIO_N_WAY_SWITCH:
            if(_notif_mode == GPIO_WHEN_TOGGLED_ON ||
               _notif_mode == GPIO_WHEN_TOGGLED_OFF)
            {
                GPIO_LOG_WARNING("Cannot set notif mode when toggled on/off to Input id %d", _id);
                GPIO_LOG_WARNING("Such Notif mode can only be set for Binary input types");
                return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
            }
            break;

        default:
            break;
        }

        _notif_mode = notif_mode;

        GPIO_LOG_INFO("Notif mode set for Input id %d", _id);
        return GPIO_OK;
    }

    /**
     * @brief Add pins to this digital input controller
     * @param num_pins The number of pins to add.
     * @param pin_list Pointer to the list of pins to be added.
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER if more than 2 pins are added
     *         to a hw type of rotary encoder.
     *         GPIO_NO_PINS_AVAILABLE when no more pins are available.
     *         GPIO_OK otherwise.
     */
    inline GpioReturnStatus add_pins(int num_pins,
                                     const uint8_t* const pin_list)
    {
        if(_hw_type == GPIO_ROTARY_ENCODER)
        {
            if(_num_pins_used + num_pins > 2)
            {
                GPIO_LOG_ERROR("Cannot add pins for Input id %d, too many pins for type encoder", _id);
                return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
            }
        }
        else
        {
            if(_num_pins_used + num_pins > NumInputPins)
            {
                GPIO_LOG_ERROR("Cannot add pins for Input id %d, no new pins available", _id);
                return GPIO_NO_PINS_AVAILABLE;
            }
        }

        for(int i = 0; i < num_pins; i++)
        {
            _pin_nums[_num_pins_used] = pin_list[i];
            _num_pins_used++;
        }

        // Set max value
        switch (_hw_type)
        {
        case GPIO_BINARY_INPUT:
            for(int i = 0; i < _num_pins_used; i++)
            {
                _max_val = (_max_val << i) | 0x01;
            }
            break;

        case GPIO_N_WAY_SWITCH:
            _max_val = _num_pins_used;
            break;

        default:
            break;
        }

        GPIO_LOG_INFO("Added %d pins for Input id %d", num_pins, _id);
        return GPIO_OK;
    }

    /**
     * @brief Set the mute status of this digital input controller
     * @param mute_status the mute status.
     * @return GPIO_OK always
     */
    inline GpioReturnStatus set_mute_status(ControllerMuteStatus mute_status)
    {
        if(mute_status == GPIO_CONTROLLER_MUTED)
        {
            _is_muted = true;
            _val = 0;
            _previous_val = 0;
        }

        GPIO_LOG_INFO("Input Id %d mute status set to %d", _id, _is_muted);
        return GPIO_OK;
    }

    /**
     * @brief Set the input range for this digital input controller.
     * @param min_val The minimum value
     * @param max_val The maximum value
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER if hwtype is not a rotary
     *         encoder.
     *         GPIO_OK otherwise.
     */
    inline GpioReturnStatus set_range(uint32_t min_val, uint32_t max_val)
    {
        if(_hw_type != GPIO_ROTARY_ENCODER)
        {
            GPIO_LOG_WARNING("Cannot set range for Input id %d of non encoder type", _id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        _min_val = min_val;
        _max_val = max_val;

        GPIO_LOG_INFO("Range for Input id %d set to %d - %d", _id, min_val, max_val);
        return GPIO_OK;
    }

    /**
     * @brief Enable or disable debounce for this input controller
     * @param debounce_mode The Debounce mode
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER if this digital input controller
     *         is of hw type Rotary Encoder.
     *         GPIO_OK otherwise.
     */
    inline GpioReturnStatus set_debounce_mode(ControllerDebounceMode debounce_mode)
    {
        if(_hw_type == GPIO_ROTARY_ENCODER)
        {
            GPIO_LOG_WARNING("Cannot set debounce for Input id %d of encoder type", _id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        if(debounce_mode == GPIO_CONTROLLER_DEBOUNCE_DISABLED)
        {
            _is_debounced = false;
        }

        GPIO_LOG_INFO("Debounce for Input id %d set to %d", _id, _is_debounced);
        return GPIO_OK;
    }

    /**
     * @brief Get this controllers value
     * @return The controller value.
     */
    inline uint32_t get_val()
    {
        return _val;
    }

    /**
     * @brief Set the value of this input controller. This can be used to set
     *        up the initial condition of this controller, however, the value
     *        will be overwritten once this controller has been processed.
     * @param val The new value of this analog controller.
     * @return GPIO_PARAMETER_ERROR if value is over the range of the controller.
     *         GPIO_OK otherwise
 */
    inline GpioReturnStatus set_val(uint32_t val)
    {
        if(val > _max_val)
        {
            GPIO_LOG_WARNING("Cannot set val %d for Input id %d, val > max val %d of ctrlr", _id, val, _max_val);
            return GPIO_PARAMETER_ERROR;
        }

        _val = val;
        _previous_val = val;

        GPIO_LOG_INFO("Input id %d val set to %d", _id, val);
        return GPIO_OK;
    }

    /**
     * @brief Process this digital input controller,
     */
    inline void process()
    {
        if(_gpio_sys_interface->get_current_system_tick() != _next_ctrlr_tick)
        {
            return;
        }

        _next_ctrlr_tick = _gpio_sys_interface->get_current_system_tick() +
                           _tick_rate;

        if(_is_muted)
        {
            return;
        }

        if(_is_muxed)
        {
            if(_mux_ctrlr_intf->get_mux_current_input_ctrlr_id() != _id)
            {
                return;
            }
        }

        switch (_hw_type)
        {
        case GPIO_BINARY_INPUT:
            _process_binary_input();
            break;

        case GPIO_N_WAY_SWITCH:
            _process_n_way_switch();
            break;

        case GPIO_ROTARY_ENCODER:
            _process_rotary_encoder();
            break;

        default:
            break;
        }
    }

private:
    /**
     * @brief Helper function to set or clear bits in the controllers value.
     * @param bit_pos The bit vosition
     * @param bit_value The bit value
     */
    inline void _write_bit_in_val(uint32_t bit_pos, uint32_t bit_value)
    {
        uint32_t bit_mask = (0x1 << bit_pos);
        _val = (_val & ~bit_mask) | (bit_value << bit_pos);
    }

    /**
     * @brief Process the controller depending on the notification mode.
     */
    inline void _process_notif_mode()
    {
        switch (_notif_mode)
        {
        case GPIO_ON_VALUE_CHANGE:
            if(_val != _previous_val)
            {
                _previous_val = _val;
                _tx_packet_fifo->send_val(_id,
                        _val, _gpio_sys_interface->get_current_system_tick());
                GPIO_LOG_INFO("Id %d val = %d", _id, _val);
                return;
            }
            break;

        case GPIO_EVERY_CONTROLLER_TICK:
            _previous_val = _val;
            _tx_packet_fifo->send_val(_id,
                    _val, _gpio_sys_interface->get_current_system_tick());
            return;

        case GPIO_WHEN_TOGGLED_ON:
            if(_previous_val == _pin_off_val &&
               _val == _pin_on_val)
            {
                _previous_val = _val;
                _tx_packet_fifo->send_val(_id,
                        _val, _gpio_sys_interface->get_current_system_tick());
                return;
            }
            break;

        case GPIO_WHEN_TOGGLED_OFF:
            if(_previous_val == _pin_on_val &&
               _val == _pin_off_val)
            {
                _previous_val = _val;
                _tx_packet_fifo->send_val(_id,
                        _val, _gpio_sys_interface->get_current_system_tick());
                return;
            }
            break;

        default:
            break;
        }

        // only needed for GPIO_WHEN_TOGGLED_ON or GPIO_WHEN_TOGGLED_OFF
        if(_val != _previous_val)
        {
            _previous_val = _val;
        }
    }

    /**
     * @brief Process controllers of type binary input
     */
    inline void _process_binary_input()
    {
        for(int i = 0; i < _num_pins_used; i++)
        {
            const auto pin_val =
                    _gpio_sys_interface->get_digital_input_pin_val(_pin_nums[i]);

            if(_is_debounced)
            {
                _debounce_buffer[i] =
                        (uint32_t)(_debounce_buffer[i] << 1) | pin_val;
                if(_debounce_buffer[i] == _debounce_buffer_on_val)
                {
                    _write_bit_in_val(i, 1);
                }
                else if(_debounce_buffer[i] == _debounce_buffer_off_val)
                {
                    _write_bit_in_val(i, 0);
                }
            }
            else
            {
                if(pin_val == _pin_on_val)
                {
                    _write_bit_in_val(i, 1);
                }
                else
                {
                    _write_bit_in_val(i, 0);
                }
            }
        }

        _process_notif_mode();
    }

    /**
     * @brief Process controllers of type n way switch input
     */
    inline void _process_n_way_switch()
    {
        for(int i = 0; i < _num_pins_used; i++)
        {
            const auto pin_val = _gpio_sys_interface->
                            get_digital_input_pin_val( _pin_nums[i]);

            if(_is_debounced)
            {
                _debounce_buffer[i] = (_debounce_buffer[i] < 1) | pin_val;
                if(_debounce_buffer[i] == _debounce_buffer_on_val)
                {
                    _val = i + 1;
                }

                break;
            }

            if(pin_val == _pin_on_val)
            {
                _val = i + 1;
                break;
            }
        }

        _process_notif_mode();
    }

    /**
     * @brief Process controllers of type rotary encoders
     */
    inline void _process_rotary_encoder()
    {
        const auto pin_val_A =
                 _gpio_sys_interface->get_digital_input_pin_val(_pin_nums[0]);
        const auto pin_val_B =
                _gpio_sys_interface->get_digital_input_pin_val(_pin_nums[1]);

        uint32_t& prev_transition_reading = _debounce_buffer[0];
        uint32_t& transition_snapshot = _debounce_buffer[1];

        uint32_t transition_reading = (pin_val_A << 1) | pin_val_B;

        /* needed to weed out multiple reading of the same transition value -
           similar to debouncing */
        if(transition_reading != prev_transition_reading)
        {
            prev_transition_reading = transition_reading;
            transition_snapshot = (transition_snapshot << 2 | transition_reading) & 0x000000FF;
        }

        // Increment data if snapshot matches CW snapshot
        if(transition_snapshot == _enc_cw_transition_val)
        {
            // reset transition_snapshot
            transition_snapshot = 0;
            if(_val != _max_val)
            {
                _val++;
            }
        }
        // Decrement data if snapshot matches CCW snapshot
        else if(transition_snapshot == _enc_ccw_transition_val)
        {
            // reset transition_snapshot
            transition_snapshot = 0;
            if(_val != _min_val)
            {
                _val--;
            }
        }

        _process_notif_mode();
    }

    GpioSysInterface* _gpio_sys_interface;
    GpioTxPacketFifo* _tx_packet_fifo;

    int _id;
    bool _is_active;
    GpioHwType _hw_type;
    bool _is_muxed;
    MuxCtrlrInterface* _mux_ctrlr_intf;
    int _num_pins_used;
    std::array<uint32_t, NumInputPins> _pin_nums;
    bool _is_muted;
    bool _is_debounced;
    ControllerNotifMode _notif_mode;
    uint32_t _pin_on_val;
    uint32_t _pin_off_val;
    uint32_t _debounce_buffer_on_val;
    uint32_t _debounce_buffer_off_val;
    uint32_t _min_val;
    uint32_t _max_val;
    uint32_t _tick_rate;
    uint32_t _enc_cw_transition_val;
    uint32_t _enc_ccw_transition_val;

    uint32_t _val;
    uint32_t _previous_val;
    uint32_t _next_ctrlr_tick;
    std::array<uint32_t, NumInputPins> _debounce_buffer;
};

} // gpio

#endif // INPUT_CTRLR_H_