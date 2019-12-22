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
 * @brief Contains the implementation of the AnalogCtrlr class which performs the
 * logic of analog controllers
 *
 * @copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */

#ifndef ANALOG_CTRLR_H_
#define ANALOG_CTRLR_H_

#include <cmath>

#include "gpio_protocol/gpio_protocol.h"
#include "gpio_fifo.h"
#include "gpio_system_interface.h"

namespace gpio {

constexpr float DEFAULT_ANALOG_FILTER_TIME_CONSTANT = 0.020; //20 ms
constexpr int NUM_FILTER_WARMUP_ITERATIONS = 300;

/**
 * @brief Class which contains the logic for analog input controllers.
 * @tparam NumAnalogPins the number of analog pins on the board
 */
template <int NumAnalogPins>
class AnalogCtrlr
{
public:
    AnalogCtrlr() : _tx_packet_fifo(nullptr),
                    _gpio_sys_interface(nullptr)

    {
        static_assert(NumAnalogPins != 0);
        reset_to_initial_state();
    }

    ~AnalogCtrlr() = default;

    /**
     * @brief Initialize this analog controller.
     * @param gpio_sys_interface The pointer to an instance of the
     *        GpioSysInterface class needed for run time info of the gpio system
     * @param tx_packet_fifo Pointer to an instance of the tx packet fifo.
     */
    void init(GpioSysInterface* gpio_sys_interface,
              GpioTxPacketFifo* tx_packet_fifo)
    {
        _gpio_sys_interface = gpio_sys_interface;
        _tx_packet_fifo = tx_packet_fifo;
    }

    /**
     * @brief Initialize this analog controller to its initial state.
     */
    inline void reset_to_initial_state()
    {
        _id = 0;
        _is_active = false;
        _pin_num = 0;
        _pin_info_recvd = false;
        _is_muted = false;
        _notif_mode = GPIO_ON_VALUE_CHANGE;
        _delta_ticks = 1;
        _res_diff = 0;
        _current_ctrlr_tick = 0;
        _time_constant = DEFAULT_ANALOG_FILTER_TIME_CONSTANT;

        reset_ctrlr_val();
    }

    /**
     * @brief Get the id of this analog controller
     * @return The id of this analog controller
     */
    inline int get_id()
    {
        return _id;
    }

    /**
     * @brief Check if this analog controller is active
     * @return True if active, false if not
     */
    inline bool is_active()
    {
        return _is_active;
    }

    /**
     * @brief Check if this analog controller has been initialized, i.e if it
     *        has been assigned a pin number.
     * @return True if ok, false if not.
     */
    inline bool is_init()
    {
        if(!_pin_info_recvd)
        {
            GPIO_LOG_ERROR("Analog Ctrlr id %d has no pin", _id);
            return false;
        }

        return true;
    }

    /**
     * @brief Iterate through a list of used analog pins and check if this
     *        analog controller's pin is duplicated. If not, then add it
     *        to the used pin list.
     * @param used_analog_pin_list A list of all used analog pins.
     * @param total_num_used_pins The total number of analog pins used in the list.
     * @return True if duplicated, false if not.
     */
    inline bool check_duplicated_pins(std::array<uint32_t, NumAnalogPins>& used_analog_pin_list,
                                      int& total_num_used_pins)
    {
        for(int j = 0; j < total_num_used_pins; j++)
        {
            if(_pin_num == used_analog_pin_list[j])
            {
                return true;
            }
        }

        used_analog_pin_list[total_num_used_pins] = _pin_num;
        total_num_used_pins++;
        return false;
    }

    /**
     * @brief Reset the value of this analog controller to its default value.
     */
    inline void reset_ctrlr_val()
    {
        _val = 0;
        _previous_val = 0;
    }

    /**
     * @brief Activate the controller.
     * @param id The new id of the controller
     */
    inline void activate(int id)
    {
        reset_to_initial_state();

        _id = id;
        _is_active = true;
    }

    /**
     * @brief Set the tick rate of the analog controller. The tick rate is a
     *        multiple of the system tick rate.
     * @param tick_rate The new tick rate of the analog controller
     * @return GPIO_OK
     */
    inline GpioReturnStatus set_tick_rate(int tick_rate)
    {
        _delta_ticks = tick_rate;

        GPIO_LOG_INFO("Analog id %d tick rate set to %d", _id, tick_rate);
        return GPIO_OK;
    }

    /**
     * @brief Set the notification mode for this analog controller.
     * @param notif_mode The new notification mode.
     * @return GPIO_INVALID_COMMAND_FOR_CONTROLLER when notification mode is
     *         GPIO_WHEN_TOGGLED_ON or GPIO_WHEN_TOGGLED_OFF.
     *         GPIO_OK otherwise.
     */
    inline GpioReturnStatus set_notif_mode(ControllerNotifMode notif_mode)
    {
        if(_notif_mode == GPIO_WHEN_TOGGLED_ON ||
           _notif_mode == GPIO_WHEN_TOGGLED_OFF)
        {
            GPIO_LOG_WARNING("Cannot set notif mode to analog ctrlr id %d", _id);
            GPIO_LOG_WARNING("It only support when toggled on/off notif modes");
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        _notif_mode = notif_mode;

        GPIO_LOG_INFO("Analog ctrlr id %d notif mode set to %d", _id, notif_mode);
        return GPIO_OK;
    }

    /**
     * @brief Add pins to the analog controller
     * @param num_pins The number of pins. It should be 1
     * @param pin_list The pointer to the list of pin numbers.
     * @return GPIO_PARAMETER_ERROR when more than 1 pin is to be added.
     *         GPIO_Ok otherwise
     */
    inline GpioReturnStatus add_pins(int num_pins,
                                     const uint8_t* const pin_list)
    {
        if(num_pins != 1)
        {
            GPIO_LOG_ERROR("Cannot add pins to analog ctrlr id %d, expects 1 pin only");
            return GPIO_PARAMETER_ERROR;
        }

        _pin_num = pin_list[0];
        _pin_info_recvd = true;

        GPIO_LOG_INFO("Added pin %d to analog id %d", pin_list[0], _id);
        return GPIO_OK;
    }

    /**
     * @brief Mute or unmute this analog controller
     * @param mute_status The mute status.
     * @return GPIO_OK always.
     */
    inline GpioReturnStatus set_mute_status(ControllerMuteStatus mute_status)
    {
        if(mute_status == GPIO_CONTROLLER_MUTED)
        {
            _is_muted = true;
            _val = 0;
            _previous_val = 0;
        }

        GPIO_LOG_INFO("Analog id %d mute status set to %d", _id, _is_muted);
        return GPIO_OK;
    }

    /**
     * @brief Set the difference in bits between the resolution of this analog
     *        controller and the ADC's resolution.
     * @param adc_res_diff The difference in resolution of this analog
     *        controller and the ADC's resolution.
     * @return GPIO_OK always
     */
    inline GpioReturnStatus set_res_diff(int adc_res_diff)
    {
        if(_res_diff == adc_res_diff)
        {
            return GPIO_OK;
        }

        _res_diff = adc_res_diff;

        GPIO_LOG_INFO("Analog id %d res lowered by %d bits", _id, adc_res_diff);
        return GPIO_OK;
    }

    /**
     * @brief Warms up the filters with the current value on the pin
     */
    inline void warmup_filter()
    {
        _calc_filter_coeffs();
        _settle_filter();
    }

    /**
     * Sets the time constant for this controller's filter
     * @param time_constant The time constant
     * @return GPIO_PARAMETER_ERROR if time constant is below 0
     *         GPIO OK otherwise.
     */
    inline GpioReturnStatus set_time_constant(float time_constant)
    {
        if(time_constant <= 0.0)
        {
            GPIO_LOG_ERROR("Cannot set time constant %.3f to analog %d : " \
                           " its below 0", time_constant, _id);
            return GPIO_PARAMETER_ERROR;
        }

        _time_constant = time_constant;

        GPIO_LOG_INFO("Analog id %d time constant set to %.3f at " \
                      "sample rate %d Hz", _id, _time_constant, _adc_sampling_freq);
        return  GPIO_OK;
    }

    /**
     * @brief Get the value of this analog controller.
     * @return The value of this analog controller.
     */
    inline uint32_t get_val()
    {
        return _val;
    }

    /**
     * @brief Set the value of this analog controller. This can be used to set
     *        up the initial condition of this controller, however, the value
     *        will be overwritten once this controller has been processed.
     * @param val The new value of this analog controller.
     * @return GPIO_OK always
     */
    inline GpioReturnStatus set_val(uint32_t val)
    {
        _val = val;

        GPIO_LOG_INFO("Analog id %d val set to %d", _id, val);
        return GPIO_OK;
    }

    /**
     * @brief Process this analog controller.
     */
    inline void process()
    {
        // Check if it is this ctrlr's pin which has been sampled in this tick
        if(!_gpio_sys_interface->is_adc_pin_sampled_this_tick(_pin_num))
        {
            return;
        }

        // process the sample through the filter
        _process_filter();

        // check if its this ctrlrs turn to send a notification to host
        _current_ctrlr_tick++;
        if(_current_ctrlr_tick != _delta_ticks)
        {
            return;
        }
        _current_ctrlr_tick = 0;

        if(_is_muted)
        {
            return;
        }

        _process_notif_mode();
    }

private:
    /**
     * @brief Helper function to process the analog controller depending on its
     *        notification mode.
     */
    inline void _process_notif_mode()
    {
        if(_notif_mode == GPIO_ON_VALUE_CHANGE)
        {
            if(_val != _previous_val)
            {
                _previous_val = _val;
                _tx_packet_fifo->send_val(_id, _val,
                          _gpio_sys_interface->get_current_system_tick());
            }
        }
        else
        {
            // On every ctrlr tick
            _previous_val = _val;
            _tx_packet_fifo->send_val(_id, _val,
                    _gpio_sys_interface->get_current_system_tick());
        }
    }

    /**
     * @brief Process this analog controllers pin value through the filter.
     */
    inline void _process_filter()
    {
        auto pin_val = _gpio_sys_interface->get_analog_input_pin_val(_pin_num);
        pin_val = pin_val >> _res_diff;

        float val = (_filter_coeffs_b0 * pin_val) + _filter_state_s1;

        _filter_state_s1 = _filter_state_s2 + (_filter_coeffs_b1 * pin_val)
                           - (_filter_coeffs_a1 * val);

        _filter_state_s2 = (_filter_coeffs_b2 * pin_val) - (_filter_coeffs_a2 * val);

        _val = std::round(val);
    }

    /**
     * @brief Helper function to calculate the filter coefficients using the
     *        filters time constant.
     */
    inline void _calc_filter_coeffs()
    {
        float w0 = 1.0/(_time_constant * _gpio_sys_interface->get_adc_tick_rate());
        float alpha = std::sin(w0);
        float beta = std::cos(w0);
        float filter_coeffs_a0 = 1.0 + alpha;

        _filter_coeffs_b0 = (0.5 * (1. - beta)) / filter_coeffs_a0;
        _filter_coeffs_b1 = (1.0 - beta) / filter_coeffs_a0;
        _filter_coeffs_b2 = _filter_coeffs_b0;
        _filter_coeffs_a1 = (-2.0 * beta) / filter_coeffs_a0;
        _filter_coeffs_a2 = (1.0 - alpha) / filter_coeffs_a0;

        // reset state of filter
        _filter_state_s1 = 0.0;
        _filter_state_s2 = 0.0;
    }

    /**
     * @brief Helper function to settle or warm up the filter.
     */
    inline void _settle_filter()
    {
        if(!_pin_info_recvd)
        {
            return;
        }

        for(int i = 0; i < NUM_FILTER_WARMUP_ITERATIONS; i++)
        {
            _process_filter();
        }
    }

    GpioTxPacketFifo* _tx_packet_fifo;
    GpioSysInterface* _gpio_sys_interface;

    int _id;
    bool _is_active;
    uint32_t _pin_num;
    bool _pin_info_recvd;
    bool _is_muted;
    ControllerNotifMode _notif_mode;
    uint32_t _delta_ticks;
    uint32_t _res_diff;

    uint32_t _val;
    uint32_t _previous_val;
    uint32_t _current_ctrlr_tick;

    // Filter vars
    float _time_constant;
    float _filter_coeffs_b0;
    float _filter_coeffs_b1;
    float _filter_coeffs_b2;
    float _filter_coeffs_a1;
    float _filter_coeffs_a2;
    float _filter_state_s1;
    float _filter_state_s2;
};

} // gpio

#endif // ANALOG_CTRLR_H_