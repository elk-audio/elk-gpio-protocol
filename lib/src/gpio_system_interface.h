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
 * @brief Simple wrapper and interface to access the various aspects of the
 *        underlying gpio system
 *
 * @copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef GPIO_SYSTEM_INTERFACE_H_
#define GPIO_SYSTEM_INTERFACE_H_

namespace gpio {

// Default system tick period and system tick rate
constexpr int DEFAULT_SYSTEM_TICK_PERIOD_NS = 1000000; // 1KHz
constexpr int DEFAULT_SYSTEM_TICK_RATE = 1000; // 1KHz
constexpr int DEFUALT_ADC_SAMPLING_FREQ = DEFAULT_SYSTEM_TICK_RATE;

/**
 * @brief Class which holds information about the underlying gpio system and
 *        provides necesarry information to the various controllers.
 */
class GpioSysInterface
{
public:
    /**
     * @brief Construct the gpio system interface.
     * @param num_digital_input_pins The number of digital input pins.
     * @param num_digital_output_pins The number of digital output pins.
     * @param num_analog_pins The number of analog input pins.
     */
    GpioSysInterface(const int num_digital_input_pins,
                     const int num_digital_output_pins,
                     const int num_analog_pins) : _num_digital_input_pins(num_digital_input_pins),
                                                  _num_digital_output_pins(num_digital_output_pins),
                                                  _num_analog_pins(num_analog_pins),
                                                  _digital_input_pin_data(nullptr),
                                                  _digital_output_pin_data(nullptr),
                                                  _analog_pin_data(nullptr),
                                                  _adc_chans_per_tick(0),
                                                  _system_tick_rate(DEFAULT_SYSTEM_TICK_RATE),
                                                  _system_tick_period_ns(DEFAULT_SYSTEM_TICK_PERIOD_NS),
                                                  _current_system_tick(0),
                                                  _current_adc_tick(0),
                                                  _max_num_ticks_to_sample_all_adc_chans(0),
                                                  _adc_tick_rate(DEFUALT_ADC_SAMPLING_FREQ),
                                                  _is_init(false)

    {
        reset();
    }

    ~GpioSysInterface() = default;

    /**
     * @brief Initialize the Gpio system interface with info about the hw gpio
     * @param digital_input_pin_data Pointer to the memory location of the
     *        sampled digital input pin data.
     * @param digital_output_pin_data Pointer to the memory location of the
     *        digital output pin data which will be driven on the pins.
     * @param analog_pin_data Pointer to the memory location of the
     *        sampled analog input pin data.
     * @param adc_chans_per_tick information of how many adc channels (or pins)
     *        are sampled every system tick.
     */
    inline void init(uint32_t* digital_input_pin_data,
                     uint32_t* digital_output_pin_data,
                     uint32_t* analog_pin_data,
                     int adc_chans_per_tick)
    {
        _digital_input_pin_data = digital_input_pin_data;
        _digital_output_pin_data = digital_output_pin_data;
        _analog_pin_data = analog_pin_data;
        _adc_chans_per_tick = adc_chans_per_tick;

        if(_num_analog_pins > 0 && adc_chans_per_tick > 0)
        {
            _max_num_ticks_to_sample_all_adc_chans = _num_analog_pins
                                                        / adc_chans_per_tick;
            _adc_tick_rate = (_system_tick_rate * _adc_chans_per_tick)
                             / _num_analog_pins;
        }

        _is_init = true;
        reset();
    }

    /**
     * @brief Reset the system information to its initial state.
     */
    inline void reset()
    {
        _current_system_tick = 0;
        _current_adc_tick = 0;
        _system_tick_rate = DEFAULT_SYSTEM_TICK_RATE;
        _system_tick_period_ns = DEFAULT_SYSTEM_TICK_PERIOD_NS;

        if(!_is_init)
        {
            return;
        }

        for(int i = 0; i < _num_digital_input_pins; i++)
        {
            _digital_input_pin_data[i] = 0;
        }

        for(int i = 0; i < _num_digital_output_pins; i++)
        {
            _digital_output_pin_data[i] = 0;
        }

        for(int i = 0; i < _num_analog_pins; i++)
        {
            _analog_pin_data[i] = 0;
        }
    }

    /**
     * @brief Set a digital output pin
     * @param val The value of the pin
     * @param pin_num The pin number
     */
    inline void set_digital_output_pin_val(uint32_t val, uint32_t pin_num)
    {
        _digital_output_pin_data[pin_num] = val;
    }

    /**
     * @brief Get the value of a digital input pin
     * @param pin_num The pin number
     * @return The pin value
     */
    inline uint32_t get_digital_input_pin_val(uint32_t pin_num)
    {
        return _digital_input_pin_data[pin_num];
    }

    /**
     * @brief Get the value of an analog input pin.
     * @param pin_num The pin number
     * @return The value
     */
    inline uint32_t get_analog_input_pin_val(uint32_t pin_num)
    {
        return _analog_pin_data[pin_num];
    }

    /**
     * @brief Update the current system tick counter.
     */
    inline void update_current_system_tick()
    {
        _current_system_tick++;

        if(_num_analog_pins == 0)
        {
            return;
        }

        _current_adc_tick++;
        if(_current_adc_tick == _max_num_ticks_to_sample_all_adc_chans)
        {
            _current_adc_tick = 0;
        }
    }

    /**
     * @brief Get the value of the current system tick.
     * @return The current system tick.
     */
    inline uint32_t get_current_system_tick()
    {
        return _current_system_tick;
    }

    /**
     * @brief Set the system tick rate.
     * @param system_tick_rate The new system tick rate.
     */
    inline void set_system_tick_rate(uint32_t system_tick_rate)
    {
        _system_tick_rate = system_tick_rate;
        _system_tick_period_ns = 1000000000/system_tick_rate;

        if(_adc_chans_per_tick > 0 && _num_analog_pins > 0)
        {
            _adc_tick_rate = (system_tick_rate * _adc_chans_per_tick)
                             / _num_analog_pins;
        }
    }

    /**
     * @brief Get the system tick period in nanoseconds.
     * @return The system tick period in nanoseconds
     */
    inline uint32_t get_system_tick_period_ns()
    {
        return _system_tick_period_ns;
    }

    /**
     * @brief Get the effective adc tick rate. This is equal to the system tick
     *        rate * adc channels per tick / total number of analog pins.
     * @return The effective adc tick rate
     */
    inline uint32_t get_adc_tick_rate()
    {
        return _adc_tick_rate;
    }

    /**
     * @brief Check to see if a particular analog pin num is sampled in a given
     *        system tick. This is dependent on how many adc channels are sampled
     *        in one system tick.
     * @param pin_num The pin num
     * @return True if it is sampled in this tick, false if not.
     */
    inline bool is_adc_pin_sampled_this_tick(uint32_t pin_num)
    {
        if(_num_analog_pins == 0)
        {
            return false;
        }

        uint32_t start_pin = _current_adc_tick * _adc_chans_per_tick;
        uint32_t end_pin = start_pin + _adc_chans_per_tick - 1;

        return ((pin_num >= start_pin) && (pin_num <= end_pin));
    }

private:
    const int _num_digital_input_pins;
    const int _num_digital_output_pins;
    const int _num_analog_pins;

    // The memory location of the sampled pin data
    uint32_t* _digital_input_pin_data;
    uint32_t* _digital_output_pin_data;
    uint32_t* _analog_pin_data;
    uint32_t _adc_chans_per_tick;

    // system information
    uint32_t _system_tick_rate;
    uint32_t _system_tick_period_ns;
    uint32_t _current_system_tick;

    /**
     * Represents the current adc tick. _adc_chans_per_tick channels are sampled
     * every _current_adc_tick.
     */
    uint32_t _current_adc_tick;

    /**
     * Num of system ticks for all adc channels to be sampled. it is equal to the
     * number of num of analog pins divided by the number of adc channels
     * sampled per system tick.
     */
    uint32_t _max_num_ticks_to_sample_all_adc_chans;

    /**
     * Effective sampling rate of adc pins. This is equal to the system tick
     * rate * adc channels per tick / total number of analog pins.
     */
    uint32_t _adc_tick_rate;

    bool _is_init;
};

} // gpio

#endif // GPIO_SYSTEM_INTERFACE_H_