#ifndef ANALOG_CTRLR_H_
#define ANALOG_CTRLR_H_

#include <cmath>

#include "gpio_protocol/gpio_protocol.h"
#include "gpio_fifo.h"

namespace gpio {

constexpr float DEFAULT_ANALOG_FILTER_TIME_CONSTANT = 0.020; //20 ms
constexpr int DEFUALT_ADC_SAMPLING_FREQ = 250;
constexpr int NUM_FILTER_WARMUP_ITERATIONS = 300;

template <int NumAnalogPins>
class AnalogCtrlr
{
public:
    static_assert(NumAnalogPins != 0);

    inline void set_system_info(uint32_t* const pin_data,
                                const uint32_t* current_system_tick,
                                GpioTxPacketFifo* tx_packet_fifo,
                                int adc_chans_per_tick)
    {
        _pin_data = pin_data;
        _current_system_tick = current_system_tick;
        _tx_packet_fifo = tx_packet_fifo;
        _delta_sampling_ticks = NumAnalogPins/adc_chans_per_tick;
    }

    inline void init()
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
        _current_sampling_tick = 0;
        _next_sampling_tick = 1;
        _time_constant = DEFAULT_ANALOG_FILTER_TIME_CONSTANT;
        _adc_sampling_freq = DEFUALT_ADC_SAMPLING_FREQ;

        reset_val();
    }

    inline int get_id()
    {
        return _id;
    }

    inline bool is_active()
    {
        return _is_active;
    }

    inline bool is_init()
    {
        if(!_pin_info_recvd)
        {
            GPIO_LOG_ERROR("Analog Ctrlr id %d has no pin", _id);
            return false;
        }

        return true;
    }

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

    inline void reset_val()
    {
        _val = 0;
        _previous_val = 0;
    }

    inline void set_ctrlr_info(int id)
    {
        init();

        _id = id;
        _is_active = true;

        reset_val();
    }

    inline GpioReturnStatus set_tick_rate(int tick_rate)
    {
        _delta_ticks = tick_rate;

        GPIO_LOG_INFO("Analog id %d tick rate set to %d", _id, tick_rate);
        return GPIO_OK;
    }

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

        // settle filter for current input value
        _settle_filter();

        _calc_initial_next_sampling_tick();

        GPIO_LOG_INFO("Added pin %d to analog id %d", pin_list[0], _id);
        return GPIO_OK;
    }

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

    inline GpioReturnStatus set_res_diff(int adc_res_diff)
    {
        if(_res_diff == adc_res_diff)
        {
            return GPIO_OK;
        }

        _res_diff = adc_res_diff;

        // Settle filter
        if(_pin_info_recvd)
        {
            _settle_filter();
        }

        GPIO_LOG_INFO("Analog id %d res lowered by %d bits", _id, adc_res_diff);
        return GPIO_OK;
    }

    inline void set_adc_sampling_rate(int adc_sampling_freq)
    {
        _adc_sampling_freq = adc_sampling_freq;
        _calc_filter_coeffs();

        // Settle filter
        if(_pin_info_recvd)
        {
            _settle_filter();
        }
    }

    inline GpioReturnStatus set_time_constant(float time_constant)
    {
        if(time_constant <= 0.0)
        {
            GPIO_LOG_ERROR("Cannot set time constant %.3f to analog %d : its below 0", time_constant, _id);
            return GPIO_PARAMETER_ERROR;
        }

        _time_constant = time_constant;
        _calc_filter_coeffs();

        // Settle filter
        if(_pin_info_recvd)
        {
            _settle_filter();
        }

        GPIO_LOG_INFO("Analog id %d time constant set to %.3f at " \
                      "sample rate %d Hz", _id, _time_constant, _adc_sampling_freq);
        return  GPIO_OK;
    }

    inline uint32_t get_val()
    {
        return _val;
    }

    inline GpioReturnStatus set_val(uint32_t val)
    {
        _val = val;

        GPIO_LOG_INFO("Analog id %d val set to %d", _id, val);
        return GPIO_OK;
    }

    inline void process()
    {
        // Check if it is this ctrlrs pin which has been sampled in this tick
        _current_sampling_tick++;
        if(_current_sampling_tick != _next_sampling_tick)
        {
            return;
        }
        _current_sampling_tick = 0;
        _next_sampling_tick = _delta_sampling_ticks;

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
    inline void _process_notif_mode()
    {
        if(_notif_mode == GPIO_ON_VALUE_CHANGE)
        {
            if(_val != _previous_val)
            {
                _previous_val = _val;
                _tx_packet_fifo->send_val(_id, _val, *_current_system_tick);
            }
        }
        else
        {
            // On every ctrlr tick
            _previous_val = _val;
            _tx_packet_fifo->send_val(_id, _val, *_current_system_tick);
        }
    }

    inline void _process_filter()
    {
        uint32_t pin_val = _pin_data[_pin_num];
        pin_val = pin_val >> _res_diff;

        float val = (_filter_coeffs_b0 * pin_val) + _filter_state_s1;

        _filter_state_s1 = _filter_state_s2 + (_filter_coeffs_b1 * pin_val)
                           - (_filter_coeffs_a1 * val);

        _filter_state_s2 = (_filter_coeffs_b2 * pin_val) - (_filter_coeffs_a2 * val);

        _val = std::round(val);
    }

    /**
     * @brief Function to compute the initial tick offset depending on the number of adc
     *        channels sampled per tick. The goal is to process the analog filter only when
     *        the pins have been sampled. This function sets the initial tick delay so that
     *        when _next_sampling_tick == _current_sample_tick, it is guarenteed that its
     *        pin has been sampled.
     */
    inline void _calc_initial_next_sampling_tick()
    {
        int _adc_chans_per_tick = _delta_sampling_ticks * NumAnalogPins;
        uint32_t chan_end = _adc_chans_per_tick;
        _next_sampling_tick = 1;

        for(int i = 0; i < NumAnalogPins; i++)
        {
            if(i != 0 && i == chan_end)
            {
                _next_sampling_tick++;
                chan_end = chan_end + _adc_chans_per_tick;
            }

            if(i == _pin_num)
            {
                return;
            }
        }
    }

    inline void _calc_filter_coeffs()
    {
        float w0 = 1.0/(_time_constant * _adc_sampling_freq);
        float alpha = std::sin(w0);
        float beta = std::cos(w0); // poor naming??
        float filter_coeffs_a0 = 1.0 + alpha;

        _filter_coeffs_b0 = (0.5 * (1. - beta)) / filter_coeffs_a0;
        _filter_coeffs_b1 = (1.0 - beta) / filter_coeffs_a0;
        _filter_coeffs_b2 = _filter_coeffs_b0;
        _filter_coeffs_a1 = (-2.0 * beta) / filter_coeffs_a0;
        _filter_coeffs_a2 = (1.0 - alpha) / filter_coeffs_a0;

        // reset state of filter
        _filter_state_s1 = 0.0;
        _filter_state_s2 = 0.0;

        // Settle filter
        _settle_filter();
    }

    inline void _settle_filter()
    {
        for(int i = 0; i < NUM_FILTER_WARMUP_ITERATIONS; i++)
        {
            _process_filter();
        }
    }

    uint32_t* _pin_data;
    const uint32_t* _current_system_tick;
    int _delta_sampling_ticks;
    int _adc_sampling_freq;
    GpioTxPacketFifo* _tx_packet_fifo;

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
    uint32_t _current_sampling_tick;
    uint32_t _next_sampling_tick;
    uint32_t _current_ctrlr_tick;

    // Filter vars
    int _adc_tick_rate;
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