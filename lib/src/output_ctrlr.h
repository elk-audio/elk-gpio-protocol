#ifndef OUTPUT_CTRLR_H_
#define OUTPUT_CTRLR_H_

#include <cstdint>
#include <cstring>
#include <variant>
#include <array>

#include "gpio_protocol/gpio_protocol.h"
#include "gpio_fifo.h"

namespace gpio {

class MuxCtrlrInterface
{
public:
    virtual bool attach_ctrlr(int id, uint32_t mux_pin) = 0;

    virtual int get_mux_current_input_ctrlr_id() = 0;

    virtual int get_mux_current_output_ctrlr_id() = 0;
};

/**
 * @brief Structure to hold mux ctlrr related data.
 *        The mux controller can have a maximum of
 *        NumOutputPins associated to it, to activate or
 *        or deactivate a controller per pin
 */
template <int NumOutputPins>
struct MuxCtrlrType
{
    uint32_t current_active_pin_index;
    uint32_t next_active_pin_index;
    int current_active_id;
    int next_active_id;
    int num_attached_ctrlrs;
    std::array<int, NumOutputPins> attached_ctrlrs;
};

template <int NumOutputPins>
class OutputCtrlr : public MuxCtrlrInterface
{
public:
    static_assert(NumOutputPins != 0);

    inline void set_system_info(uint32_t* const pin_data,
                                const uint32_t* current_system_tick)
    {
        _pin_data = pin_data;
        _current_system_tick = current_system_tick;
    }

    inline void init()
    {
        _id = 0;
        _is_active = false;
        _mux_ctrlr_intf = nullptr;
        _num_pins_used = 0;
        _pin_nums.fill(0);
        _is_muxed = false;
        _is_muted = false;
        _max_val = 0;
        _pin_on_val = 1;
        _pin_off_val = 0;
    }

    inline int get_id()
    {
        return _id;
    }

    inline bool is_active()
    {
        return _is_active;
    }

    inline int get_num_pins_used()
    {
        return _num_pins_used;
    }

    inline void reset_val()
    {
        _val = 0;
        _previous_val = 0;
        _muted_val = 0;
        _current_ctrlr_tick = *_current_system_tick;

        if(_hw_type == GPIO_MUX_OUTPUT)
        {
            _mux_type.current_active_id = _mux_type.attached_ctrlrs[0];
            _mux_type.next_active_id = _mux_type.attached_ctrlrs[1];
            _mux_type.current_active_pin_index = 0;
            _mux_type.next_active_pin_index = 1;
        }
    }

    inline void set_ctrlr_info(int id, GpioHwType type)
    {
        init();

        _id = id;
        _hw_type = type;
        _is_active = true;

        if(_hw_type == GPIO_MUX_OUTPUT)
        {
            _mux_type.num_attached_ctrlrs = 0;
            _mux_type.attached_ctrlrs.fill(-1);
        }

        reset_val();
    }

    inline GpioReturnStatus set_pol(ControllerPolarity pol)
    {
        if(pol == GPIO_ACTIVE_LOW)
        {
            _pin_on_val = 0;
            _pin_off_val = 1;
        }

        GPIO_LOG_INFO("Id %d polarity set to %d", _id, pol);
        return GPIO_OK;
    }

    inline GpioReturnStatus set_mute_status(ControllerMuteStatus mute_status)
    {
        if(_hw_type == GPIO_MUX_OUTPUT)
        {
            GPIO_LOG_WARNING("Cannot mute id %d. it is of type mux", _id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        if(mute_status == GPIO_CONTROLLER_MUTED)
        {
            _is_muted = true;
            _muted_val = _val;
            _val = 0;
        }
        else
        {
            _is_muted = false;
            _val = _muted_val;
            _muted_val = 0;
        }

        GPIO_LOG_INFO("Id %d mute status set to %d", _id, _is_muted);
        return GPIO_OK;
    }

    inline bool is_init()
    {
        switch(_hw_type)
        {
        case GPIO_BINARY_OUTPUT:
        case GPIO_STEPPED_OUTPUT:
            if(_num_pins_used == 0)
            {
                GPIO_LOG_ERROR("Id %d has no pins", _id);
                return false;
            }
            break;

        case GPIO_MUX_OUTPUT:
            /*There should be a minimum of 2 pins with 1 controller attached
              to each in a mux output */
            if(_num_pins_used < 2 ||
               _num_pins_used != _mux_type.num_attached_ctrlrs)
            {
                GPIO_LOG_ERROR("Id %d of type mux does not have atleast 2 pins", _id);
                return false;
            }
            break;

        default:
            break;
        }

        return true;
    }

    inline GpioReturnStatus attach_to_mux(MuxCtrlrInterface* const mux_ctrlr_intf, int mux_pin)
    {
        // A mux cannot be attached to another mux ctrlr
        if(_hw_type == GPIO_MUX_OUTPUT)
        {
            GPIO_LOG_ERROR("Cannot attach id %d to mux, it is of type Mux already", _id);
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        if(mux_ctrlr_intf == nullptr)
        {
            return GPIO_INVALID_MUX_CONTROLLER;
        }

        if(!mux_ctrlr_intf->attach_ctrlr(_id, mux_pin))
        {
            GPIO_LOG_ERROR("Cannot attach id %d to mux", _id);
            return GPIO_INVALID_MUX_CONTROLLER;
        }

        _mux_ctrlr_intf = mux_ctrlr_intf;
        _is_muxed = true;

        GPIO_LOG_INFO("Attached id %d to mux", _id);
        return GPIO_OK;
    }

    /* Check if the mux exits
    if so check if the pin specified exists in its configured list
    if so check of the pin is already used by another controller */
    inline bool attach_ctrlr(int ctrlr_id, uint32_t mux_pin) override
    {
        if(_hw_type != GPIO_MUX_OUTPUT)
        {
            return false;
        }

        for(int i = 0; i < _num_pins_used; i++)
        {
            if(_pin_nums[i] == mux_pin)
            {
                if(_mux_type.attached_ctrlrs[i] >= 0)
                {
                    GPIO_LOG_ERROR("Cannot attach id %d to mux. Mux pin %d"\
                                   " is already attached to id %d", ctrlr_id, mux_pin,
                                    _mux_type.attached_ctrlrs[i]);
                    return false;
                }

                _mux_type.attached_ctrlrs[i] = ctrlr_id;
                _mux_type.num_attached_ctrlrs++;
                return true;
            }
        }

        GPIO_LOG_ERROR("Cannot attach id %d to mux, Mux pin %d is not found", _id, mux_pin);
        return false;
    }

    inline GpioReturnStatus add_pins(int num_pins,
                                     const uint8_t* const pin_list)
    {
        // Check if the controller already uses all the pins
        if(_num_pins_used + num_pins > NumOutputPins)
        {
            GPIO_LOG_ERROR("Cannot add pins for id %d, no more new pins available", _id);
            return GPIO_NO_PINS_AVAILABLE;
        }

        for(int i = 0; i < num_pins; i++)
        {
            _pin_nums[_num_pins_used] = pin_list[i];
            _num_pins_used++;
        }

        _set_max_val();

        GPIO_LOG_INFO("Added %d pins for id %d", num_pins, _id);
        return GPIO_OK;
    }

    inline bool check_duplicated_pins(std::array<uint32_t, NumOutputPins>& used_output_pin_list,
                                      int& total_num_used_pins)
    {
        // ignore if ctrlr is muxed as they can share the same pin
        if(_is_muxed)
        {
            return false;
        }

        if(_num_pins_used + total_num_used_pins > NumOutputPins)
        {
            return true;
        }

        for(int i = 0; i < _num_pins_used; i++)
        {
            for(int j = 0; j < total_num_used_pins; j++)
            {
                if(_pin_nums[i] == used_output_pin_list[j])
                {
                    return true;
                }
            }

            used_output_pin_list[total_num_used_pins] = _pin_nums[i];
            total_num_used_pins++;
        }

        return false;
    }

    inline uint32_t get_val()
    {
        return _val;
    }

    inline GpioReturnStatus set_val(uint32_t val)
    {
        if(_hw_type == GPIO_MUX_OUTPUT)
        {
            GPIO_LOG_WARNING("Cannot set val %d for id %d, it is of type mux output");
            return GPIO_INVALID_COMMAND_FOR_CONTROLLER;
        }

        if(val > _max_val)
        {
            GPIO_LOG_WARNING("Cannot set val %d for id %d, val > max val %d of ctrlr", _id, val, _max_val);
            return GPIO_PARAMETER_ERROR;
        }

        if(_is_muted)
        {
            _muted_val = val;
        }
        else
        {
            _val = val;
        }

        GPIO_LOG_INFO("Output id %d val set to %d", _id, val);
        return GPIO_OK;
    }

    inline void process()
    {
        switch(_hw_type)
        {
        case GPIO_BINARY_OUTPUT:
            _process_binary_output();
            break;

        case GPIO_STEPPED_OUTPUT:
            _process_stepped_output();
            break;

        case GPIO_MUX_OUTPUT:
            _process_mux_output();
            break;

        default:
            // to suppress warnings
            break;
        }
    }

    inline int get_mux_current_output_ctrlr_id() override
    {
        if(_hw_type != GPIO_MUX_OUTPUT)
        {
            return -1;
        }

        /* If mux ctrlr has not been processed yet,
           the current id is the one after the mux has been
           processed */
        if(_current_ctrlr_tick != *_current_system_tick)
        {
            return _mux_type.next_active_id;
        }

        return _mux_type.current_active_id;
    }

    inline int get_mux_current_input_ctrlr_id()
    {
        if(_hw_type != GPIO_MUX_OUTPUT)
        {
            return -1;
        }

        /* Input controllers should read data only if muxes are currently
           active on their id. Only then the data present on the pins
           is the correct one for that input ctrlr*/
        return _mux_type.current_active_id;
    }

private:
    inline void _set_max_val()
    {
        switch (_hw_type)
        {
        case GPIO_STEPPED_OUTPUT:
        case GPIO_MUX_OUTPUT:
            _max_val = _num_pins_used;
            break;

        case GPIO_BINARY_OUTPUT:
            // fast way to compute 2^_num_pins_used - 1
            for(int i = 0; i < _num_pins_used; i++)
            {
                _max_val = (_max_val << i) | 0x01;
            }
            break;

        default:
            break;
        }
    }

    inline void _process_binary_output()
    {
        if(_is_muxed)
        {
            // Check if its this ctrlrs turn to send its output to pins
            if(_mux_ctrlr_intf->get_mux_current_output_ctrlr_id() != _id)
            {
                return;
            }
        }
        else if(_val == _previous_val)
        {
            return;
        }

        for(int i = 0; i < _num_pins_used; i++)
        {
            // get the pin of the ctrlr
            uint32_t pin_num = _pin_nums[i];

            // get the value of the pin.
            uint32_t pin_val = (_val >> i) & 0x1;

            // Set it on PinData according to the polarity configured
            if(pin_val == 0)
            {
                _pin_data[pin_num] = _pin_off_val;
            }
            else
            {
                _pin_data[pin_num] = _pin_on_val;
            }
        }

        /* no point of storing previous val for muxed ctrlrs as they
        should be refreshsed every cycle */
        if(!_is_muxed)
        {
            _previous_val = _val;
        }
    }

    inline void _process_stepped_output()
    {
        if(_is_muxed)
        {
            // Check if its this ctrlrs turn to send its output to pins
            if(_mux_ctrlr_intf->get_mux_current_output_ctrlr_id() != _id)
            {
                return;
            }
        }
        else if(_val == _previous_val)
        {
            return;
        }

        for(uint32_t i = 0; i < (uint32_t)_num_pins_used; i++)
        {
            uint32_t pin_num = _pin_nums[i];

            if(i < _val)
            {
                _pin_data[pin_num] = _pin_on_val;
            }
            else
            {
                _pin_data[pin_num] = _pin_off_val;
            }
        }

        /* no point of storing previous val for muxed ctrlrs as they
           should be refreshsed every cycle */
        if(!_is_muxed)
        {
            _previous_val = _val;
        }
    }

    void _process_mux_output()
    {
        _mux_type.current_active_pin_index = _mux_type.next_active_pin_index;
        _mux_type.current_active_id = _mux_type.next_active_id;

        // Update active pin and active id settings
        _mux_type.next_active_pin_index++;
        if(_mux_type.next_active_pin_index == _max_val)
        {
            _mux_type.next_active_pin_index = 0;
        }
        _mux_type.next_active_id =
            _mux_type.attached_ctrlrs[_mux_type.next_active_pin_index];

        // Process the output pins. Only switch on the current active pin
        for(int pin_index = 0; pin_index < _num_pins_used; pin_index++)
        {
            uint32_t& pin_num = _pin_nums[pin_index];

            if(pin_index == _mux_type.current_active_pin_index)
            {
                _pin_data[pin_num] = _pin_on_val;
            }
            else
            {
                _pin_data[pin_num] = _pin_off_val;
            }
        }

        _current_ctrlr_tick = *_current_system_tick;
    }

    uint32_t* _pin_data;
    const uint32_t* _current_system_tick;

    int _id;
    bool _is_active;
    GpioHwType _hw_type;
    bool _is_muxed;
    MuxCtrlrInterface* _mux_ctrlr_intf;
    int _num_pins_used;
    std::array<uint32_t, NumOutputPins> _pin_nums;
    bool _is_muted;
    uint32_t _val;
    uint32_t _previous_val;
    uint32_t _muted_val;
    uint32_t _max_val;
    uint32_t _pin_on_val;
    uint32_t _pin_off_val;
    uint32_t _current_ctrlr_tick;
    MuxCtrlrType<NumOutputPins> _mux_type;
};

} // gpio

#endif // OUTPUT_CTRLR_H_