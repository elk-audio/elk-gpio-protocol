#ifndef CTRLR_SET_H_
#define CTRLR_SET_H_

#include <cstddef>
#include <tuple>

#include "gpio_protocol/gpio_protocol.h"
#include "output_ctrlr.h"
#include "input_ctrlr.h"
#include "analog_ctrlr.h"
#include "gpio_fifo.h"

namespace gpio {

template<class CtrlrType,
         int NumPins>
class CtrlrSet
{
public:
    inline void set_info(uint32_t* const pin_data,
                         const uint32_t* const current_system_tick)
    {
        static_assert(std::is_same<CtrlrType, OutputCtrlr<NumPins>>::value);

        _num_ctrlrs = 0;
        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.set_system_info(pin_data, current_system_tick);
        }
    }


    inline void set_info(uint32_t* const pin_data,
                         const uint32_t* const current_system_tick,
                         GpioTxPacketFifo* tx_packet_fifo)
    {
        static_assert(std::is_same<CtrlrType, InputCtrlr<NumPins>>::value);

        _num_ctrlrs = 0;
        _tx_packet_fifo = tx_packet_fifo;
        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.set_system_info(pin_data, current_system_tick, tx_packet_fifo);
        }
    }

    inline void set_info(uint32_t* const pin_data,
                         const uint32_t* const current_system_tick,
                         GpioTxPacketFifo* tx_packet_fifo,
                         int adc_chans_per_tick)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        _num_ctrlrs = 0;
        _tx_packet_fifo = tx_packet_fifo;
        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.set_system_info(pin_data, current_system_tick, tx_packet_fifo,
                                  adc_chans_per_tick);
        }
    }

    inline void init()
    {
        _num_ctrlrs = 0;

        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.init();
        }
    }

    inline int get_num_ctrlrs() const
    {
        return _num_ctrlrs;
    }

    inline bool does_id_exist(int id)
    {
        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                break;
            }

            if(ctrlr.get_id() == id)
            {
                return true;
            }
        }

        return false;
    }

    inline bool check_ctrlr_init_status()
    {
        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                break;
            }

            if(!ctrlr.is_init())
            {
                return false;
            }
        }

        return true;
    }

    inline bool check_duplicated_pins()
    {
        std::array<uint32_t, NumPins> used_pin_list;
        int num_used_pins = 0;

        used_pin_list.fill(0);

        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                break;
            }

            if(ctrlr.check_duplicated_pins(used_pin_list, num_used_pins))
            {
                return true;
            }
        }

        return false;
    }

    inline void reset_all_ctrlrs()
    {
        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                return;
            }

            ctrlr.reset_val();
        }
    }

    inline bool reset_ctrlr(int id)
    {
        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                break;
            }

            if(ctrlr.get_id() == id)
            {
                ctrlr.reset_val();
                return true;
            }
        }

        return false;
    }

    inline GpioReturnStatus add_ctrlr(int id, GpioHwType type)
    {
        /* Only applicable to InputCtrlr and OutputCtrlr<NumPins> types. AnalogCtr type
           should use the function below */
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);
        if(_num_ctrlrs == NumPins)
        {
            GPIO_LOG_ERROR("Cannot add new ctrlr of id %d, All pins are used up", id);
            return GPIO_NO_PINS_AVAILABLE;
        }

        _ctrlrs[_num_ctrlrs].set_ctrlr_info(id, type);
        _num_ctrlrs++;

        GPIO_LOG_INFO("Added new ctrlr of id %d", id);
        return GPIO_OK;
    }

    inline GpioReturnStatus add_ctrlr(int id)
    {
        // This function is only applicable for AnalogCtrlr
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                ctrlr.set_ctrlr_info(id);
                _num_ctrlrs++;

                GPIO_LOG_INFO("Added new ctrlr of id %d", id);
                return GPIO_OK;
            }

            if(ctrlr.get_id() == id)
            {
                return GPIO_INVALID_CONTROLLER_ID;
            }
        }

        GPIO_LOG_ERROR("Cannot add new ctrlr of id %d, All pins are used up", id);
        return GPIO_NO_PINS_AVAILABLE;
    }

    inline CtrlrType* get_ctrlr(int id)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return nullptr;
        }

        return &_ctrlrs[ctrlr_num];
    }

    inline GpioReturnStatus attach_to_mux(int id, uint32_t mux_pin, MuxCtrlrInterface* const mux_ctrlr_intf)
    {
        // Not applicable for AnalogCtrlr
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        if(mux_ctrlr_intf == nullptr)
        {
            return GPIO_INVALID_MUX_CONTROLLER;
        }

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].attach_to_mux(mux_ctrlr_intf, mux_pin);
    }

    inline GpioReturnStatus set_pol(int id, ControllerPolarity pol)
    {
        // Not applicable for AnalogCtrlr
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_pol(pol);
    }

    inline GpioReturnStatus set_tick_rate(int id, int tick_rate)
    {
        // Not applicable for OutputCtrlr<NumPins>
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_tick_rate(tick_rate);
    }

    inline GpioReturnStatus set_notif_mode(int id, ControllerNotifMode notif_mode)
    {
        // Not applicable for OutputCtrlr<NumPins>
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_notif_mode(notif_mode);
    }

    inline GpioReturnStatus add_pins(int id,
                                     int num_pins,
                                     const uint8_t* const pin_list)
    {
        // check if num pins specifief is greater than total available pins
        if(num_pins >= NumPins)
        {
            GPIO_LOG_ERROR("Cannot add %d pins to id %d - No more pins available", num_pins, id);
            return GPIO_NO_PINS_AVAILABLE;
        }

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        // check if each pin is within range of available pins
        for(int i = 0; i < num_pins; i++)
        {
            if(pin_list[i] >= NumPins)
            {
                GPIO_LOG_ERROR("Cannot add %d pin to id %d - Pin number out of range", pin_list[i], id);
                return GPIO_NO_PINS_AVAILABLE;
            }
        }

        return _ctrlrs[ctrlr_num].add_pins(num_pins, pin_list);
    }

    inline GpioReturnStatus set_mute_status(int id, ControllerMuteStatus mute_status)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_mute_status(mute_status);
    }

    inline GpioReturnStatus set_res_diff(int analog_ctrlr_id, int adc_res_diff)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(analog_ctrlr_id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_res_diff(adc_res_diff);
    }

    inline GpioReturnStatus set_range(int id,
                                      uint32_t min_val,
                                      uint32_t max_val)
    {
        static_assert(std::is_same<CtrlrType, InputCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_range(min_val, max_val);
    }

    inline GpioReturnStatus set_debounce_mode(int id,
                                              ControllerDebounceMode debounce_mode)
    {
        static_assert(std::is_same<CtrlrType, InputCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_debounce_mode(debounce_mode);
    }

    inline GpioReturnStatus set_time_constant(int id, float time_constant)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_time_constant(time_constant);
    }

    inline void calc_and_set_adc_tick_rate(int system_tick_rate, int adc_chans_per_tick)
    {
        int adc_tick_rate = (system_tick_rate * adc_chans_per_tick)/NumPins;
        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.set_adc_sampling_rate(adc_tick_rate);
        }
    }

    inline std::pair<bool, uint32_t> get_val(int id)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return std::make_pair(false, 0);
        }

        return std::make_pair(true, _ctrlrs[ctrlr_num].get_val());
    }

    inline GpioReturnStatus set_val(int id, uint32_t val)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;;
        }

        return _ctrlrs[ctrlr_num].set_val(val);
    }

    inline void process()
    {
        for(int i = 0; i < _num_ctrlrs; i++)
        {
            _ctrlrs[i].process();
        }
    }

private:
    inline int _get_ctrlr_num(int id)
    {
        int ctrlr_num = 0;
        for(auto& ctrlr : _ctrlrs)
        {
            if(!ctrlr.is_active())
            {
                break;
            }

            if(ctrlr.get_id() == id)
            {
                return ctrlr_num;
            }

            ctrlr_num++;
        }

        return -1;
    }

    std::array<CtrlrType, NumPins> _ctrlrs;
    int _num_ctrlrs;

    GpioTxPacketFifo* _tx_packet_fifo;
};

template <class CtrlrType>
class CtrlrSet<CtrlrType, 0>
{
public:
    void set_info(uint32_t* const pin_data,
                  const uint32_t* const current_system_tick)
    {}


    void set_info(uint32_t* const pin_data,
                  const uint32_t* const current_system_tick,
                  GpioTxPacketFifo* tx_packet_fifo)
    {}

    void set_info(uint32_t* const pin_data,
                  const uint32_t* const current_system_tick,
                  GpioTxPacketFifo* tx_packet_fifo,
                  int adc_chans_per_tick)
    {}

    void init()
    {}

    constexpr int get_num_ctrlrs() const
    {
        return 0;
    }

    constexpr bool does_id_exist(int id) const
    {
        return false;
    }

    constexpr bool check_ctrlr_init_status() const
    {
        return true;
    }

    constexpr bool check_duplicated_pins() const
    {
        return false;
    }

    void reset_all_ctrlrs()
    {}

    constexpr bool reset_ctrlr(int id)
    {
        return false;
    }

    constexpr GpioReturnStatus add_ctrlr(int id, GpioHwType type)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<0>>::value);
        return GPIO_NO_PINS_AVAILABLE;
    }

    constexpr GpioReturnStatus add_ctrlr(int id)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_NO_PINS_AVAILABLE;
    }

    constexpr CtrlrType* get_ctrlr(int id)
    {
        return nullptr;
    }

    constexpr GpioReturnStatus attach_to_mux(int id, uint32_t mux_pin, MuxCtrlrInterface* const mux_ctrlr_intf)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_pol(int id, ControllerPolarity pol)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_tick_rate(int id, int tick_rate)
    {
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_notif_mode(int id, ControllerNotifMode notif_mode)
    {
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus add_pins(int id,
                                        int num_pins,
                                        const uint8_t* const pin_list)
    {
        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_mute_status(int id, ControllerMuteStatus mute_status)
    {
        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_res_diff(int analog_ctrlr_id, int adc_res_diff)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_range(int ctrlr_num,
                                         uint32_t min_val,
                                         uint32_t max_val)
    {
        // Only applicable for InputCtrlr
        static_assert(std::is_same<CtrlrType, InputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_debounce_mode(int ctrlr_num,
                                                 ControllerDebounceMode debounce_mode)
    {
        static_assert(std::is_same<CtrlrType, InputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    constexpr GpioReturnStatus set_time_constant(int id, float time_constant)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<0>>::value);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    void calc_and_set_adc_tick_rate(int system_tick_rate, int adc_chans_per_tick)
    {}

    constexpr std::pair<bool, uint32_t> get_val(int id)
    {
        return std::make_pair(false, 0);
    }

    constexpr GpioReturnStatus set_val(int id, uint32_t val)
    {
        return GPIO_INVALID_CONTROLLER_ID;
    }

    void process()
    {}
};

} // gpio

#endif // CTRLR_SET_H_