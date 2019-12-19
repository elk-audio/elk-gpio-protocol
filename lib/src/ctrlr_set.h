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
 * @brief Contains the definition of the CtrlrSet container class which is a
 *        container of controllers.
 *
 * @copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */

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

/**
 * @brief A Container to store a fixed size of controller types, along with
 *        utility functions to modify the underlying stored elements.
 * @tparam CtrlrType The controller type
 * @tparam NumPins The number of pins available for this type of controller.
 */
template<class CtrlrType,
         int NumPins>
class CtrlrSet
{
public:
    /**
     * @brief Function to set internal system information. This function should
     *        be called after instantiation. Only applicable when the container
     *        holds elements of output controller type (CtrlrType = OutputCtrlr)
     * @param pin_data The pointer to the memory location containing the
     *                 sampled data of the pins.
     * @param current_system_tick Pointer to the system tick counter.
     */
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

    /**
     * @brief Function to set internal system information. This function should
     *        be called after instantiation. Only applicable when the container
     *        holds elements of input controller type (CtrlrType = InputCtrlr)
     * @param pin_data The pointer to the memory location containing the
     *                 sampled data of the pins.
     * @param current_system_tick Pointer to the system tick counter.
     * @param tx_packet_fifo Pointer to an instance of a tx packet fifo.
     */
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

    /**
     * @brief Function to set internal system information. This function should
     *        be called after instantiation. Only applicable when the container
     *        holds elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param pin_data The pointer to the memory location containing the
     *                 sampled data of the pins.
     * @param current_system_tick Pointer to the system tick counter.
     * @param tx_packet_fifo Pointer to an instance of a tx packet fifo.
     * @param adc_chans_per_tick Number of adc channels sampled per system tick.
     */
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

    /**
     * @brief Function to initialize this container and set it to its default
     *        state.
     */
    inline void init()
    {
        _num_ctrlrs = 0;

        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.init();
        }
    }

    /**
     * @brief Get number of active controllers in this container.
     * @return The number of controllers.
     */
    inline int get_num_ctrlrs() const
    {
        return _num_ctrlrs;
    }

    /**
     * @brief Helper function to check if a controller of a given id exists
     *        in this container.
     * @param id The id of the controller
     * @return True if id exists, false if not.
     */
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

    /**
     * @brief Check if all active controllers in this container are initialized
     *        fully.
     * @return True if all active containers are initialized fully, false if not
     */
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

    /**
     * @brief Checks if any of the active controllers in the container share the
     * same pin.
     * @return True if there are duplicated pins, false if not.
     */
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

    /**
     * @brief Resets the value of all the controllers in this container to its
     *        default.
     */
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

    /**
     * @brief Reset the value of a controller to its default.
     * @param id The controller id.
     * @return True if controller exists, false if not
     */
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

    /**
     * @brief Add a new controller. Not applicable when the container holds
     *        elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param id The id of the new controller
     * @param type The hardware type of the new controller
     * @return GPIO_NO_PINS_AVAILABLE When all the pins have been used up.
     *         GPIO_INVALID_CONTROLLER_ID if controller id already exists.
     *         GPIO_OK on success
     */
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

        for(int i = 0; i < _num_ctrlrs; i++)
        {
            if(_ctrlrs[i].get_id() == id)
            {
                return GPIO_INVALID_CONTROLLER_ID;
            }
        }

        _ctrlrs[_num_ctrlrs].set_ctrlr_info(id, type);
        _num_ctrlrs++;

        GPIO_LOG_INFO("Added new ctrlr of id %d", id);
        return GPIO_OK;
    }

    /**
     * @brief Add a new controller. Only applicable when the container holds
     *        elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param id The id of the new controller
     * @return GPIO_NO_PINS_AVAILABLE When all the pins have been used up.
     *         GPIO_INVALID_CONTROLLER_ID if controller id already exists.
     *         GPIO_OK on success
     */
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

    /**
     * @brief Get an instance of the controller of a given id
     * @param id The id of the controller
     * @return Pointer to the instance of the controller if id exists.
     *         nullptr if id does not exist
     */
    inline CtrlrType* get_ctrlr(int id)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return nullptr;
        }

        return &_ctrlrs[ctrlr_num];
    }

    /**
     * @brief Attach a controller to a mux controller. Not applicable when the
     *        container holds elements of analog controller type
     *        (CtrlrType = AnalogCtrlr)
     * @param id The id of the controller.
     * @param mux_pin The pin of the mux controller to which the controller
     *                should be attached to.
     * @param mux_ctrlr_intf The mux controller interface
     * @return GPIO_INVALID_MUX_CONTROLLER if mux_ctrlr_intf is null
     *         GPIO_INVALID_CONTROLLER_ID if controller id does not exist
     *         Different GpioReturnStatus code otherwise
     */
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

    /**
     * @brief Set the polarity of a controller id. Not applicable when the
     *        container holds elements of analog controller type
     *        (CtrlrType = AnalogCtrlr)
     * @param id The id of the controller
     * @param pol The new polarity of the controller.
     * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist
     *         Different GpioReturnStatus code otherwise
     */
    inline GpioReturnStatus set_pol(int id, ControllerPolarity pol)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_pol(pol);
    }

    /**
     * @brief Set the tick rate of a controller. Not applicable when the
     *        container holds elements of output controller
     *        type (CtrlrType = OutputCtrlr)
     * @param id The id of the controller
     * @param tick_rate The tick rate of the controller.
     * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist
     *         Different GpioReturnStatus code otherwise
     */
    inline GpioReturnStatus set_tick_rate(int id, int tick_rate)
    {
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_tick_rate(tick_rate);
    }

    /**
     * @brief Set notification mode of a controller. Not applicable when
     *        the container holds elements of output controller type
     *        (CtrlrType = OutputCtrlr)
     * @param id The id of the controller
     * @param notif_mode The notification mode
     * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist
     *         Different GpioReturnStatus code otherwise
     */
    inline GpioReturnStatus set_notif_mode(int id, ControllerNotifMode notif_mode)
    {
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_notif_mode(notif_mode);
    }

    /**
     * @brief Add pins to a controller in the container.
     * @param id The id of the controller
     * @param num_pins The number of pins to be added.
     * @param pin_list Pointer to the list containing the pin numbers
     * @return GPIO_NO_PINS_AVAILABLE if all the pins have been used.
     *         GPIO_INVALID_CONTROLLER_ID if controller id does not exist
     *         Different GpioReturnStatus code otherwise
     */
    inline GpioReturnStatus add_pins(int id,
                                     int num_pins,
                                     const uint8_t* const pin_list)
    {
        // check if num pins specified is greater than total available pins
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

    /**
     * @brief Set the mute status of a controller in the container.
     * @param id The id of the controller
     * @param mute_status The mute status.
     * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist,
    *         different GpioReturnStatus code otherwise
     */
    inline GpioReturnStatus set_mute_status(int id, ControllerMuteStatus mute_status)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_mute_status(mute_status);
    }

    /**
    * @brief Set the difference in resolution of a controller compared with
    *        that of the ADC's resolution. Only applicable when the container
    *        holds elements of analog controller type (CtrlrType = AnalogCtrlr)
    * @param analog_ctrlr_id
    * @param adc_res_diff The difference in resolution of the controller with
    *        that of the adc
    * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist,
    *         different GpioReturnStatus code otherwise
    */
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

    /**
     * @brief Set the range of an digital input controller. Only applicable when
     *         the container holds elements of input controller type
     *        (CtrlrType = InputCtrlr)
     * @param id The controller id
     * @param min_val The minimum value of the controller
     * @param max_val The maximum value of the controller
     * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist,
     *         different GpioReturnStatus code otherwise.
     */
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

    /**
     * @brief Set the debounce mode for a digital input controller of a given ID
     * @param id The controller id
     * @param debounce_mode The debounce mode
     * @return GPIO_INVALID_CONTROLLER_ID if the id does not exist in the list
     *         of configured digital input controllers
     *         Other GpioReturnStatus code otherwise.
     */
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

    /**
     * @brief Sets the time constant for the filter for an analog controller of
     * a given ID.
     * @param id The id of the analog controller
     * @param time_constant The time constant
     * @return GPIO_INVALID_CONTROLLER_ID if the id does not exist in the list
     *         of configured analog controllers
     *         Other GpioReturnStatus code otherwise.
     */
    inline GpioReturnStatus set_time_constant(int id, float time_constant)
    {
        // Only applicable to analog controller types
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<NumPins>>::value);

        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;
        }

        return _ctrlrs[ctrlr_num].set_time_constant(time_constant);
    }

    /**
     * @brief Calculate the adc tick rate from the system tick rate and from
     *        how many channels are sampled per tick.
     * @param system_tick_rate  The system tick rate.
     * @param adc_chans_per_tick Number of channels sampled per tick.
     */
    inline void calc_and_set_adc_tick_rate(int system_tick_rate, int adc_chans_per_tick)
    {
        int adc_tick_rate = (system_tick_rate * adc_chans_per_tick)/NumPins;
        for(auto& ctrlr : _ctrlrs)
        {
            ctrlr.set_adc_sampling_rate(adc_tick_rate);
        }
    }

    /**
     * @brief Get the value of a controller
     * @param id The id of the controller
     * @return false if id does not exist, true if it does.
       @return 0 if id does not exist, controller value if it does.
     */
    inline std::pair<bool, uint32_t> get_val(int id)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return std::make_pair(false, 0);
        }

        return std::make_pair(true, _ctrlrs[ctrlr_num].get_val());
    }

    /**
     * @brief Function to the set the value of a particular controller.
     * @param id The id of the controller.
     * @param val The new value of the controller
     * @return GPIO_INVALID_CONTROLLER_ID if controller id does not exist,
     *         different GpioReturnStatus code otherwise.
     */
    inline GpioReturnStatus set_val(int id, uint32_t val)
    {
        auto ctrlr_num = _get_ctrlr_num(id);
        if(ctrlr_num < 0)
        {
            return GPIO_INVALID_CONTROLLER_ID;;
        }

        return _ctrlrs[ctrlr_num].set_val(val);
    }

    /**
     * @brief Function to process all the active controllers in the container.
     */
    inline void process()
    {
        for(int i = 0; i < _num_ctrlrs; i++)
        {
            _ctrlrs[i].process();
        }
    }

private:
    /**
     * @brief Get the controller array index from the id
     * @param id The id of the controller
     * @return The array index of the controller if controller exists
     *         -1 if not
     */
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

/**
 * @brief Template specialization of CtrlrSet when NumPins is 0. Contains
 *        default implementations of all functions and no elements.
 * @tparam CtrlrType The controller type
 */
template <class CtrlrType>
class CtrlrSet<CtrlrType, 0>
{
public:
    /**
     * @brief Function to set internal system information. This function should
     *        be called after instantiation. Only applicable when the container
     *        holds elements of output controller type (CtrlrType = OutputCtrlr)
     * @param pin_data The pointer to the memory location containing the
     *                 sampled data of the pins.
     * @param current_system_tick Pointer to the system tick counter.
     */
    void set_info(uint32_t* const pin_data,
                  const uint32_t* const current_system_tick)
    {
        static_assert(std::is_same<CtrlrType, OutputCtrlr<0>>::value);
    }

    /**
     * @brief Function to set internal system information. This function should
     *        be called after instantiation. Only applicable when the container
     *        holds elements of input controller type (CtrlrType = InputCtrlr)
     * @param pin_data The pointer to the memory location containing the
     *                 sampled data of the pins.
     * @param current_system_tick Pointer to the system tick counter.
     * @param tx_packet_fifo Pointer to an instance of a tx packet fifo.
     */
    void set_info(uint32_t* const pin_data,
                  const uint32_t* const current_system_tick,
                  GpioTxPacketFifo* tx_packet_fifo)
    {
        static_assert(std::is_same<CtrlrType, InputCtrlr<0>>::value);
    }

    /**
     * @brief Function to set internal system information. This function should
     *        be called after instantiation. Only applicable when the container
     *        holds elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param pin_data The pointer to the memory location containing the
     *                 sampled data of the pins.
     * @param current_system_tick Pointer to the system tick counter.
     * @param tx_packet_fifo Pointer to an instance of a tx packet fifo.
     * @param adc_chans_per_tick Number of adc channels sampled per system tick.
     */
    void set_info(uint32_t* const pin_data,
                  const uint32_t* const current_system_tick,
                  GpioTxPacketFifo* tx_packet_fifo,
                  int adc_chans_per_tick)
    {
        static_assert(std::is_same<CtrlrType, OutputCtrlr<0>>::value);
    }

    /**
     * @brief Function to initialize this container and set it to its default
     *        state.
     */
    void init()
    {}

    /**
     * @brief Get number of active controllers in this container.
     * @return Always returns 0 as there are no controllers in this
     *         container when number of pins is 0
     */
    constexpr int get_num_ctrlrs() const
    {
        return 0;
    }

    /**
     * @brief Helper function to check if a controller of a given id exists
     *        in this container.
     * @param id The id of the controller
     * @return Always returns false as there are no controllers in this
     *         container when number of pins is 0
     */
    constexpr bool does_id_exist(int id) const
    {
        return false;
    }

    /**
     * @brief Check if active controllers in this container are initialized
     *        fully.
     * @return Always returns true as there are no controllers in this
     *         container when number of pins is 0
     */
    constexpr bool check_ctrlr_init_status() const
    {
        return true;
    }

    /**
     * @brief Checks if any of the active controllers in the container share the
     * same pin.
     * @return Always returns false as there are no controllers in this
     *         container when number of pins is 0
     */
    constexpr bool check_duplicated_pins() const
    {
        return false;
    }

    /**
     * @brief Resets the value of all the controllers in this container to its
     *        default. As there are no controllers in this container when number
     *        of pins is 0, this function does nothing.
     */
    void reset_all_ctrlrs()
    {}

    /**
     * @brief Reset the value of a controller to its default.
     * @param id The controller id.
     * @return False as there are no controllers in this container when number
     *         of pins is 0
     */
    constexpr bool reset_ctrlr(int id)
    {
        return false;
    }

    /**
     * @brief Add a new controller. Nit applicable when the container holds
     *        elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param id The id of the new controller
     * @param type The hw type of the new controller
     * @return GPIO_NO_PINS_AVAILABLE as there are zero pins in this container.
     */
    constexpr GpioReturnStatus add_ctrlr(int id, GpioHwType type)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<0>>::value);
        return GPIO_NO_PINS_AVAILABLE;
    }

    /**
     * @brief Add a new controller. Only applicable when the container holds
     *        elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param id The id of the new controller
     * @return GPIO_NO_PINS_AVAILABLE as there are zero pins in this container.
     */
    constexpr GpioReturnStatus add_ctrlr(int id)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_NO_PINS_AVAILABLE;
    }

    /**
     * @brief Get an instance of the controller of a given id
     * @param id The id of the controller
     * @return Always returns nullptr as there are no controllers in this
     *         container when number of pins is 0
     */
    constexpr CtrlrType* get_ctrlr(int id)
    {
        return nullptr;
    }

    /**
     * @brief Attach a controller to a mux controller. Not applicable when the
     *        container holds elements of analog controller type
     *        (CtrlrType = AnalogCtrlr)
     * @param id The id of the controller.
     * @param mux_pin The pin of the mux controller to which the controller
     *                should be attached to.
     * @param mux_ctrlr_intf The mux controller interface
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus attach_to_mux(int id, uint32_t mux_pin, MuxCtrlrInterface* const mux_ctrlr_intf)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set the polarity of a controller id. Not applicable when the
     *        container holds elements of analog controller type
     *        (CtrlrType = AnalogCtrlr)
     * @param id The id of the controller
     * @param pol The new polarity of the controller.
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_pol(int id, ControllerPolarity pol)
    {
        static_assert(!std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set the tick rate of a controller. Not applicable when the
     *        container holds elements of output controller
     *        type (CtrlrType = OutputCtrlr)
     * @param id The id of the controller
     * @param tick_rate The tick rate of the controller.
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_tick_rate(int id, int tick_rate)
    {
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set notification mode of a controller. Not applicable when
     *        the container holds elements of output controller type
     *        (CtrlrType = OutputCtrlr)
     * @param id The id of the controller
     * @param notif_mode The notification mode
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_notif_mode(int id, ControllerNotifMode notif_mode)
    {
        static_assert(!std::is_same<CtrlrType, OutputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Add pins to a controller in the container.
     * @param id The id of the controller
     * @param num_pins The number of pins to be added.
     * @param pin_list Pionter to the list containing the pin numbers
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus add_pins(int id,
                                        int num_pins,
                                        const uint8_t* const pin_list)
    {
        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set the mute status of a controller in the container.
     * @param id The id of the controller
     * @param mute_status The mute status.
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_mute_status(int id, ControllerMuteStatus mute_status)
    {
        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set the difference in resolution of a controller compared with
     *        that of the ADC's resolution. Only applicable when the container
     *        holds elements of analog controller type (CtrlrType = AnalogCtrlr)
     * @param analog_ctrlr_id
     * @param adc_res_diff The difference in resolution of the controller with
     *        that of the adc
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_res_diff(int analog_ctrlr_id, int adc_res_diff)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set the range of an digital input controller. Only applicable when
     *         the container holds elements of input controller type
     *        (CtrlrType = InputCtrlr)
     * @param id The controller id
     * @param min_val The minimum value of the controller
     * @param max_val The maximum value of the controller
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_range(int id,
                                         uint32_t min_val,
                                         uint32_t max_val)
    {
        // Only applicable for InputCtrlr
        static_assert(std::is_same<CtrlrType, InputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set debounce mode for a controller. Only applicable when the
     *        container holds elements of input controller type
     *        (CtrlrType = InputCtrlr)
     * @param id The controller id
     * @param debounce_mode The debounce mode
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_debounce_mode(int id,
                                                 ControllerDebounceMode debounce_mode)
    {
        static_assert(std::is_same<CtrlrType, InputCtrlr<0>>::value);

        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Set the time constant for the filter of a controller. Only
     *        applicable when the container holds elements of analog controller
     *        type (CtrlrType = AnalogCtrlr)
     * @param id The id of the analog controller
     * @param time_constant The time constant
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_time_constant(int id, float time_constant)
    {
        static_assert(std::is_same<CtrlrType, AnalogCtrlr<0>>::value);
        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Calculate the adc tick rate from the system tick rate and from
     *        how many channels are sampled per tick. As there are no
     *        controllers in this container when number of pins is 0, this
     *        function does nothing.
     * @param system_tick_rate  The system tick rate.
     * @param adc_chans_per_tick Number of channels sampled per tick.
     */
    void calc_and_set_adc_tick_rate(int system_tick_rate, int adc_chans_per_tick)
    {}

    /**
     * @brief Get the value of a controller
     * @param id The id of the controller
     * @return An std pair of (false, 0) as there are no controllers in this
     *         container when number of pins is 0
     */
    constexpr std::pair<bool, uint32_t> get_val(int id)
    {
        return std::make_pair(false, 0);
    }

    /**
     * @brief Function to the set the value of a particular controller.
     * @param id The id of the controller.
     * @param val The new value of the controller
     * @return Always returns GPIO_INVALID_CONTROLLER_ID as there are no
     *         controllers in this container when number of pins is 0
     */
    constexpr GpioReturnStatus set_val(int id, uint32_t val)
    {
        return GPIO_INVALID_CONTROLLER_ID;
    }

    /**
     * @brief Function to process all controllers in the container. Since
     *        there are elements in this container, it does not do anything.
     */
    void process()
    {}
};

} // gpio

#endif // CTRLR_SET_H_