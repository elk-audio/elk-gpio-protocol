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
 * @brief Contains the definitions for the internal fifos for the packets and
 *        log msgs
 *
 * @copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */

#ifndef GPIO_FIFO_H_
#define GPIO_FIFO_H_

#include <cstdarg>

#include "gpio_protocol/gpio_protocol.h"
#include "gpio_protocol_client/gpio_logger_interface.h"

namespace gpio {

/**
 * @brief Describes the log fifo sizes required depending on
 *        the log level specified
 */
#if defined(GPIO_LOG_LEVEL_INFO)
    constexpr int GPIO_LOG_FIFO_SIZE = 25;
#elif defined(GPIO_LOG_LEVEL_WARNING)
    constexpr int GPIO_LOG_FIFO_SIZE = 10;
#elif defined(GPIO_LOG_LEVEL_ERROR)
    constexpr int GPIO_LOG_FIFO_SIZE = 5;
#else
    constexpr int GPIO_LOG_FIFO_SIZE = 0;
#endif

/**
 * @brief Macro to provide the instance to the logger if any log level
 *        has been defined. If not its an empty macro and the log msgs
 *        will not be compiled into the code.
 */
#if defined(GPIO_LOG_LEVEL_INFO) || defined(GPIO_LOG_LEVEL_WARNING) || defined(GPIO_LOG_LEVEL_ERROR)
    #define GPIO_WITH_LOGGING
    #define GPIO_LOG_RESET (GpioLogFifo::get_logger_instance())->reset();
    #define GPIO_LOG_HAS_NEW_MSG (GpioLogFifo::get_logger_instance())->has_new_elements();
    #define GPIO_LOG_GET_MSG(msg) (GpioLogFifo::get_logger_instance())->get_log_msg(msg);
#else
    #define GPIO_LOG_RESET
    #define GPIO_LOG_HAS_NEW_MSG false
    #define GPIO_LOG_GET_MSG(msg) nullptr
#endif

#ifdef GPIO_LOG_LEVEL_INFO
    #define GPIO_LOG_INFO(...) (GpioLogFifo::get_logger_instance())->log_info(__VA_ARGS__)
#else
    #define GPIO_LOG_INFO(...)
#endif

#ifdef GPIO_LOG_LEVEL_WARNING
    #define GPIO_LOG_WARNING(...) (GpioLogFifo::get_logger_instance())->log_warn(__VA_ARGS__)
#else
    #define GPIO_LOG_WARNING(...)
#endif

#ifdef GPIO_LOG_LEVEL_ERROR
    #define GPIO_LOG_ERROR(...) (GpioLogFifo::get_logger_instance())->log_error(__VA_ARGS__)
#else
    #define GPIO_LOG_ERROR(...)
#endif

/**
 * @brief Macro to retrieve the log msg from the arguments and print them into
 *        a char buffer
 */
#define STORE_LOG_MSG(dst, src) \
    std::memset(static_cast<void*>(dst), 0, GPIO_LOG_MAX_MSG_SIZE); \
    va_list args; \
    va_start(args, src); \
    std::vsnprintf(dst, GPIO_LOG_MAX_MSG_SIZE, src, args); \
    va_end(args);\

/**
 * @brief Macro which prevents allocation of GpioLogFifo if it is not defined.
 *        This macro is defined if GPIO_LOG_LEVEL is any of INFO, WARNING and ERROR
 */
#ifdef GPIO_WITH_LOGGING

/**
 * Class which helps in the creation and queing of log messages.
 */
class GpioLogFifo
{
public:
    /**
     * @brief Get the pointer of the singleton logger instance
     * @return BaseLogger* pointer to logger instance
     */
    inline static GpioLogFifo* get_logger_instance()
    {
        static GpioLogFifo log_fifo;
        return &log_fifo;
    }

    /**
     * @brief Reset the logging queue.
     */
    inline void reset()
    {
        _head = 0;
        _tail = 0;
        _has_new_elem = 0;
    }

    /**
     * @brief function to log messages of log level info and store it into the
     *        queue
     * @param msg The message
     */
    inline void log_info(__attribute__((unused)) const char* msg, ...)
    {
        GpioLogMsg& gpio_log_data = _fifo[_head];
        gpio_log_data.level = GpioLogLevel::GPIO_LOG_INFO;
        char* dst = gpio_log_data.msg.data();
        STORE_LOG_MSG(dst, msg);
        _update_head();
    }

    /**
     * @brief function to log messages of log level warning and store it into the
     *        queue
     * @param msg The message
     */
    inline void log_warn(__attribute__((unused)) const char* msg, ...)
    {
        GpioLogMsg& gpio_log_data = _fifo[_head];
        gpio_log_data.level = GpioLogLevel::GPIO_LOG_WARNING;
        char* dst = gpio_log_data.msg.data();
        STORE_LOG_MSG(dst, msg);
        _update_head();
    }

    /**
     * @brief function to log messages of log level error and store it into the
     *        queue
     * @param msg The message
     */
    inline void log_error(__attribute__((unused)) const char* msg, ...)
    {
        GpioLogMsg& gpio_log_data = _fifo[_head];
        gpio_log_data.level = GpioLogLevel::GPIO_LOG_ERROR;
        char* dst = gpio_log_data.msg.data();
        STORE_LOG_MSG(dst, msg);
        _update_head();
    }

    /**
     * @brief Get the log message from the fifo tail
     * @param The pointer to store the address of the new log message, if
     *        new messages exists, else a nullptr is stored.
     * @return The pointer to the gpio log message
     */
    inline bool get_log_msg(GpioLogMsg** msg)
    {
        if(!_has_new_elem)
        {
            *msg = nullptr;
            return false;
        }

        *msg = &_fifo[_tail];
        _update_tail();
        return true;
    }

private:
    /**
     * @brief Helper function to update the head of the fifo
     */
    inline void _update_head()
    {
        _head++;
        if(_head == GPIO_LOG_FIFO_SIZE)
        {
            _head = 0;
        }

        _has_new_elem = true;
    }

    /**
     * @brief Helper function to update the tail of the fifo
     */
    inline void _update_tail()
    {
        _tail++;
        if(_tail == GPIO_LOG_FIFO_SIZE)
        {
            _tail = 0;
        }

        if(_tail != _head)
        {
            _has_new_elem = true;
        }
        else
        {
            _has_new_elem =  false;
        }
    }

    int _head;
    int _tail;
    bool _has_new_elem;

    std::array<GpioLogMsg, GPIO_LOG_FIFO_SIZE> _fifo;
};

#endif

/**
 * @brief Specifies the size of the TX packet fifo. This assumes beyond the worst case
 *        size required as each controller generates atleast 2 packets per
 *        system tick.
 */
constexpr int GPIO_PACKET_FIFO_SIZE = 160;

/**
 * @brief Class which handles the creation and queuing of tx gpio packets from
 *        the gpio client which are generated every system tick. As the gpio
 *        client can generate anywhere from 0 to NUM_CONTROLLERS amount of
 *        packets, this fifo be emptied every system tick.
 *        It is not meant to be a queue but as a simple buffer between the host
 *        and the gpio client.
 */
class GpioTxPacketFifo
{
public:
    /**
     * @brief Reset the fifo.
     */
    inline void reset()
    {
        _head = 0;
        _tail = 0;
    }

    /**
     * @brief Get the packet at the current tail of the fifo. If a packet exists
     *        it will return true and store the address of the new packet in
     *        packet. Else it will return false and store nullptr in packet.
     * @param pointer to access the gpiopacket,
     * @return True if packet exists, false if not
     */
    inline bool get_packet(GpioPacket** packet)
    {
        if(!_has_new_elem)
        {
            *packet = nullptr;
            return false;
        }

        *packet = &_fifo[_tail];
        _update_tail();
        return true;
    }

    /**
     * @brief Create an ack packet and insert it into the fifo.
     * @param status The GpioReturnStatus code
     * @param current_system_tick The system tick at which this packet is
     *        created
     * @param seq_num The sequence number of the acq packet to which this ack
     *        packet is responding to.
     */
    inline void send_ack(const GpioReturnStatus status,
                         uint32_t current_system_tick,
                         uint32_t seq_num)
    {
        if(_check_overflow())
        {
            GPIO_LOG_ERROR("Fatal : Cannot generate ack packet. Gpio"
                           " client tx packet fifo is full!");

            GPIO_LOG_ERROR("Increase queue size or drain the buffer, Dropping"
                           "packet");
        }

        auto& packet = _fifo[_head];
        clear_packet(packet);

        packet.command = GPIO_ACK;
        packet.timestamp = current_system_tick;
        packet.payload.gpio_ack_data.returned_seq_no = seq_num;
        packet.payload.gpio_ack_data.gpio_return_status = status;

        _update_head();
    }

    /**
     * @brief Create a board information packet and insert it into a queue
     * @param current_system_tick The system tick at which this packet is created.
     * @param num_inputs The number of digital input pins.
     * @param num_outputs The number of digital output pins
     * @param num_analog The number if analog input pins.
     * @param adc_res_in_bits The resolution of the ADC
     */
    inline void send_board_info(uint32_t current_system_tick,
                                uint32_t num_inputs,
                                uint32_t num_outputs,
                                uint32_t num_analog,
                                uint32_t adc_res_in_bits)
    {
        if(_check_overflow())
        {
            GPIO_LOG_ERROR("Fatal : Cannot generate ack packet. Gpio"
                           " client tx packet fifo is full!");

            GPIO_LOG_ERROR("Increase queue size or drain the buffer, Dropping"
                           "packet");
        }

        auto& packet = _fifo[_head];
        clear_packet(packet);

        packet.command = GPIO_CMD_SYSTEM_CONTROL;
        packet.sub_command = GPIO_SUB_CMD_GET_BOARD_INFO;
        packet.timestamp = current_system_tick;
        packet.sequence_no = 0;

        auto& board_info = packet.payload.gpio_board_info_data;
        board_info.num_digital_input_pins = num_inputs;
        board_info.num_digital_output_pins = num_outputs;
        board_info.num_analog_input_pins = num_analog;
        board_info.adc_res_in_bits = adc_res_in_bits;

        _update_head();
    }

    /**
     * @brief Create a send value packet and insert it into the queue
     * @param id The id of the controller whose value is to be sent.
     * @param val The value of the controller
     * @param current_system_tick The system tick when this packet is created.
     */
    inline void send_val(int id, uint32_t val, uint32_t current_system_tick)
    {
        if(_check_overflow())
        {
            GPIO_LOG_ERROR("Fatal : Cannot generate ack packet. Gpio"
                           " client tx packet fifo is full!");

            GPIO_LOG_ERROR("Increase queue size or drain the buffer, Dropping"
                           "packet");
        }

        auto& packet = _fifo[_head];
        clear_packet(packet);

        packet.command = GPIO_CMD_GET_VALUE;
        packet.timestamp = current_system_tick;
        packet.sequence_no = 0;
        packet.payload.gpio_value_data.controller_id = (uint8_t)id;
        packet.payload.gpio_value_data.controller_val = val;

        _update_head();
    }

    /**
     * @brief Helper function to clear a gpio packet.
     * @param packet The packet to be cleared.
     */
    inline void clear_packet(GpioPacket& packet)
    {
        std::memset(static_cast<void*>(&packet), 0, GPIO_PACKET_SIZE);
    }

private:
    /**
     * @brief Helper function to update the fifo head.
     */
    inline void _update_head()
    {
        _head++;
        _has_new_elem = true;
        if(_head == GPIO_PACKET_FIFO_SIZE)
        {
            _head = 0;
        }
    }

    /**
     * @brief Helper function to update the fifo tail.
     */
    inline void _update_tail()
    {
        _tail++;
        if(_tail == GPIO_PACKET_FIFO_SIZE)
        {
            _tail = 0;
        }

        if(_tail != _head)
        {
            _has_new_elem = true;
        }
        else
        {
            _has_new_elem = false;
        }
    }

    inline bool _check_overflow()
    {
        return (_has_new_elem && (_head == _tail));
    }

    int _head;
    int _tail;
    bool _has_new_elem;

    GpioPacket _fifo[GPIO_PACKET_FIFO_SIZE];
};

} // gpio

#endif