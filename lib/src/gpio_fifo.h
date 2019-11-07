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
    #define GPIO_LOG_GET_MSG (GpioLogFifo::get_logger_instance())->get_log_msg();
#else
    #define GPIO_LOG_RESET
    #define GPIO_LOG_HAS_NEW_MSG false
    #define GPIO_LOG_GET_MSG nullptr
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
 * @brief Specifies the size of the TX packet fifo. this assumes the worst case
 *        size required i.e that there are so many controllers sending packets out
 *        in a single system tick.
 */
constexpr int GPIO_PACKET_FIFO_SIZE = 64;

class GpioTxPacketFifo
{
public:
    inline void reset()
    {
        _head = 0;
        _tail = 0;
    }

    inline bool has_new_elements()
    {
        return _has_new_elem;
    }

    inline GpioPacket* get_packet()
    {
        auto packet = &_fifo[_tail];
        _update_tail();
        return packet;
    }

    inline void send_ack(const GpioReturnStatus status,
                         uint32_t current_system_tick,
                         uint32_t seq_num)
    {
        auto& packet = _fifo[_head];
        clear_packet(packet);

        packet.command = GPIO_ACK;
        packet.timestamp = current_system_tick;
        packet.payload.gpio_ack_data.returned_seq_no = seq_num;
        packet.payload.gpio_ack_data.gpio_return_status = status;

        _update_head();
    }

    inline void send_board_info(uint32_t current_system_tick,
                                uint32_t num_inputs,
                                uint32_t num_outputs,
                                uint32_t num_analog,
                                uint32_t adc_res_in_bits)
    {
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

    inline void send_val(int id, uint32_t val, uint32_t current_system_tick)
    {
        auto& packet = _fifo[_head];
        clear_packet(packet);

        packet.command = GPIO_CMD_GET_VALUE;
        packet.timestamp = current_system_tick;
        packet.sequence_no = 0;
        packet.payload.gpio_value_data.controller_id = (uint8_t)id;
        packet.payload.gpio_value_data.controller_val = val;

        _update_head();
    }

    inline void clear_packet(GpioPacket& packet)
    {
        std::memset(static_cast<void*>(&packet), 0, GPIO_PACKET_SIZE);
    }

private:
    inline void _update_head()
    {
        _head++;
        _has_new_elem = true;
        if(_head == GPIO_PACKET_FIFO_SIZE)
        {
            _head = 0;
        }
    }

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

    int _head;
    int _tail;
    bool _has_new_elem;

    GpioPacket _fifo[GPIO_PACKET_FIFO_SIZE];
};

/**
 * @brief Macro which prevents allocation of GpioLogFifo if it is not defined.
 *        This macro is defined if GPIO_LOG_LEVEL is any of INFO, WARNING and ERROR
 */
#ifdef GPIO_WITH_LOGGING


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

    inline void reset()
    {
        _head = 0;
        _tail = 0;
        _has_new_elem = 0;
    }

    inline bool has_new_elements()
    {
        return _has_new_elem;
    }

    inline void log_info(__attribute__((unused)) const char* msg, ...)
    {
        GpioLogMsg& gpio_log_data = _fifo[_head];
        gpio_log_data.level = GpioLogLevel::GPIO_LOG_INFO;
        char* dst = gpio_log_data.msg.data();
        STORE_LOG_MSG(dst, msg);
        _update_head();
    }

    inline void log_warn(__attribute__((unused)) const char* msg, ...)
    {
        GpioLogMsg& gpio_log_data = _fifo[_head];
        gpio_log_data.level = GpioLogLevel::GPIO_LOG_WARNING;
        char* dst = gpio_log_data.msg.data();
        STORE_LOG_MSG(dst, msg);
        _update_head();
    }

    inline void log_error(__attribute__((unused)) const char* msg, ...)
    {
        GpioLogMsg& gpio_log_data = _fifo[_head];
        gpio_log_data.level = GpioLogLevel::GPIO_LOG_ERROR;
        char* dst = gpio_log_data.msg.data();
        STORE_LOG_MSG(dst, msg);
        _update_head();
    }

    GpioLogMsg* get_log_msg()
    {
        auto msg = &_fifo[_tail];
        _update_tail();
        return msg;
    }

private:
    inline void _update_head()
    {
        _head++;
        if(_head == GPIO_LOG_FIFO_SIZE)
        {
            _head = 0;
        }

        _has_new_elem = true;
    }

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

} // gpio

#endif