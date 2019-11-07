#ifndef GPIO_LOGGER_INTERFACE_H_
#define GPIO_LOGGER_INTERFACE_H_

/**
 * @brief logging macros which provide an interface to the platforms
 *        logging mechanism
 */

constexpr int GPIO_LOG_MAX_MSG_SIZE = 80;

#ifdef GPIO_LOG_LEVEL_INFO
    #define GPIO_LOG_LEVEL_WARNING
    #define GPIO_LOG_LEVEL_ERROR
#endif

#ifdef GPIO_LOG_LEVEL_WARNING
    #define GPIO_LOG_LEVEL_ERROR
#endif

namespace gpio {

enum class GpioLogLevel
{
    GPIO_LOG_INFO,
    GPIO_LOG_WARNING,
    GPIO_LOG_ERROR
};

struct GpioLogMsg
{
    GpioLogLevel level;
    std::array<char, GPIO_LOG_MAX_MSG_SIZE> msg;
};

}

#endif