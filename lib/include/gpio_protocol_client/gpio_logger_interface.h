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
 * File which contains the interface for the logging mechanism
 */
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

/**
 * @brief Enum to denote different logging levels
 */
enum class GpioLogLevel
{
    GPIO_LOG_INFO,
    GPIO_LOG_WARNING,
    GPIO_LOG_ERROR
};

/**
 * @brief Data structure to hold a gpio log msg
 */
struct GpioLogMsg
{
    GpioLogLevel level;
    std::array<char, GPIO_LOG_MAX_MSG_SIZE> msg;
};

}

#endif // GPIO_LOGGER_INTERFACE_H_