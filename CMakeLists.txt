cmake_minimum_required(VERSION 3.8)
project(gpio_protcol)

add_library(gpio_protocol INTERFACE)
target_include_directories(gpio_protocol INTERFACE include)

option(GPIO_PROTOCOL_BUILD_CLIENT_LIB "Build Gpio proctocol client library" OFF)

if (${GPIO_PROTOCOL_BUILD_CLIENT_LIB})
    set(GPIO_LOG_LEVEL none CACHE STRING "Set log level. Options are \"none\", \"info\", \"warning\" and \"error\".")

    add_library(gpio_protocol_client INTERFACE)
    target_include_directories(gpio_protocol_client INTERFACE lib/include/
                               lib/common/
                               lib/src/)

    if (GPIO_LOG_LEVEL MATCHES "info")
        set(GPIO_LOG_LEVEL_DEFINE "-DGPIO_LOG_LEVEL_INFO")
    elseif (GPIO_LOG_LEVEL MATCHES "warning")
        set(GPIO_LOG_LEVEL_DEFINE "-DGPIO_LOG_LEVEL_WARNING")
    elseif (GPIO_LOG_LEVEL MATCHES "error")
        set(GPIO_LOG_LEVEL_DEFINE "-DGPIO_LOG_LEVEL_ERROR")
    endif ()

    target_compile_options(gpio_protocol_client INTERFACE -Wall -Wextra -Wno-sign-compare)
    target_compile_definitions(gpio_protocol_client INTERFACE ${GPIO_LOG_LEVEL_DEFINE})
endif ()