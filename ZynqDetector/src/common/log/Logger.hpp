#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <memory>
#include <xil_printf.h>

#include "semphr.h"

#include "Register.hpp"

#define RED_TEXT    "\x1b[31m"
#define YELLOW_TEXT "\x1b[33m"
#define RESET_TEXT  "\x1b[0m"

template <typename Owner>
class Logger {
public:
    enum LogType {
        LOG_ERROR_TYPE = 0x01,
        LOG_WARN_TYPE  = 0x02,
        LOG_DEBUG_TYPE = 0x04
    };

    Logger( std::shared_ptr<Register<Owner>> reg);

    void set_log_control( uint8_t control );
    uint8_t read_log_control();

    void log_error( const char *format, ... );
    void log_error( uint32_t error_code, const char *format, ... );
    void log_warn( const char *format, ... );
    void log_debug( const char *format, ... );

private:

    void xvprintf(const char* format, va_list args);

    void log_va(
        LogType type,
        const char* color,
        const char *format,
        va_list args
    );

    std::shared_ptr<Register<Owner>> reg_;

    uint8_t control_word_;
    xSemaphoreHandle mutex_;

    const char *log_leaders_[3] =
    {
        "[ERROR] %s:%d: ",
        "[WARNING] %s:%d ",
        "[DEBUG] %s:%d : "
    };

    void log(LogType type, const char *format, ...);
    void log(LogType type, char* color, const char *format, ...);
};


#include "Logger.tpp"
