#include "log.h"
#include <stdio.h>
#include <stdarg.h>
#include "HardwareSerial.h"
#include <avr/pgmspace.h>

#define TEMP_BUFFER_SIZE (LOG_BUFFER_SIZE * 2)

int __log_level__ = LOG_LEVEL_INFO;
static char l_buffer[LOG_BUFFER_SIZE + 1];
static char *temp_str = NULL;
static char *temp_buff = NULL;

void __printf__(const char *format, ...)
{
    va_list arg_ptr;
    memset(l_buffer, 0, LOG_BUFFER_SIZE + 1);
    va_start(arg_ptr, format);
    vsnprintf(l_buffer, LOG_BUFFER_SIZE, format, arg_ptr);
    va_end(arg_ptr);
    Serial.print(l_buffer);
}

void __printf_from_flash__(const void *format, ...)
{
    va_list arg_ptr;
    va_start(arg_ptr, format);

    temp_buff = new char[LOG_BUFFER_SIZE + 1];
    if (!temp_buff)
        return;

    memset(temp_buff, 0, LOG_BUFFER_SIZE + 1);
    memset(l_buffer, 0, LOG_BUFFER_SIZE + 1);

    memcpy_P(temp_buff, format, strnlen_P(format, LOG_BUFFER_SIZE));

    vsnprintf(l_buffer, LOG_BUFFER_SIZE, temp_buff, arg_ptr);

    va_end(arg_ptr);

    delete temp_buff;
    temp_buff = NULL;

    Serial.print(l_buffer);
}
/**
 * @brief __printf__
 * Custom __printf__ function, doesn't test much
 * @param ... 
 */
void __printf__(...)
{
    va_list arg_ptr;
    memset(l_buffer, 0, LOG_BUFFER_SIZE + 1);
    temp_str = va_arg(arg_ptr, char *);
    vsnprintf(l_buffer, LOG_BUFFER_SIZE, temp_str, (va_list)(arg_ptr + strlen(temp_str) + 1));
    va_end(arg_ptr);
    Serial.print(temp_str);
}

void __set_log_level__(log_level_t level)
{
    __log_level__ = level;
}