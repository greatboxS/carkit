/**
 * @file log.h
 * @author greatboxs <https://github.com/greatboxs>
 * @brief 
 * @version 0.1
 * @date 2021-11-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**
 * Notes: Print float setting for arduino
 * 
 * C:\Users*\AppData\Local\Arduino15\packages*\hardware\avr**
 * or
 * C:\Program Files (x86)\Arduino\hardware\arduino\avr\
 * 
 * Add the file “platform.local.txt” and in that file adding the following line:
 * compiler.c.elf.extra_flags=-Wl,-u,vfprintf -lprintf_flt -lm
 * 
 */
#ifndef __LOG_H__
#define __LOG_H__

#define LOG_BUFFER_SIZE (256U)

enum log_level_t
{
    LOG_LEVLE_NONE,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_TRACE,
    LOG_LEVEL_ERROR,
};

extern int __log_level__;
void __printf__(...);
void __printf__(const char *format, ...);
void __printf_from_flash__(const void *format, ...);
void __set_log_level__(log_level_t level);

/**
 * This function is normal LOG, it will print a string stored in SRAM
 * It will cost more SRAM than FLOG but will make the function faster
 * Eg: LOG_I("Your string %d\n", ur_arg);
 */
#define LOG_I(...)                       \
    if (__log_level__ >= LOG_LEVEL_INFO) \
        __printf__(__VA_ARGS__);

#define LOG_W(x, ...)                    \
    if (__log_level__ >= LOG_LEVEL_WARN) \
        __printf__(x, __VA_ARGS__);

#define LOG_T(x, ...)                     \
    if (__log_level__ >= LOG_LEVEL_TRACE) \
        __printf__(x, __VA_ARGS__);

#define LOG_E(x, ...)                     \
    if (__log_level__ >= LOG_LEVEL_ERROR) \
        __printf__(x, __VA_ARGS__);

/**
 * String is store in program memory, so this method will save 
 * the SRAM capacity but it will make the function more slower
 * We have define String with F("Your string")
 * Eg: FLOG(F("Your string %d"), arg);
 */
#define FLOG_I(x, ...)                   \
    if (__log_level__ >= LOG_LEVEL_INFO) \
        __printf_from_flash__(x, __VA_ARGS__);

#define FLOG_W(x, ...)                   \
    if (__log_level__ >= LOG_LEVEL_WARN) \
        __printf_from_flash__(x, __VA_ARGS__);

#define FLOG_T(x, ...)                    \
    if (__log_level__ >= LOG_LEVEL_TRACE) \
        __printf_from_flash__(x, __VA_ARGS__);

#define FLOG_E(x, ...)                    \
    if (__log_level__ >= LOG_LEVEL_ERROR) \
        __printf_from_flash__(x, __VA_ARGS__);
#endif // __LOG_H__