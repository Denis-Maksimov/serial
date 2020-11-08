#ifndef SERIAL_H
#define SERIAL_H


#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <stdlib.h>  /* Функции стандартной библиотеки  */
#include <string.h>  /* Объявления строковых функций */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */

#include <sys/select.h>
#include <stdint.h>
#include <stddef.h>    

#define SERIAL_API      extern

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef signed char s8;
typedef signed short s16;
typedef signed int s32;

enum serial_mode{
    mode_8N1=0,
    mode_7E1,
    mode_701,
    mode_7S1
};

#define SERIAL_DEVICE "/dev/ttyUSB0"


typedef struct Serial
{
    int fd;
    struct termios options;
    struct timeval tv;
    // struct queue* rx_buffer;
    u8 rx_buffer[256];
    void(*in_handler)(struct Serial* serial);      //обработчик событий ввода
    void(*out_handler)(struct Serial* serial);     //обработчик событий вывода
    int(*timeout_handler)(struct Serial* serial);    //обработчик таймаута раунда (if not 0 завершает сервер)
}Serial;

typedef void(*serial_handler_t)(Serial* serial);
typedef int(*serial_to_handler_t)(Serial* serial);


SERIAL_API void serial_begin(Serial* hSerial, const char* DEVISE,int speed, enum serial_mode mode);
SERIAL_API void serial_set_speed(Serial* serial,int speed);
SERIAL_API int serial_write(Serial* port, char* s, int _n);
SERIAL_API ssize_t serial_read(Serial* serial,void* data_buffer, u8 __n);
SERIAL_API void serial_close(Serial* serial);
SERIAL_API void serial_set_period_to(Serial* serial,__time_t seconds, __suseconds_t useconds);
SERIAL_API void serial_set_default_handlers(Serial* serial);

SERIAL_API int serial_work(Serial* port);
#endif // !SERIAL_H
