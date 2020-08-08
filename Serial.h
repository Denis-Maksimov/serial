#ifndef SERIAL_H
#define SERIAL_H


// #include <stdio.h>   /* Стандартные объявления ввода/вывода */
// #include <string.h>  /* Объявления строковых функций */
// #include <unistd.h>  /* Объявления стандартных функций UNIX */
// #include <fcntl.h>   /* Объявления управления файлами */
// #include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */
// #include "FIFO.h"
#include "queue.h"

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

struct Serial
{
    int fd;
    struct termios options;
    struct queue* rx_buffer;
};

SERIAL_API struct Serial* serial_begin(const char* DEVISE,int speed, enum serial_mode mode);

SERIAL_API int serial_write(struct Serial* port, char* s, int _n);
SERIAL_API void serial_flush(struct Serial* serial);
SERIAL_API int serial_read(struct Serial* serial,void* data_buffer, u8 __n);
SERIAL_API void serial_close(struct Serial* serial);



#endif // !SERIAL_H
