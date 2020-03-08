#ifndef SERIAL_H
#define SERIAL_H


#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <string.h>  /* Объявления строковых функций */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */
#include "FIFO.h"

// #define mode_8N1 0
// #define mode_7E1 1
// #define mode_701 2
// #define mode_7S1 3

enum serial_mode{
    mode_8N1=0,
    mode_7E1,
    mode_701,
    mode_7S1
};

#define SERIAL_DEVICE "/dev/ttyUSB0"

int serial_begin(int speed, enum serial_mode mode);
int serial_write(char* s, int _n);
void serial_flush();
int serial_read(void* data_buffer, uint8_t __n);
void serial_close();



#endif // !SERIAL_H