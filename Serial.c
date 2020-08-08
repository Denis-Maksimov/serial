#include "Serial.h"
#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <stdlib.h>  /* Функции стандартной библиотеки  */
#include <string.h>  /* Объявления строковых функций */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <termios.h> /* Объявления управления POSIX-терминалом */

#define c_new(t)     ((t*)malloc(sizeof(t)))
#define c_new_n(t,n)     ((t*)malloc(sizeof(t)*n))

// static uint32_t serial_FIFO=0;
// static int fd=0;
// static char buf[32]={0};
// static const char DEVISE[] ="/dev/ttyUSB0";




//-----------------------------------------------------------------

static void __set_speed(int speed,struct termios* options){
    //-- Установка скорости передачи ...
            speed_t __speed=0;
            switch (speed)
            {
            case 50:
                __speed = B50;
                break;
            case 75:
                __speed = B75;
                break;
            case 110:
                __speed = B110;
                break;
            case 134:
                __speed = B134;
                break;
            case 150:
                __speed = B150;
                break;
            case 200:
                __speed = B200;
                break;
            case 300:
                __speed = B300;
                break;
            case 600:
                __speed = B600;
                break;
            case 1200:
                __speed = B1200;
            case 1800:
                __speed = B1800;
                break;
            case 2400:
                __speed = B2400;
                break;
            case 4800:
                __speed = B4800;
                break;
            case 9600:
                __speed = B9600;
                break;
            case 19200:
                __speed = B19200;
                break;
            case 38400:
                __speed = B38400;
                break;
            case 57600:
                __speed = B57600;
                break;
            case 115200:
                __speed = B115200;
                break;
            case 230400:
                __speed = B230400;
                break;
            case 460800:
                __speed = B460800;
                break;
            case 500000:
                __speed = B500000;
                break;
            case 576000:
                __speed = B576000;
                break;
            case 921600:
                __speed = B921600;
                break;
            case 1000000:
                __speed = B1000000;
                break;
            case 1152000:
                __speed = B1152000;
                break;
            case 1500000:
                __speed = B1500000;
                break;
            case 2000000:
                __speed = B2000000;
                break;
            case 2500000:
                __speed = B2500000;
                break;
            case 3000000:
                __speed = B3000000;
                break;
            case 3500000:
                __speed = B3500000;
                break;
            case 4000000:
                __speed = B4000000;
                break;
            
            default:
                speed--;
            }
            cfsetispeed(options, __speed);
            cfsetospeed(options, __speed);
}
void serial_set_speed(struct Serial* serial,int speed){
    __set_speed(speed, &(serial->options));
}
//-----------------------------------------------------------------
static void __set_mode(enum serial_mode mode,struct termios* options)
{
    switch (mode)
            {
            case mode_8N1:
                //    Отсутствие проверки на четность (8N1):
                options->c_cflag &= ~PARENB;     /* no parity check */
                options->c_cflag &= ~CSTOPB;     /* -__- */
                options->c_cflag &= ~CSIZE;      /* Маскирование битов размера символов */
                options->c_cflag |= CS8;         /* Установка 8 битов данных */
                break;
            case mode_7E1:
                //    Проверка на четность (7E1):
                options->c_cflag |= PARENB;
                options->c_cflag &= ~PARODD;
                options->c_cflag &= ~CSTOPB;
                options->c_cflag &= ~CSIZE;
                options->c_cflag |= CS7;
                break;
            case mode_701:
                //    Проверка на нечетность (7O1):
                options->c_cflag |= PARENB;
                options->c_cflag |= PARODD;
                options->c_cflag &= ~CSTOPB;
                options->c_cflag &= ~CSIZE;
                options->c_cflag |= CS7;
                break;
            case mode_7S1:
                //    Пробел (space parity) бита четности устанавливается также как и отсутствие проверки на четность (7S1):
                options->c_cflag &= ~PARENB;
                options->c_cflag &= ~CSTOPB;
                options->c_cflag &= ~CSIZE;
                options->c_cflag |= CS8;
                break;

            default:
                mode=0;
            }

}
//-----------------------------------------------------------------
struct Serial* serial_begin(const char* DEVISE,int speed, enum serial_mode mode)
{


    // serial_FIFO = new_FIFO(100);
    struct Serial* hSerial=c_new(struct Serial);
    hSerial->rx_buffer=init_queue(256);
 
    hSerial->fd = open( DEVISE, O_RDWR |    // read &FIFO_mgr.FIFOs[FIFO_mgr.n_FIFOs-1 write
                            O_NOCTTY | //эта программа не хочет быть управляющим терминалом для этого порта.
                            O_NDELAY); //эта программа не заботится о состоянии сигнала DCD, т.е. что другой конец линии запущен
    if (hSerial->fd == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
        printf("%i", hSerial->fd);
        fflush(stdout);
        return 0;

    } else {

        

        fcntl(hSerial->fd, F_SETFL, O_NONBLOCK);
        // struct termios options;

        //-- Получение текущих опций для порта... 
        tcgetattr(hSerial->fd, &(hSerial->options));


            
            
            __set_speed(speed, &(hSerial->options));

            
            // Разрешение приемника и установка локального режима...
            hSerial->options.c_cflag |= (CLOCAL | CREAD);


            __set_mode(mode, &(hSerial->options));
            
            // Выбор неканонического (Raw) ввода
            hSerial->options.c_iflag = IGNPAR;
            hSerial->options.c_oflag = 0;
            hSerial->options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            tcflush(hSerial->fd, TCIFLUSH);

        // Установка новых опций для порта...
        tcsetattr(hSerial->fd, TCSANOW, &(hSerial->options));

    }
    
    return hSerial;
}
  
//-----------------------------------------------------------------
int serial_write(struct Serial* serial,char* s, int _n)
{
    int n = write(serial->fd, s, _n);
    if (n < 0)
        fputs("write_failed!\n", stderr);
    if (n==0) puts("no_w_data");
    return n;
}
//-----------------------------------------------------------------
void serial_flush(struct Serial* serial)
{
    
    int res = 1;
    u8 buf;
    //--заполняем FIFO-BUFFER пока данные доступны
    while(res && !isfull_queue(serial->rx_buffer))
    {
        sleep(0);
        res = read(serial->fd,&buf,1);
        if(res<0)
        {   
            fputs("error! cannot read!",stderr);
            return;
        }
        if (insert_queue(serial->rx_buffer, buf)) return;
        // FIFO_write(buf, res, serial_FIFO);
    }

    

}


//-----------------------------------------------------------------
//Предварительно нужно обновить данные от порта через serial_flush();
int serial_read(struct Serial* serial,void* data_buffer, u8 __n)
{
    serial_flush(serial);
    int i=0;
    int rv=serial->rx_buffer->count;
    while((!isempty_queue(serial->rx_buffer))&&(i<__n) )
    {
        ((u8*)data_buffer)[i]=pop_queue(serial->rx_buffer);
        i++;
    }
    return rv;

}


//-----------------------------------------------------------------
void serial_close(struct Serial* serial)
{
    close(serial->fd);
    free_queue(serial->rx_buffer);
    free(serial);

}
//------------------------------------------------------------------

#undef c_new   
#undef c_new_n

