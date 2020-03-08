#include "Serial.h"

static uint32_t serial_FIFO=0;
static int fd=0;
static char buf[32]={0};
static const char DEVISE[] ="/dev/ttyUSB0";



//-----------------------------------------------------------------
void __set_speed(int speed,struct termios* options){
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
//-----------------------------------------------------------------
__set_mode(enum serial_mode mode,struct termios* options)
{
    switch (mode)
            {
            case 0:
                //    Отсутствие проверки на четность (8N1):
                options->c_cflag &= ~PARENB;     /* no parity check */
                options->c_cflag &= ~CSTOPB;     /* -__- */
                options->c_cflag &= ~CSIZE;      /* Маскирование битов размера символов */
                options->c_cflag |= CS8;         /* Установка 8 битов данных */
                break;
            case 1:
                //    Проверка на четность (7E1):
                options->c_cflag |= PARENB;
                options->c_cflag &= ~PARODD;
                options->c_cflag &= ~CSTOPB;
                options->c_cflag &= ~CSIZE;
                options->c_cflag |= CS7;
                break;
            case 2:
                //    Проверка на нечетность (7O1):
                options->c_cflag |= PARENB;
                options->c_cflag |= PARODD;
                options->c_cflag &= ~CSTOPB;
                options->c_cflag &= ~CSIZE;
                options->c_cflag |= CS7;
                break;
            case 3:
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
int serial_begin(int speed, enum serial_mode mode)
{


    serial_FIFO = new_FIFO(100);
 
    fd = open( DEVISE, O_RDWR |    // read &FIFO_mgr.FIFOs[FIFO_mgr.n_FIFOs-1 write
                            O_NOCTTY | //эта программа не хочет быть управляющим терминалом для этого порта.
                            O_NDELAY); //эта программа не заботится о состоянии сигнала DCD, т.е. что другой конец линии запущен
    if (fd == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
        printf("%i", fd);
        fflush(stdout);
        return -1;

    } else {

        

        fcntl(fd, F_SETFL, O_NONBLOCK);
        struct termios options;

        //-- Получение текущих опций для порта... 
        tcgetattr(fd, &options);


            
            
            __set_speed(speed, &options);

            
            // Разрешение приемника и установка локального режима...
            options.c_cflag |= (CLOCAL | CREAD);


            __set_mode(mode, &options);
            
           
                                        /*   
                                        Ich muss durch den Monsun
                                        Hinter die Welt
                                        Ans Ende der Zeit
                                        Bis kein Regen mehr fällt
                                        Gegen den Sturm
                                        Am Abgrund entlang
                                        Und wenn ich nich' mehr kann, denk' ich daran
                                        Irgendwann laufen wir zusammen
                                        Durch den Monsun...

                                        Чёт на 2007 потянуло, скоро отпустит
                                        */
           

            // Выбор неканонического (Raw) ввода
            options.c_iflag = IGNPAR;
            options.c_oflag = 0;
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            tcflush(fd, TCIFLUSH);

        // Установка новых опций для порта...
        tcsetattr(fd, TCSANOW, &options);

    }
    
    return (fd);
}
  
//-----------------------------------------------------------------
int serial_write(char* s, int _n)
{
    int n = write(fd, s, _n);
    if (n < 0)
        fputs("writefailed!\n", stderr);
    if (n==0) puts("nodata");
    return n;
}
//-----------------------------------------------------------------
void serial_flush()
{
    
    int res = 1;
    //--заполняем FIFO-BUFFER пока данные доступны
    do{
        sleep(0);
        res = read(fd,buf,1);
        if(res<0)break;
        FIFO_write(buf, res, serial_FIFO);
    }while(res);

    

}


//-----------------------------------------------------------------
//Предварительно нужно обновить данные от порта через serial_flush();
int serial_read(void* data_buffer, uint8_t __n)
{
    serial_flush();
    return FIFO_read(data_buffer, __n, serial_FIFO);

}
//-----------------------------------------------------------------
void serial_close()
{
    close(fd);
    del_FIFO(serial_FIFO);
}
//------------------------------------------------------------------