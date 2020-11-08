#include "Serial.h"

#define c_new(t)     ((t*)malloc(sizeof(t)))
#define c_new_n(t,n)     ((t*)malloc(sizeof(t)*n))


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
void serial_set_speed(Serial* serial,int speed)
{
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
void serial_begin(Serial* hSerial, const char* DEVISE,int speed, enum serial_mode mode)
{


    // serial_FIFO = new_FIFO(100);

    // hSerial->rx_buffer=init_queue(256);
 
    hSerial->fd = open( DEVISE, O_RDWR |    // read &FIFO_mgr.FIFOs[FIFO_mgr.n_FIFOs-1 write
                            O_NOCTTY | //эта программа не хочет быть управляющим терминалом для этого порта.
                            O_NDELAY); //эта программа не заботится о состоянии сигнала DCD, т.е. что другой конец линии запущен
    if (hSerial->fd == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
        printf("%i", hSerial->fd);
        fflush(stdout);
        return;

    } else {

        

        // fcntl(hSerial->fd, F_SETFL, O_NONBLOCK);

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
    serial_set_period_to(hSerial, 1, 0);
    serial_set_default_handlers(hSerial);
    return;
}
  
//-----------------------------------------------------------------
int serial_write(Serial* serial,char* s, int _n)
{
    int n = write(serial->fd, s, _n);
    if (n < 0)
        fputs("write_failed!\n", stderr);
    if (n==0) puts("no_w_data");
    return n;
}
//-----------------------------------------------------------------
ssize_t serial_read(Serial* serial,void* data_buffer, u8 __n)
{
     
    ssize_t res = read(serial->fd, data_buffer, 1024);
    if(res<0)
    {   
        fputs("error! cannot read!",stderr);
        return 0;
    }

    return res;

    

}

//------------------------------------------------------------------
int serial_work(Serial* port)
{

    fd_set fd_in,fd_out;
    struct timeval tv;

    // u_socket_t largest_sock=port->io.in;
    while(1){  

        //занулить
        FD_ZERO( &fd_in );
        FD_ZERO( &fd_out );


        FD_SET( port->fd, &fd_in );//добавляем в сет
        FD_SET( port->fd, &fd_out );//добавляем в сет

        tv.tv_sec = port->tv.tv_sec;
        tv.tv_usec = port->tv.tv_usec;
  
        int largest_sock=port->fd;
        
        int ret = select( largest_sock + 1, &fd_in, &fd_out, NULL, &tv );
        // проверяем успешность вызова
        if ( ret == -1 )
        {
            printf("SELECT_ERROR!!\n");
            return -1;  

        }else if( ret == 0 )
        {
            // Событий за таймаут не произошло
            if(port->timeout_handler(port)) return 0;

        }else{
            // обнаружили событие

            if ( FD_ISSET( port->fd, &fd_in ) && port->fd==0 ){
                // Событие входа
                printf("new data\n");
                port->in_handler(port);
            }
          
            if ( FD_ISSET( port->fd, &fd_out )&& port->fd){
                // Событие выхода
                printf("end writing\n");
                port->out_handler(port);
            }            


        }   

    }
}

//------------------------------------------------------------------
void serial_set_in_handler(Serial* port, serial_handler_t in_handler)
{
    port->in_handler=in_handler;
}
void serial_set_out_handler(Serial* port, serial_handler_t out_handler){
    port->out_handler=out_handler;
}
void serial_set_timeout_handler(Serial* port,serial_to_handler_t timeout_handler){
    port->timeout_handler=timeout_handler;
}
//------------------------------------------------------------------

//обработчик событий ввода
void default_in_handler(Serial* serial)
{
    ssize_t res = read(serial->fd, serial->rx_buffer, 256);
    if(res<0)
    {   
        fputs("error! cannot read!",stderr);
        return ;
    }
}      
//обработчик событий вывода
void default_out_handler(Serial* serial)
{
    puts("send ok");
}
//обработчик таймаута раунда (if not 0 завершает сервер)     
int default_timeout_handler(Serial* serial)
{
    static int wath_duck=300;
    wath_duck--;
    printf("kra-kra!!! = *%d*seconds\n",wath_duck);
    return !wath_duck;
}  


void serial_set_default_handlers(Serial* serial)
{
    serial->in_handler=default_in_handler;
    serial->out_handler=default_out_handler;
}

void serial_set_period_to(Serial* serial,__time_t seconds, __suseconds_t useconds)
{
    serial->tv.tv_sec=seconds;
    serial->tv.tv_usec=useconds;
}

//-----------------------------------------------------------------
void serial_close(Serial* serial)
{
    close(serial->fd);

}
//------------------------------------------------------------------

#undef c_new   
#undef c_new_n

