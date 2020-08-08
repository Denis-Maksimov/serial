//#include <stdlib.h>
#include "Serial.h"
#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <stdlib.h>

int main(){

    char buf[150];
    
    struct Serial* COM_port=serial_begin(SERIAL_DEVICE, 9600, mode_8N1);
    int count;
    puts("start");
    int limit=20;
    while (limit>0)
    {
        count = serial_read(COM_port, buf, 150);
        if(count)
        {
            printf(buf);
            fwrite(buf,1,count,stdout);
            printf("\n");
            limit--;
            serial_write(COM_port,"hello",sizeof("hello"));
        }
        
    }
    serial_close(COM_port);

}


