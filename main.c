//#include <stdlib.h>
#include "Serial.h"



void recv_handler(Serial* serial)
{
    ssize_t res = read(serial->fd, serial->rx_buffer, 256);
    if(res<0)
    {   
        fputs("error! cannot read!",stderr);
        return ;
    }
    printf("Received data:\n%.*s\n *%ld*\n",(int)res, serial->rx_buffer,res);
}

void send_handler(Serial* serial)
{

}

int periodic_to_handler(Serial* serial){
    return 0;
}


int main(){

    Serial COM_port;
    serial_begin(&COM_port, SERIAL_DEVICE, 9600, mode_8N1);
    puts("start");
    COM_port.in_handler=recv_handler;
    serial_work(&COM_port);

    serial_close(&COM_port);

}


