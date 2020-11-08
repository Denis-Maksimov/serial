//#include <stdlib.h>
#include "Serial.h"







int main(){

    Serial COM_port;
    serial_begin(&COM_port, SERIAL_DEVICE, 9600, mode_8N1);
    puts("start");

    serial_work(&COM_port);

    serial_close(&COM_port);

}


