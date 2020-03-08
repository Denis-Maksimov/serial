//#include <stdlib.h>
#include "Serial.h"

int main(){

    char* buf=(char*)malloc(150);
    char* old=buf;
    
    int port=serial_begin(9600, mode_8N1);
    int limit=123;
    int count=0;
    puts("start");
    
    while (limit>0)
    {
    //    serial_flush();
        count = serial_read( buf, 0);
        serial_read( buf, count);
        if(count){
            buf=buf+count;
            limit=limit-count;
        }
        
    }
    printf("\n%s",old);
    serial_close();

    free(old);

}


