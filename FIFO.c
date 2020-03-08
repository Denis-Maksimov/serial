#include "FIFO.h"

//-- Обёртки stdlib ---
#if defined(__MY_BARE_ARM__)


void* malloc(int size) {
    return u_malloc(size);
}

void free(void* ptr){
    u_free(ptr);
}

#endif

//------------------------------
typedef struct __FIFOdata{
    uint8_t* data;
    uint8_t readPTR;
    uint8_t writePTR;
    uint8_t size;
    uint32_t ID;
} FIFOdata;

//-- Local structure mgr ---
struct {

	uint8_t n_FIFOs;
	FIFOdata** FIFOs; // { & FIFO1, & FIFO2 , .... , & FIFOn }
	uint32_t IDs;	  // !!! ГОСТ Р 57764-2017 !!!
} FIFO_mgr;

//-------------------------------------------------------------------
/****************************************************
 * Я не знаю почему, но это работает, хотя 
 * -1 в последних 2х строчках подозрительно выглядит 
 *  *************************************************/
uint32_t new_FIFO(uint8_t size){
	/****** FIFO obj ********/
	 //-- memory for new obj --- 
	 FIFOdata* fifo =(FIFOdata*)malloc(sizeof(FIFOdata));
	 
	 fifo->data = (uint8_t*)malloc(size);
	 fifo->size=size;
	 fifo->readPTR=0;
	 fifo->writePTR=0;
	 fifo->ID=FIFO_mgr.IDs;
	 FIFO_mgr.IDs++;
	 
	/****** FIFO mgr *******/
	 //-- append obj --
	 
	 FIFO_mgr.n_FIFOs++;
	 //--- запомнили старый массив
	 FIFOdata** tmp = FIFO_mgr.FIFOs;
	 //--- выделили память под новый массив
	 FIFO_mgr.FIFOs = (FIFOdata**)malloc( (FIFO_mgr.n_FIFOs) * (sizeof(FIFOdata*) ) );
	 
	 if(!tmp){
		 FIFO_mgr.FIFOs[FIFO_mgr.n_FIFOs-1]=fifo;

		 return (fifo->ID);
	 }
	 //--- заполняем старые объекты в новый массив
	 // !!! ГОСТ Р 57764-2017 !!!
	 
	 for(int i=0;i<(FIFO_mgr.n_FIFOs-1);i++)
	 {
		 
		 FIFO_mgr.FIFOs[i]=tmp[i];
		 
	 }
	 free(tmp);

	 //--- добавляем новый объект
	FIFO_mgr.FIFOs[FIFO_mgr.n_FIFOs-1]=fifo;
	return fifo->ID;
}
//----------------------------------------------------------------------

int FIFO_get_N_from_ID(uint32_t ID)
{
	int n= -1;
	for(int i=0; i<FIFO_mgr.n_FIFOs; i++)
	{
		if ((FIFO_mgr.FIFOs[i]->ID)==(ID))
		{
			n=i;
			break;
		}
	}
	return n;
}

//----------------------------------------------------------------------

void del_FIFO(uint32_t ID)
{

	//ищем по ID n
	int n = FIFO_get_N_from_ID( ID );

	//-- присваиваем найденный адрес
	FIFOdata* fifo = FIFO_mgr.FIFOs[n];

	free(fifo);
	FIFO_mgr.n_FIFOs--;



}

//----------------------------------------------------------------------


void FIFO_write(void* __data,uint8_t _n,uint32_t ID)
{
	//-- кастомизация
	uint8_t* _data=(uint8_t*)__data;
	
	//ищем по ID n
	int n = FIFO_get_N_from_ID( ID );
	if (n < 0) return;

	//-- присваиваем найденный адрес
	FIFOdata* fifo = FIFO_mgr.FIFOs[n];


	//-- запишем данные побайтно
	for(int i=0;i<_n;i++)
	{
		//-- write byte
		fifo->data[ fifo->writePTR ] = _data[i];
		//-- если дошли до конца буфера то переходим в начало (закольцовываемся)
		if ( fifo->writePTR == fifo->size ) {
			fifo->writePTR = 0;
		}else{
			fifo->writePTR = fifo->writePTR+1;
		}

	}

}

//--------------------------------------------------------------------------

int FIFO_data_available(FIFOdata* fifo)
{
	int retval = fifo->writePTR - fifo->readPTR;
	if(retval<0)
	{
		retval += fifo->size;	
	}
	return retval;
}

//--------------------------------------------------------------------------

int FIFO_read(void* data_buffer, uint8_t __n, uint32_t ID)
{
	//-- кастомизация
	uint8_t* __data=(uint8_t*)data_buffer;
	
	//-- ищем по ID n
	int n = FIFO_get_N_from_ID( ID );
	if (n < 0) return -1;

	//-- присваиваем найденный адрес
	FIFOdata* fifo = FIFO_mgr.FIFOs[n];

	//-- ограничиваем чтение доступной областью
	if(__n > FIFO_data_available(fifo))
	{
		__n = FIFO_data_available(fifo);
	}

	//-- запишем данные в буффер побайтно
	for(int i=0;i<__n;i++)
	{
		//-- write byte
		__data[i] = fifo->data[fifo->readPTR];

		//-- если дошли до конца буфера то переходим в начало (закольцовываемся)
		if ( fifo->readPTR == fifo->size ) {
			fifo->readPTR=0;
		}else{
			fifo->readPTR = fifo->readPTR+1;
		}

	}
	
	//-- возвращаем число доступных для чтения байтов
	return FIFO_data_available(fifo);
}

//--------------------------------------------------------------------------
/************** SAMPLE ******************************

#include <stdio.h>
#include <string.h>

char buffer[50]; 
char str1[]="hello, FIFO\r\n";
char str2[]="good, FIFO\r\n";



int main(int argc,char** argw){

	
	uint32_t FIFO1 = new_FIFO(50);
	uint32_t FIFO2 = new_FIFO(50);

	FIFO_write( str1, 11, FIFO1);
	FIFO_write( str2, sizeof("good, FIFO"), FIFO1);
	
	FIFO_read(buffer, 50, FIFO1);
	puts(buffer);
	

	FIFO_write( str2, sizeof("good, FIFO"), FIFO2);
	int f = FIFO_read(buffer, 5, FIFO2);
	printf("%i\n", f);
	puts(buffer);

	return 0;
}

*/
