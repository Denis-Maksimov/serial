#ifndef QUEUE_H
#define QUEUE_H
#include <stddef.h>    

#define QUEUE_API extern
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef signed char s8;
typedef signed short s16;
typedef signed int s32;

struct queue 
{
  size_t size_queue; //max размер очереди
  u8* buf;          //элементы
  size_t count; //текущее число элементов
};

QUEUE_API struct queue* init_queue(size_t n);
QUEUE_API void free_queue(struct queue* q);
QUEUE_API int insert_queue(struct queue *q, u8 x);
QUEUE_API u8 pop_queue(struct queue *q);
QUEUE_API int isfull_queue(struct queue *q);
QUEUE_API int isempty_queue(struct queue *q) ;

#endif // !QUEUE_H