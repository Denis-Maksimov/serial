#include "queue.h"
#include <stdio.h>
#include <stdlib.h>


#define c_new(t)     ((t*)malloc(sizeof(t)))
#define c_new_n(t,n)     ((t*)malloc(sizeof(t)*n))


struct queue* init_queue(size_t n)
{
    struct queue* rv = c_new(struct queue);
    rv->buf=c_new_n(u8,n);
    rv->size_queue=n;
    return rv;
}

void free_queue(struct queue* q)
{
    free(q->buf);
    free(q);
}
//---------------------------------
int isempty_queue(struct queue *q) 
{
  if(!q->count)
    return 1;
  return 0;
}

int isfull_queue(struct queue *q) 
{
  if(q->count >= q->size_queue)
    return 1;
  return 0;
}

int insert_queue(struct queue *q, u8 x) 
{
  if(!isfull_queue(q)) 
  {
    ((u8*)q->buf)[q->count]=x;
    q->count++;
    return 0;
  }
  return -1;
}

u8 pop_queue(struct queue *q) 
{
  int retval;

  if(isempty_queue(q)) {
    return 0;
  }

  retval = ((u8*)q->buf)[0];

  //сдвинуть по байтикам
  for(int i = 0; i < q->count; i++) {
    ((u8*)q->buf)[i] = ((u8*)q->buf)[i+1];
  }

  q->count--;
  return retval;
}
//---------------------------------
//---------------------------------
// void print_queue(struct queue *q) {
    
//   if(isempty_queue(q)) 
//   {
//     printf("Очередь пуста!\n");
//     return;
//   }
//   for(int i=0; i < q->count; i++)
//     printf("%d ",((u8*)q->buf)[i]);
//   return;
// }

//---------------------------------
// #define QMAX 100


// int main(int argc, char const *argv[])
// {
//   struct queue* q= init(QMAX);
//   int a;
  
//   print_queue(q);
//   int i=0;

//   //--полностью заполняем от нуля до конца
//   while(!isfull(q)) {
//     insert(q, i);
//     i++;
//   }
//   printf("\n");
//   print_queue(q);

//   //--читаем с очереди пока там есть значения
//   while(!isempty_queue(q)) 
//   {
//     a = pop_queue(q);
//     printf("\nУдален элемент %d\n", a);
//     print_queue(q);
//   }

//   //--утечку памяти никто не отменял - попользовался прибери за собой
//   free_queue(q);

//   return 0;
// }

#undef c_new   
#undef c_new_n
