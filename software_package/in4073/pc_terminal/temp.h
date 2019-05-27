#include<string.h>
#define QUEUE_SIZE 256
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint16_t first,last;
  	uint16_t count; 
} queue;
void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);
void flushQueue(queue *q);