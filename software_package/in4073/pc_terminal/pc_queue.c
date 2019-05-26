/*------------------------------------------------------------------
 *  queue.c -- some queue implementation stolen from the interwebs
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "string.h"

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

void init_queue(queue *q){
	
	q->first = 0;
	q->last = QUEUE_SIZE - 1;
	q->count = 0;
}

void enqueue(queue *q,char x){

//	fprintf(stderr, "enqueue: %d\n", x);
	q->last = (q->last + 1) % QUEUE_SIZE;
	q->Data[ q->last ] = x;
	q->count += 1;

}

char dequeue(queue *q){

	char x = q->Data[ q->first ];
	q->first = (q->first + 1) % QUEUE_SIZE;
	if(q->count < 1) {
		//printf("%s\n", "counter is going below 0");
		flushQueue(q);
	} else {
		q->count -= 1;
	}



	return x;
}

void flushQueue(queue *q){
	memset(q->Data, 0, QUEUE_SIZE);
	init_queue(q);
}
