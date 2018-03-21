/*
 * queue.c
 *
 * Created: 3/20/2018 6:54:19 PM
 *  Author: Bill
 */ 

struct Queue {
	Item *front;
	Item *back;
	int size;	
};

struct Item {
	uint8_t code;
	Item *next;
	
};

void enqueue (Queue queue, uint8_t code)
{
	struct Item *item, *prev_tail;
	item->code = code;
	item->next = NULL;
	
	prev_tail = queue->back;
	queue->back = item;
	
}

uint8_t dequeue (Queue* queue)
{
	uint8_t code = queue->front->code;
	queue->front = queue->front->next;
	return code;
	
}