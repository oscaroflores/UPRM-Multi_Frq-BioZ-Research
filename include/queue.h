#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_SIZE
#define MAX_SIZE 100
#endif

typedef struct {
    int items[MAX_SIZE];
    int front;
    int rear;
} Queue;

void initializeQueue(Queue* q);
bool isEmpty(Queue* q);
bool isFull(Queue* q);
void enqueue(Queue* q, int value);
void dequeue(Queue* q);
int  peek(Queue* q);
void printQueue(Queue* q);

#ifdef __cplusplus
}
#endif

#endif // QUEUE_H