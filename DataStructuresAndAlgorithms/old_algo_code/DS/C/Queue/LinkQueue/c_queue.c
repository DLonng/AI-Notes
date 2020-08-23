#include "c_queue.h"


CQueue *cqueue_new() {
  CQueue *queue = (CQueue *)malloc(sizeof(CQueue));
  check_address(queue);

  queue->head = NULL;
  queue->tail = NULL;
}

void check_address(void *address) {
  if (NULL == address) {
    printf("Malloc failed.\n");
	exit(EXIT_FAILURE);
  }
}

void print_queue(CQueue *queue) {
  node_t *current = queue->head;
  while (current) {
    printf("%d -> ", current->data);
	current = current->next;
  }

  putchar('\n');
}



void enqueue(CQueue *queue, int item) {
  if (NULL == queue) {
    printf("Unable to add item to invalid queue.\n");
	exit(EXIT_FAILURE);
  }

  
  node_t *new_node = (node_t *)malloc(sizeof(node_t));
  check_address(new_node);
  new_node->data = item;
  new_node->next = NULL;
  
  if (queue->tail != NULL) {
    queue->tail->next = new_node;
  }

  queue->tail = new_node;

  if (NULL == queue->head) 
    queue->head = new_node;
}


int dequeue(CQueue *queue) {
  if (NULL == queue) {
    printf("Unable to dequeue from invalid queue.\n");
	exit(EXIT_FAILURE);
  }

  node_t *head = queue->head;
  
  int value = head->data;
  
  if (queue->head == queue->tail)
    queue->head = NULL;

  queue->head = head->next;
  
  free(head);
  head = NULL;

  return value;
}



int empty(CQueue *queue) {
  return queue->head == NULL;
}

