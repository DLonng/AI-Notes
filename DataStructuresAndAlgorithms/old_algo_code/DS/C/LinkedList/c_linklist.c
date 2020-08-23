#include "c_linklist.h"

int size(node_t *head) {
  int size = 0;
  node_t *current = head;
  
  while (current) {
    size++;
	current = current->next;
  }

  return size;
}


int empty(node_t *head) {
  return head == NULL;
}


int value_at(node_t *head, int index) {
  if ((index + 1) > size(head))
	  return -1;
  
  node_t *current = head;
  
  for (int i = 0; i < index; i++)
	  current = current->next;
  
  return current->data;
}


void push_back(node_t **head, int item) {
  node_t *new_node = (node_t *)malloc(sizeof(node_t));
  new_node->data = item;
  new_node->next = NULL;

  if (*head == NULL) {
    *head = new_node;
	return;
  }

  node_t *current = *head;

  while (current->next)
	  current = current->next;
  
  current->next = new_node;
}


void print_list(node_t *head) {
  int list_size = size(head);
  node_t *current = head;
  for (int i = 0; i < list_size; i++) {
    printf("%d -> ", current->data);
	current = current->next;
  }
}

void push_front(node_t **head, int item) {
  node_t *new_node = (node_t *)malloc(sizeof(node_t));
  new_node->data = item;
  new_node->next = *head;
  *head = new_node;
}

int pop_front(node_t **head) {
  if (NULL == *head) {
    printf("Unable pop empty list.\n");
	return -1;
  }

  node_t *front_node = *head;
  *head = (*head)->next;
  int pop_value = front_node->data;
  free(front_node);
  return pop_value;
}


int pop_back(node_t **head) {
  if (NULL == *head) {
    printf("Unable pop empty list.\n");
	return -1;
  }

  node_t *current = *head;
  node_t *prev = NULL;

  while (current->next) {
    prev = current;
    current = current->next;
  }
  
  int pop_value = current->data;
  
  if (prev) 
    prev->next = NULL;
  else
    *head = NULL;

  free(current);
  current = NULL;

  return pop_value;
}



int front(node_t *head) {
  return head->data;
}



int back(node_t *head) {
 node_t *current = head;
 
 while (current->next)
   current = current->next;
 
 return current->data;
}


void insert(node_t **head, int index, int item) {
#if 0	
  if ((index < 0) || (index> (size(*head) + 1))) {
    printf("Insert index error.\n");
	return;
  }
  
  if (index == 0) {
    push_front(head, item);
  } else if (index == size(*head)) {
    push_back(head, item);
  } else {
    node_t *current = *head;
    for (int i = 0; i < index - 1; i++)
	    current = current->next;

    node_t *new_node = (node_t *)malloc(sizeof(node_t));
    new_node->data = item;

    new_node->next = current->next;  
    current->next = new_node;
}
#endif
  if (NULL == (*head)) {
    printf("Unable to erase empty list.\n");
	return;
  }
  
  node_t *prev = NULL;
  node_t *current = *head;
  int i = 0;
  
  for (i = 0; (i < index) && current; i++) {
    prev = current;
	current = current->next;
  }

  if (i != index) {
    printf("Index out of bounds.\n");
	return;
  }
  
  node_t *new_node = (node_t *)malloc(sizeof(node_t));
  new_node->data = item;

  if (prev) {
	new_node->next = prev->next;
    prev->next = new_node;
  } else {
	new_node->next = *head;
    *head = new_node;
  }
}


void erase(node_t **head, int index) {
  if (NULL == (*head)) {
    printf("Unable to erase empty list.\n");
	return;
  }
  
  node_t *prev = NULL;
  node_t *current = *head;
  int i = 0;
  
  for (i = 0; (i < index) && current; i++) {
    prev = current;
	current = current->next;
  }

  if (i != index) {
    printf("Index out of bounds.\n");
	return;
  }
  
  if (prev) 
    prev->next = current->next;
  else
    *head = current->next;
  
  free(current);
  current = NULL;
}



int value_n_from_end(node_t *head, int n) {
  if ((n < 1) || (head == NULL)) {
    printf("Unable to handle empty list and n must be > 1.\n");
	return -1;
  }
  
  node_t *current = head;
  node_t *match = head;

  int i = 0;
  for (i = 0; (i < n) && current; i++)
	  current = current->next;

  if (i != n) {
    printf("The list is too short.\n");
	return -1;
  }

  while (current) {
    current = current->next;
	match = match->next;
  }

  return match->data;
}




void reverse(node_t **head) {
  node_t *prev = NULL;
  node_t *current = *head;
  node_t *next = *head;
  
  while (current) {
    next = current->next;
	current->next = prev;
	prev = current;
	current = next;
  }

  *head = prev;
}





void remove_value(node_t **head, int value) {
  if ((*head) == NULL) {
    printf("Unable to remove value from empty list.\n");
	return;
  }

  node_t *current = *head;
  node_t *prev = NULL;
  node_t *del_node = NULL;
  
  while (current) {
    if (value == current->data) {
	  if (prev) 
	    prev->next = current->next;
	  else 
        *head = current->next;

	  del_node = current;
	  free(del_node);

	  current = prev;
	  break;
    }
 
    prev = current;
    current = current->next;
  }
}




/**
 * @synopsis  reverse_list 
 *
 * @param head
 *
 * @return   
 *
 * 使用 3 个指针来逆转一个链表，一定注意检查边界条件
 */
node_t *reverse_list(node_t *head) {
	// 1. 判断链表是否为空
	if (NULL == head || NULL == head->next)
		return NULL;

	// 2. 设置移动指针
	node_t *p1 = head;
	node_t *p2 = p1->next;
	node_t *p3 = p2->next;
	p1->next = NULL;

	// 3. 循环将前面节点放到另一个链表的尾部
	while (p3 != NULL) {
		p2->next = p1;
		p1 = p2;
		p2 = p3;
		p3 = p3->next;
	}

	// 4. 处理最后一个节点
	p2->next = p1;

	// 4. 改变头节点位置并返回
	head = p2;
	return head;
}



















