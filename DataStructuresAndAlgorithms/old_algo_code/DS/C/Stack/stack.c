#include <stdio.h>

#define MAXSIZE 50

typedef struct {
	char data[MAXSIZE];
	int top;
}Stack;

void init_stack(Stack *S);        
int stack_empty(Stack S);         
int push_stack(Stack *S, char e);   
int pop_stack(Stack *S, char *e);   
int get_top(Stack S, char *e); 
 

void init_stack(Stack *S) {
	S->top = 0;
}

int stack_empty(Stack S){
	if (S.top == 0)
		return 1;
	else
		return 0;
}

int push_stack(Stack *S, char e) {
	if (S->top >= MAXSIZE) {
		printf("Stack has full!");
		return 0;
	} else {
		S->data[S->top] = e;
		S->top++;
		return 1;
	}
}

int pop_stack(Stack *S, char *e) {
	if (S->top == 0) {
		printf("Stack is empty!\n");
		return 0;
	} else {
		S->top--;
		*e = S->data[S->top];
		return 1;
	}
}

int get_top(Stack S, char *e) {
	if (S.top <= 0) {
		printf("Stack is empty\n");
		return 0;
	} else {
		*e = S.data[S.top - 1];
		return 1;
	}
}



int main() {

	return 0;
}
