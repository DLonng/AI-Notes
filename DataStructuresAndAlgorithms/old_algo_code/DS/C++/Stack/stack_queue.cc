#include <iostream>


/**
 * @synopsis  This stack has bug!!!
 */
class Stack {
public:
	Stack(const int size) : top(0), data(nullptr){
		if (size < 1) {
			std::cout << "size < 1" << std::endl;
		} else {
			data = new int[size];
			if (nullptr == data) {
				std::cout << "new int[size] failed." << std::endl;
				exit(-1);
			}
		}
	}

	~Stack() {
		if (data != nullptr)
			delete []data;
	}

	void Push(const int item) {
		data[top++] = item;
	}

	int Pop(void) {
		return data[--top];
	}

	int Count(void) {
		return top;
	}
private:	
	int *data;
	int top;
};


/**
 * @synopsis  使用 2 个栈实现一个队列
 */
class Queue {
public:
	Queue(const int s1_size, const int s2_size) {
		s1 = new Stack(s1_size);
		s2 = new Stack(s2_size);
	}
	~Queue() {
		if (s1 != nullptr)
			delete s1;// new -> delete, new [] -> delete []
		if (s2 != nullptr)
			delete s2;
	}
	void Enqueue(const int item) {
		s1->Push(item);
	}

	int Dequeue() {
		// 1. s2 不为空，直接弹出栈顶元素
		if (s2->Count() != 0) {
			return s2->Pop();
		} else {
			// 2. s2 为空，如果 s1 也为空，提示错误
			if (0 == s1->Count()) {
				std::cout << "s2->Count() == 0" << std::endl;
				return -99;
			} else { 
				// 3. 如果 s1 不为空，将 s1->Count() - 1 个元素倒入 s2 中
				while (s1->Count() > 1)
					s2->Push(s1->Pop());
				// 4. 直接返回 s1 的栈顶元素
				return s1->Pop();
			}
		}
	}

private:
	Stack *s1;
	Stack *s2;
};


int main(void) {
	Queue q(10, 10);
	q.Enqueue(1);
	q.Enqueue(2);
	q.Enqueue(3);
	q.Enqueue(4);

	std::cout << q.Dequeue() << std::endl;
	std::cout << q.Dequeue() << std::endl;
	std::cout << q.Dequeue() << std::endl;
	std::cout << q.Dequeue() << std::endl;
	return 0;
}


int main1(void) {
	Stack s(10);
	s.Push(1);
	s.Push(2);
	s.Push(3);
	s.Push(4);
	
	std::cout << "s.count = " << s.Count() << std::endl;
	
	std::cout << s.Pop() << std::endl;
	std::cout << s.Pop() << std::endl;
	std::cout << s.Pop() << std::endl;
	std::cout << s.Pop() << std::endl;
	return 0;
}
