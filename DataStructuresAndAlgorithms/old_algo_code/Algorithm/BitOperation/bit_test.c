#include <stdio.h>
#include <stdlib.h>

int main() {
	float f1 = 1.0;
	double db = 2.3;
	// 只有整数和字符型的数据才适用于位运算实数型不能参加位运算 浮点不行
	printf("%f", f1 & 1); 
	printf("%f", db & 1);
	return 0;
}


int main3() {
	// Octal 
	int num = 010;
	printf("%d\n", num);

	int a = 0x12345678;
	printf("%p\n", &a);
	return 0;
}


int main2() {
	int num = 1;

	//0000 0000 0000 0001 内存中按照这种方式
	//1000 0000 0000 0000
	printf("%lu\n", sizeof(num));
	printf("\n%p", &num);
	return 0;	
}


// &  1&1 = 1	1&0 = 0	0&0 = 0 0&1 = 1
int  main1()
{
	int num = 1234567;
	//1234567 取出低8位
	//0000 0000 0001 0010  1101 0110 1000 0111
	//0000 0000 0000 0000  0000 0000 1111 1111  255
	//0000 0000 0000 0000  0000 0000 1000 0111
	printf("%d\n", num & 255);
	return 0;
}
