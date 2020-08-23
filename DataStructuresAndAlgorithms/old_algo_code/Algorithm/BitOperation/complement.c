#include <stdio.h>
#include <stdlib.h>
#include <limits.h>


int main1(void)
{
	// Parse error, print junk data.
	printf("%d\n", 10.011);
	// printf parse data with %d or %f
	printf("%f\n", 10);
	return 0;
}


int main2()
{
	int num = 100;

	// %p: print var address in memory.
	printf("%p\n", &num);

	return 0;
}

int main3()
{
	char ch = 1;//赋值数值
	char ch1 = '1';//赋值字符
	printf("%d %d\n", ch, ch1);//1 49
	return 0;
}

int main6()
{
	//[0 - (2^16 - 1)]
	unsigned short num = 65535;
	printf("%d\n", num);
	
	// overflow -> 0
	num += 1;
	
	printf("%d\n", num);

	return 0;
}

int main7()
{
	int num = -1;
	// -1, 4294967295
	printf("%d %u\n", num,num);
	return 0;
}

int main8()
{
	int x = 1;
	int y = -1;
	// Debug: View binary data in memory address.
	printf("x = %p,y = %p\n", &x, &y);
	return 0;
}	

int main9()
{
	// 0 正数 1 负数
	// 有符号位 只有 31 个数据的二进制位
	// 0111 1111 1111 1111 1111 1111 1111 1111 
	// 1111 1111 1111 1111 1111 1111 1111 1111  
	// 无符号 正数 没有符号位 全部都是数据

	//2147483647 -2147483648
	printf("%d %d\n", INT_MAX, INT_MIN);	
	
	//会溢出  %d 只能显示 INT_MIN -> INT_MAX
	// -2147483648 -2147483647
	printf("%d %d\n", INT_MAX + 1, INT_MIN + 1);
		
	// 4294967295, 0
	printf("%u %u\n", UINT_MAX, 0);
	
	// 0, 4294967295
	printf("%u %u\n", UINT_MAX + 1, 0 - 1);

	return 0;
}

int main10()
{
	int x = 4294967295;
	int y = -1;
	//1111 1111 1111 1111 1111 1111 1111 1111 
	//-1 和 4294967295 在内存中都是按照32个1来存储
	//printf解析的方式不同 输出的数值不同

	// -1, 4294967295
	printf("%d %u\n", x, x);
	// -1, 4294967295
	printf("%d %u\n", y, y);
	//%u 按照无符号解析全部都是数据位 没有符号位 就是 4294967295

	//%d 按照有符号位解析 有符号位 由补码求反码再求原码 -1
	//无符号位就全部解析为数据位

	return 0;
}


int main()
{
	// 面试经常问

	// 4294967295
	unsigned int num = -1;
	// -1, 4291967295
	printf("%d %u\n", num, num);
	//printf输出数据与数据的类型无关
	//-1在内存中是32个1存储
	//所以%d就按有符号位来解析
	//数据在计算机内存里面以补码的形式存储
	//有符号位就由补码->反码->原码
	
	//而%u按照无符号位来解析 32个1全部都是数据位
	//计算出来就是4294967295

	return 0;
}
