#include <stdio.h>
#include <stdlib.h>



int main(void)
{
	/*
	 * & Fun: 
	 * 		1. &0: clear some bits.
	 *		2. &1: keep some bits.
	 * 1 & 1 = 1  
	 * 1 & 0 = 0 
	 * 0 & 1 = 0 
	 * 0 & 0 = 0
	 */
	unsigned char ch1 = 15;		// 0000 1111
	unsigned char ch2 = 3;		// 0000 0011
	printf("%d\n", ch1 & ch2);	// 0000 0011 = 3

	/*
	 * | Fun:
	 *		1. |0: keep some bits.
	 * 		2. |1: set some bits.
	 * 1 | 1 = 1 
	 * 1 | 0 = 1 
	 * 0 | 1 = 1 
	 * 0 | 0 = 0
	 */
	unsigned char ch3 = 169;	// 1010 1001
	unsigned char ch4 = 240;	// 1111 0000
	printf("%d\n", ch3 | ch4);	// 1010 1001 = 249

	/*
	 * ~ Fun:
	 * 		1. ~0: 1
	 * 		2. ~1: 0
	 */
	unsigned char ch = 15;	// 0000 1111
	unsigned char uch = ~ch;// 1111 0000
	printf("%d\n", ch);		// ch = 15
	printf("%d\n", uch);	// uch = 240

	unsigned char ch5 = 15;	// 0000 1111
	ch5 = ~ch5;				// 1111 0000
	printf("%d\n", ch5);	// ch5 = 240

	/*
	 * ^ Fun:
	 * 		1. ^1: reversal bits.
	 * 		2. ^0: keep bits.
	 * 		3. swap var: save memory
	 * 1 ^ 1 = 0
	 * 0 ^ 0 = 0
	 * 1 ^ 0 = 1
	 * 0 ^ 1 = 1
	 */
	unsigned char ch6 = 169;	// 1010 1001
	unsigned char ch7 = 15; 	// 0000 1111 
	printf("%d\n", ch6 ^ ch7);	// 1010 0110
	
	/*
	 * Swap:
	 * 		x = x ^ y
	 * 		y = y ^ x
	 * 		x = x ^ y
	 */
	unsigned char ch8 = 20;		//0001 0100
	unsigned char ch9 = 10;		//0000 1010
	printf("%d, %d\n", ch8, ch9);

	//0001 0100
	//0000 1010
	ch8 = ch8 ^ ch9;			//ch5 = 0001 1110

	//0001 1110
	//0000 1010
	ch9 = ch9 ^ ch8;			//ch6 = 0001 0100 = 20
 
	//0001 0100
	//0001 1110
	ch8 = ch8 ^ ch9;			//ch5 = 0000 1010 = 10
	printf("%d, %d\n", ch8, ch9);

	/*
	 * >> << Fun:
	 * 		1. x >> 1: x / 2
	 * 		2. x << 1: x * 2
	 * Note:
	 * 		<<: Fill 0 on the right.
	 * 		>>:
	 * 			1. signed  : Fill 1 on the left.
	 * 			2. unsigned: Fill 0 on the left.
	 */
	unsigned char ch10 = 2;		// 0000 0010
	printf("2 >> 1 = %d, 2 << 1 = %d\n", ch10 >> 1 , ch10 << 1);

	return 0;
}
