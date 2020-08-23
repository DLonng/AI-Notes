#include <stdio.h>

/**
 * @synopsis  pop_sort 
 *
 * @param L[]
 * @param length
 * 
 * 0. 基本思想
 *		通过与相邻元素的比较和交换来把小的数交换到最前面 
 *
 * 1. 时间复杂度
 * 		平均 O(n^2)
 * 		最坏 O(n^2)
 * 		最好 O(n)
 * 
 * 2. 空间复杂度
 * 		O(1)
 * 
 * 3. 稳定性
 * 		稳定
 * 
 * 4. 复杂性
 * 		简单
 *
 * 5. 适用场合
 * 		数据量比较小，并且数据已经基本有序
 */
static void pop_sort(int L[], int length) {
	if (NULL == L || length < 1) {
		printf("L == NULL or length < 1 !\n");
		return;
	}

	int temp = 0;
	// 1. 是否需要排序的标记
	int isNeedSort = 1;

	for (int i = 0; (i < length) && isNeedSort; i++) {
		// 2. 假设本次已经有序
		isNeedSort = 0;
		for (int j = length - 1; j > i; j--) {
			if (L[j - 1] > L[j]) { 
				temp = L[j - 1];
				L[j - 1] = L[j];
				L[j] = temp; 
				// 发生交换，仍然需要排序
				isNeedSort = 1;
			}
		}
	}
}




int main(void) {
	int x[10] = { 5, 3, 2, 1, 7, 4, 6, 9, 8, 10 };
	pop_sort(x, 10);
	//pop_sort(NULL, 10);
	//pop_sort(x, -1);
	for (int i = 0; i < 10; i++)
		printf("%d ", x[i]); 

	return 0;
}
