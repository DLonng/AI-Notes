#include <stdio.h> 

/**
 * @synopsis  insert_sort 
 *
 * @param L[]
 * @param length
 *
 * 0. 基本思想
 * 		每次将一个待排序的数据，跟前面已经有序的序列的数字一一比较找到自己合适的位置，插入到序列中，直到全部数据插入完成。
 * 
 * 1. 时间复杂度
 * 		平均 O(n^2) 比冒泡和简单选择性能要好些
 * 		最坏 O(n^2) 逆序，移动 O(n^2) 次，比较 O(n^2) 次
 * 		最好 O(n) 全部有序，只需要比较，不需要移动
 * 
 * 2. 空间复杂度
 * 		只需要一个记录的辅助空间 O(1)
 *	
 * 3. 稳定性
 * 		稳定
 *
 * 4. 复杂性
 * 		简单
 *
 * 5. 适用场合
 * 		适合少量数据的排序，或者数据基本已经有序的情况
 */
static void insert_sort(int L[], int length) {  
	if (NULL == L || length < 1) {
		printf("L = NULL or length < 1\n");
		return;
	}

	// 1. 设置哨兵
	int temp = 0;
	int j = 0;

	for (int i = 1; i < length; i++) {
		if (L[i - 1] > L[i]) {
			// 2. 将要插入的数复制给哨兵
			temp = L[i];
			// 3. 移动其他元素
			for (j = i; (j > 0) && (L[j - 1] > temp); j--)
				L[j] = L[j - 1];
			// 4. 插入哨兵的数据
			L[j] = temp;
		}
	} 
}


int main(void) {
	int x[10] = { 5, 3, 2, 1, 7, 4, 6, 9, 8, 10 };
	
	insert_sort(x, 10);
	// insert_sort(NULL, 10);
	insert_sort(x, 0);

	for (int i = 0; i < 10; i++)
		printf("%d ", x[i]);

	printf("\n");
	return 0;
}





