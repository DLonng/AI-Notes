#include <stdio.h>
#include <stdlib.h>

#define MAX_LENGTH_INSERT_SORT 7

void myswap(int* L, int i, int j) {
	int temp = L[i];
	L[i] = L[j];
	L[j] = temp; 

	// 异或交换，防止溢出，两个相等的数不能用异或交换，因为相等的数异或为 0
	// 	L[i] = L[i] ^ L[j];
	// 	L[j] = L[i] ^ L[j];
	// 	L[i] = L[i] ^ L[j];
}

/**
 * @synopsis  ThreeToOne 3 数取中
 *
 * @param L
 * @param low : L[low] 是 3 个数的中间值
 * @param high
 */
void ThreeToOne(int* L, int low, int high) {
	int middle = low + (high - low) / 2;
	
	if (L[low] > L[high])
		myswap(L, low, high);
	
	if (L[middle] > L[high])
		myswap(L, high, middle);
	
	if (L[middle] > L[low])
		myswap(L, middle, low);
}


/**
 * @synopsis  Partition 划分子序列
 *
 * @param L
 * @param low
 * @param high
 *
 * @return   
 *
 * 优化：
 * 		3 数取中
 * 		替换代替交换
 */
int Partition(int* L, int low, int high) { 
	// 3 数取中间的数 - 优化！
	ThreeToOne(L, low, high);
	
	// 初始化枢轴为第一个元素
	int pivotkey = L[low];
	
	// 将枢轴临时存储，因为后面会进行替换
	int temp = pivotkey;

	while (low < high) {
		// 比 pivotkey 大的数放在右边
		while (low < high && L[high] >= pivotkey)
			high--;
		
		// myswap(L, low, high); 用替换代替交换，提高效率 - 优化！
		L[low] = L[high];

		// 比 pivotkey 小的数放在左边
		while (low < high && L[low] <= pivotkey)
			low++;
		
		L[high] = L[low];
	}

	L[low] = temp;

	// 返回枢轴所在的位置
	return low;
}

/**
 * @synopsis  QSort 快速排序
 *
 * @param L
 * @param low
 * @param high
 *
 * 0. 基本思想
 * 		通过一趟排序将要排序的数据分割成独立的两部分，其中一部分的所有数据都比另外一部分的所有数据都要小，
 * 		然后再按此方法对这两部分数据分别进行快速排序，整个排序过程可以递归进行，以此达到整个数据变成有序序列
 * 
 * 1. 时间复杂度
 * 		最优：O(nlogn)
 * 		最坏：O(n^2)
 * 		平均：O(nlogn)
 *
 * 2. 空间复杂度
 * 		递归调用产生栈空间：O(logn)
 *
 * 3. 稳定性
 * 		不稳定
 * 
 * 4. 复杂性
 * 		复杂
 *
 * 5. 适用场景
 * 		当数据基本有序，待排序的关键字是随机分布，或者数据很大时可以采用
 *
 * 其他优化方式：加上 else 在排序数据较少时使用其他排序算法 
 */
void QSort(int* L, int low, int high) {
	int pivot;
	if (low < high) {
		pivot = Partition(L, low, high);
		QSort(L, low, pivot - 1); // 枢轴左序列排序
		QSort(L, pivot + 1, high);// 枢轴右序列排序
	} 
}


/**
 * @synopsis  QSort_recursive 尾递归快速排序
 *
 * @param L
 * @param low
 * @param high
 *
 * 优化：
 * 		尾递归
 */
void QSort_rec(int* L, int low, int high) {
	int pivot;
	if (low < high) {
		while (low < high) {
			pivot = Partition(L, low, high);
			
			// 枢轴左序列排序  下一次的递归 high = pivot - 1
			QSort_rec(L, low, pivot - 1);

			low = pivot + 1;
		}  
	} 
}


int main(void) {
	int x[10] = { 50, 10, 90, 30, 70, 40, 80, 60, 20, 100 };
	QSort(x, 0, 9);
	//QSort_rec(x, 0, 9);
	
	for (int i = 0; i < 10; i++)
		printf("%d ", x[i]);
	
	printf("\n");
	return 0;
}
