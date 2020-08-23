#include <stdio.h>
#include <stdlib.h>

/**
 * @synopsis  binary_search_1 
 *
 * @param array
 * @param length
 * @param key
 *
 * @return   
 * Note : Ordinary method.
 */
int *binary_search_1(int *array, int length, int key) {
  int low = 0;
  int mid = 0;
  int high = length - 1;

  while (low <= high) {
    //mid = (high + low) / 2;
    mid = low + (high - low) * ((double)(key - array[low]) / (array[high] - array[low]));
	//printf("mid = %d\n", mid);

	if (key < array[mid])
		high = mid - 1;
	else if (key > array[mid])
		low = mid + 1;
	else 
		return array + mid;
  }

  return NULL;
}



/**
 * @synopsis  binary_search_2 
 *
 * @param array
 * @param length
 * @param key
 *
 * @return   
 * Note: Optimization，use ponit and >> to imporve efficiency.
 */
int *binary_search_2(int *array, int length, int key) {
  int *low = array;
  int *mid = NULL;
  int *high = array + length - 1;

  while (low <= high) {
    //mid = low + ((high - low) >> 1);
    mid = low + (int)((high - low) * ((double)(key - (*low)) / ((*high) - (*low))));
	//printf("*mid = %d\n", *mid);
	
	if (key < *mid)
		high = mid - 1;
	else if (key > *mid)
		low = mid + 1;
	else 
		return mid;
  }

  return NULL;
}




int main(int argc, char **argv)
{
  int a[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  int key = atoi(argv[1]);

  int *find1 = binary_search_1(a, 10, key);
  int *find2 = binary_search_2(a, 10, key);
  
  printf("%d\n", *find1);
  printf("%d\n", *find2);
  return 0;
}
