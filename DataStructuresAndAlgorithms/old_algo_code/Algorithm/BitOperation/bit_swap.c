#include <stdio.h>
#include <stdlib.h>


static void swap_xor(int *a, int *b) {
  (*a) ^= (*b);
  (*b) ^= (*a);
  (*a) ^= (*b);
}


int main(void)
{
  int a = 1;
  int b = 2;
  printf("a = %d, b = %d\n", a, b);
  swap_xor(&a, &b);
  printf("a = %d, b = %d\n", a, b);
  return 0;
}
