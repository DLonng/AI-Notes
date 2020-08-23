#include <stdio.h>
#include <stdlib.h>


static int bit_abs(int x) {
  const int bit31 = x >> 31;
  return (x ^ bit31) - bit31;
}


int main(void)
{
  printf("%d\n", bit_abs(-1));
  return 0;
}
