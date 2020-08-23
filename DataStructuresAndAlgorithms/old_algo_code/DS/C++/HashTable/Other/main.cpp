#include <iostream>
#include <string>
#include "Hash.h"
#include "HashNode.h"
#include "Hash.cpp"
using namespace std;
int main()
{
	Hash<int> my_hash;
	int a[10] = { 10, 11, 22, 33, 44, 55, 56, 67, 78, 99 };
	my_hash.init(a, 10);
	cout << my_hash.isIn(43, 43) << endl;//是否存在43

	HashNode<int>* p = my_hash.find(8);//查找索引为8
	cout << p->key << endl;//打印索引
	cout << p->t << endl;//打印索引的值


	system("pause");
	return 0;
}