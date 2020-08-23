#pragma once
#include "HashNode.h"
template <typename T>
class Hash
{
public:
	HashNode<T>* p;
	int n;


	int myHash(int key);
	void init(T* pt, int nt);
	bool isIn(int key, T t);
	HashNode<T>* find(int key);

	Hash();
	~Hash();
};

