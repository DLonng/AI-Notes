#include "Hash.h"

template <typename T>
Hash<T>::Hash()
{
	this->n = 10;
	this->p = new HashNode<T>[this->n];
}

template <typename T>
Hash<T>::~Hash()
{
	delete[]p;
}

template <typename T>
int Hash<T>::myHash(int key)
{
	return key % this->n;
}


template <typename T>
void Hash<T>::init(T* pt, int nt)
{
	for (int i = 0; i < this->n; i++)
	{
		this->p[i].key = i;
		this->p[i].cn = 0;
		this->p[i].t = pt[i];
	} 
}

template <typename T>
bool Hash<T>::isIn(int key, T t)
{
	int index = myHash(key);
	if (t == this->p[index].t)
	{
		return true;
	}
	else
	{
		return false;
	}

}

template <typename T>
HashNode<T>* Hash<T>::find(int key)
{
	int index = myHash(key);
	return this->p + index;
}










