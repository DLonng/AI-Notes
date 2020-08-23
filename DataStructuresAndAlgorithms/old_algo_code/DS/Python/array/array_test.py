#!/usr/bin/env python3


def array_test():
    ar = [3, 2, 4, 5]
    ar.pop()
    ar.append(6)
    print(ar)
    print("Index of 2:", ar.index(2))
    print("Index of 3:", ar.index(3))
    print("Index of 4:", ar.index(4))
    
    ar.remove(4)
    print("Remove 4:", ar)

    ar.reverse()
    print("Reverse:", ar)
    
    print("sorted return: ", sorted(ar))

    ar.sort()
    print("sorted in place: ", ar)


def main():
    array_test()

if __name__ == "__main__":
    main()





