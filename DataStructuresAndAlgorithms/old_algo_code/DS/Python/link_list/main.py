#!/usr/bin/env python3

from link_list import LinkedList


def main():
    link = LinkedList()
    print("Init size: ", len(link))
    
    link.push(1)
    print("Init size: ", len(link))
    print("List content: ", link)
    
    print("Push...");
    link.push(2)
    link.push(3)
    link.push(4)
    print("List content: ", link)
    
    print("poping...")
    link.pop()
    print("List content: ", link)

    print("Have contain 24?")
    if link.contains(2):
        print("Yes")
    else:
        print("No")

    print("Deleteing 2")
    link.delete(2)

    print("List content: ", link)

    print("Append...")
    link.append(11)

    print("List content: ", link)

if __name__ == "__main__":
    main()






