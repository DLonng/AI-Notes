from node import Node


class LinkedList(object):
    def __init__(self):
        self.head_ = None

    def set_head(self, head_none):
        self.head_ = head_none

    def __len__(self):
        count = 0
        current = self.head_
        while current:
            count += 1
            current = current.get_next()
        return count

    def __str__(self):
        current = self.head_
        output = ""
        while current:
            output += str(current) + "->"
            current = current.get_next()
        return output
    # Pop a new item from head
    def pop(self):
        if self.head_:
            self.head_ = self.head_.get_next()
        else:
            raise IndexError("Unable to pop from empty list")

    # Return true if list hava the give value
    def contains(self, value):
        found = False
        current = self.head_
        while current and not found:
            if current.get_data() == value:
                found = True
            else:
                current = current.get_next()
        return found        

    # Delete first instance of give value in the list
    def delete(self, value):
        current = self.head_
        prev = None

        while current:
            if current.get_data() == value:
                if prev:
                    prev.set_next(current.get_next())
                else:
                    self.head_ = current.get_next()
                break
            else:
                prev = current;
                current = current.get_next()

        
    # Push an item on the front of the list.    
    def push(self, value):
        node = Node(value)
        node.set_next(self.head_)
        self.set_head(node)
    
    # Append an item on the end of the list.
    def append(self, value):
        node = Node(value)
        current = self.head_
        if not current:
            self.head_ = node
            return

        while current.get_next():
            current = current.get_next()

        current.set_next(node)

