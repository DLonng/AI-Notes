#ifndef NODE_H_
#define NODE_H_

//namespace nodespace {

template <typename T>
class Node {
  public:
    Node(const T& data) : data_(data), next_(nullptr) {}
	~Node() {}
    const T& data() const { return data_; }
	void set_data(const T& data) { data_ = data; }
	Node<T> *next() const { return next_; }
	void set_next(Node<T> *next) { next_ = next; }
  private:
    T data_;
	Node<T> *next_;
};

//}


#endif //NODE_H_
