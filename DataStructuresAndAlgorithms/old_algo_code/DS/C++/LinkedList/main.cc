#include "linked_list.h"
#include "linked_list.cc"

#include <iostream>
#include <cassert>
#include <gtest/gtest.h>

#include "node.h"

namespace {

class SizeTest : public ::testing::Test {};

TEST_F(SizeTest, Size) {
  LinkedList<int> link;
  
  EXPECT_EQ(link.Size(), 0) << "Empty link size != 0";
  EXPECT_TRUE(link.Empty());

  link.PushFront(1);
  EXPECT_EQ(link.Size(), 1);
  EXPECT_FALSE(link.Empty());
}


TEST_F(SizeTest, Empty) {
  LinkedList<int> link;
  
  EXPECT_TRUE(link.Empty());
  
  link.PushFront(1);
  
  EXPECT_FALSE(link.Empty());
}



class PushTest : public ::testing::Test {};

TEST_F(PushTest, PushFront) {
  LinkedList<int> link;
  link.PushFront(1); 
  link.PushFront(2); 
  link.PushFront(3);
  EXPECT_EQ(link.Size(), 3);
  EXPECT_EQ(link.PopFront(), 3);
}


TEST_F(PushTest, PushBack) {
  LinkedList<int> link;
  link.PushBack(1); 
  link.PushBack(2); 
  link.PushBack(3); 
  EXPECT_EQ(link.Size(), 3);
  EXPECT_EQ(link.PopBack(), 3);
}

class PopTest : public ::testing::Test {};

TEST_F(PopTest, PopFront) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  EXPECT_EQ(link.PopFront(), "hello1");
  EXPECT_EQ(link.Size(), 3);
}


TEST_F(PopTest, PopBack) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  EXPECT_EQ(link.PopFront(), "hello1");
  EXPECT_EQ(link.Size(), 3);
}

class GetValueTest : public ::testing::Test {};

TEST_F(GetValueTest, Back) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  EXPECT_EQ(link.Back(), "hello4");
}

TEST_F(GetValueTest, Front) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  EXPECT_EQ(link.Front(), "hello1");
}


TEST_F(GetValueTest, ValueAt) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  EXPECT_EQ(link.ValueAt(0), "hello1");
  EXPECT_EQ(link.ValueAt(1), "hello2");
  EXPECT_EQ(link.ValueAt(3), "hello4");
}


TEST_F(GetValueTest, ValueNFromEnd) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  EXPECT_EQ(link.ValueNFromEnd(1), "hello4");
  EXPECT_EQ(link.ValueNFromEnd(3), "hello2");
  EXPECT_EQ(link.ValueNFromEnd(4), "hello1");
}

class InsertEraseTest : public ::testing::Test {};

TEST_F(InsertEraseTest, Insert) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello4");
  link.Insert(0, "hello0");
  link.Insert(1, "hello11");
  link.Insert(3, "hello3");

  EXPECT_EQ(link.ValueAt(0), "hello0");
  EXPECT_EQ(link.ValueAt(1), "hello11");
  EXPECT_EQ(link.ValueAt(3), "hello3");
}

TEST_F(InsertEraseTest, Erase) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");
  link.PushBack("hello5");
  link.PushBack("hello6");

  link.Erase(0);
  link.Erase(1);
  link.Erase(3);

  EXPECT_EQ(link.Back(), "hello5");
}

class RemoveTest : public ::testing::Test {};

TEST_F(RemoveTest, RemoveValue) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");

  link.RemoveValue("hello2");
 
  EXPECT_EQ(link.Size(), 4);

};

class ReverseTest : public ::testing::Test {};

TEST_F(ReverseTest, Reverse) {
  LinkedList<std::string> link;
  link.PushBack("hello1");
  link.PushBack("hello2");
  link.PushBack("hello2");
  link.PushBack("hello3");
  link.PushBack("hello4");

  EXPECT_EQ(link.Front(), "hello1");
  EXPECT_EQ(link.Back(), "hello4");
  
  link.Reverse();

  EXPECT_EQ(link.Front(), "hello4");
  EXPECT_EQ(link.Back(), "hello1");

}


}


void test_node(void) {
  Node<int> n1(1);
  Node<int> n2(2);
  Node<int> n3(3);
  
  std::cout << n1.data() << std::endl;
  std::cout << n2.data() << std::endl;
  std::cout << n3.data() << std::endl;

  n1.set_data(11);
  n2.set_data(22);
  n3.set_data(33);

  n1.set_next(&n2);
  n2.set_next(&n3);

  std::cout << n1.data() << std::endl;
  std::cout << (n1.next())->data() << std::endl;
  std::cout << (n2.next())->data() << std::endl;

}


void test_create_list(void) {
	LinkedList<int> link;
}


void test_push_front(void) {
  LinkedList<int> link;
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);
  link.PushFront(5);
  
  link.PrintList();

}

void test_empty(void) {
  LinkedList<int> link;
  assert(link.Empty() == true);
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);
  link.PushFront(5);
  
  assert(link.Empty() == false);
}

void test_size(void) {
  LinkedList<int> link;
  assert(link.Size() == 0);
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);
  link.PushFront(5);
  
  assert(link.Size() == 5);
}



void test_value_at(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);
  link.PushFront(5);
  
  link.PrintList();
  assert(link.ValueAt(0) == 5);
  assert(link.ValueAt(2) == 3);
  assert(link.ValueAt(4) == 1);
}


void test_pop_front(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  
  link.PrintList();
  
  assert(link.ValueAt(0) == 3);
  assert(link.Size() == 3);
  
  link.PopFront();
  link.PrintList();
  
  assert(link.ValueAt(0) == 2);
  assert(link.Size() == 2);
}

void test_push_back(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  
  link.PrintList();
  
  link.PushBack(2);
  link.PushBack(3);

  link.PrintList();
  
  assert(link.ValueAt(3) == 2);
  assert(link.Size() == 5);
}

void test_pop_back(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  
  link.PrintList();
  
  assert(link.PopBack() == 1);
  link.PrintList();
  assert(link.Size() == 2);
}


void test_front(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  
  assert(link.Front() == 3);
}



void test_back(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  
  assert(link.Back() == 1);
}


void test_insert(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
 
  link.Insert(0, 0);
  link.Insert(1, 22);
  link.Insert(4, 22);
  
  link.PrintList();
}



void test_erase(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);
  link.PushFront(5);
 
  link.PrintList();
  link.Erase(0);
  link.Erase(3);
  link.Erase(1);
}



void test_value_end(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);

  assert(link.ValueNFromEnd(1) == 1);
  assert(link.ValueNFromEnd(3) == 3);
  assert(link.ValueNFromEnd(4) == 4);
}


void test_reverse(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);

  link.PrintList();

  link.Reverse();

  link.PrintList();
}


void test_remove_value(void) {
  LinkedList<int> link;
  
  link.PushFront(1);
  link.PushFront(2);
  link.PushFront(2);
  link.PushFront(3);
  link.PushFront(4);


  link.PrintList();

  link.RemoveValue(2);
  
  link.PrintList();
}




void run_all_test(void) {
  //test_node();
  //test_create_list();
  //test_push_front();
  //test_empty();
  //test_size();
  //test_value_at();
  //test_pop_front();
  //test_push_back();
  //test_pop_back();
  //test_front();
  //test_back();
  //test_insert();
  //test_erase();
  //test_erase();
  //test_value_end();
  //test_reverse();
  test_remove_value();
}


int main(int argc, char **argv) {
  //run_all_test();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




