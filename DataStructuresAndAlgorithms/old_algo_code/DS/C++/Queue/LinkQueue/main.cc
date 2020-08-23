#include "linked_queue.h"
#include "linked_queue.cc"

#include <iostream>
#include <cassert>
#include <gtest/gtest.h>

#include "node.h"

namespace {

class LinkQueueTest : public ::testing::Test {};

TEST_F(LinkQueueTest, Enqueue) {
  LinkQueue<int> queue;
  EXPECT_TRUE(queue.Empty()); 
  
  queue.Enqueue(1);
  queue.Enqueue(2);
  queue.Enqueue(3);
  queue.PrintQueue(); 
  EXPECT_FALSE(queue.Empty());
}

TEST_F(LinkQueueTest, Dequeue) {
  LinkQueue<int> queue;
  EXPECT_TRUE(queue.Empty()); 
  
  queue.Enqueue(1);
  queue.Enqueue(2);
  queue.Enqueue(3);
  queue.PrintQueue(); 

  EXPECT_EQ(queue.Dequeue(), 1);
  EXPECT_EQ(queue.Dequeue(), 2);
  EXPECT_EQ(queue.Dequeue(), 3);
  
  EXPECT_TRUE(queue.Empty());
}

TEST_F(LinkQueueTest, Empty) {
  LinkQueue<int> queue;
  EXPECT_TRUE(queue.Empty()); 
  
  queue.Enqueue(1);
  
  EXPECT_FALSE(queue.Empty());
}



}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




