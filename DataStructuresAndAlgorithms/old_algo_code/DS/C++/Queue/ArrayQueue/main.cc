#include "array_queue.h"

#include <iostream>
#include <cassert>
#include <gtest/gtest.h>


namespace {

class ArrayQueueTest : public ::testing::Test {};

TEST_F(ArrayQueueTest, New) {
  ArrayQueue queue;
  EXPECT_TRUE(queue.Empty());
}


TEST_F(ArrayQueueTest, Enqueue) {
  ArrayQueue queue;
  
  for (int i = 0; i < kMaxSize; i++)
    queue.Enqueue(i + 1);

  EXPECT_TRUE(queue.Full());

}

TEST_F(ArrayQueueTest, Dequeue) {
  ArrayQueue queue;
  
  for (int i = 0; i < kMaxSize; i++)
    queue.Enqueue(i + 1);

  EXPECT_TRUE(queue.Full());

  for (int i = 0; i < kMaxSize; i++)
    queue.Dequeue();
  
  EXPECT_TRUE(queue.Empty());
}


}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


