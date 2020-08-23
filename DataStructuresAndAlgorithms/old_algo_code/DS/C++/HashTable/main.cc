#include "hash_table.h"
#include "hash_table.cc"

#include <iostream>
#include <cassert>
#include <gtest/gtest.h>


namespace {

class HashTableTest : public ::testing::Test {};

TEST_F(HashTableTest, Add) {
  HashTable table(5);
  table.Add(12);
  table.Add(67);
  table.Add(56);
  table.Add(16);
  table.Add(25);
  table.PrintDebug();
}


TEST_F(HashTableTest, Get) {
  HashTable table(5);
  table.Add(12);
  table.Add(67);
  table.Add(56);
  table.Add(16);
  table.Add(25);
  EXPECT_EQ(table.Get(12), 2);

}


TEST_F(HashTableTest, Exist) {
  HashTable table(5);
  table.Add(12);
  table.Add(67);
  table.Add(56);
  table.Add(16);
  table.Add(25);
  EXPECT_TRUE(table.Exist(12));
  EXPECT_FALSE(table.Exist(13));

}


TEST_F(HashTableTest, Remove) {
  HashTable table(5);
  table.Add(12);
  table.Add(67);
  table.Add(56);
  table.Add(16);
  table.Add(25);
  
  table.PrintDebug();

  table.Remove(12);
  table.Remove(67);
  table.Remove(56);
  table.Remove(16);
  table.Remove(25);

  table.PrintDebug();
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




