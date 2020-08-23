#include <iostream>
#include <cassert>
#include <gtest/gtest.h>

namespace {

class SizeTest : public ::testing::Test {};

TEST_F(SizeTest, Size) {
  //EXPECT_EQ(link.Size(), 1);
  //EXPECT_FALSE(link.Empty());
}

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




