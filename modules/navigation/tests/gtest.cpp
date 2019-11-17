#include <gtest/gtest.h>
#include "navigation_tests.h"

TEST(navigation_basic_test, Trivial)
{
    ASSERT_EQ(0, navigation_basic_test());
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}