#include "config_util.c"
#include "integration_test_cases.cpp"
#include <gtest/gtest.h>
 
TEST(config_util_basic_test, Trivial) {
    ASSERT_EQ(0, config_util_basic_test());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}