#include "ringbuffer.c"
#include "integration_test_cases.cpp"
#include <gtest/gtest.h>
 
TEST(ringbuffer_basic_test, PositiveNos) {
    struct ringbuffer_t rb;
    ASSERT_EQ(0, ringbuffer_basic_test());
    ASSERT_EQ(0, rb_init(&rb, 8));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}