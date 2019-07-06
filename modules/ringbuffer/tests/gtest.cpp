#include "ringbuffer.c"
#include "integration_test_cases.cpp"
#include <gtest/gtest.h>
 
TEST(ringbuffer_basic_test, Trivial) {
    struct ringbuffer_t rb;
    ASSERT_EQ(0, ringbuffer_basic_test());
    ASSERT_EQ(0, rb_init(&rb, 8));
}

TEST(rb_init, Trivial) {
    struct ringbuffer_t rb;
    ASSERT_EQ(0, rb_init(&rb, 8));
    ASSERT_EQ(-1, rb_init(&rb, 7));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}