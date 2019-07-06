#include "ringbuffer.c"
#include <gtest/gtest.h>
 
TEST(Ringbuffer_rb_init_Test, PositiveNos) {
    struct ringbuffer_t rb;
    ASSERT_EQ(0, rb_init(&rb,8));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}