#include "config_util.h"

struct car_t {
    char name[8];
    int age;
    int seat;
};

int config_util_basic_test(void)
{
    uint32_t idx = 0;
    uint32_t rc = 0;
    struct car_t cars[32];
    int numEntries;
    char specifier[] = "STRING:8 NUMBER:4 NUMBER:4";
    memset(cars, 0, sizeof(cars));

    rc = read_config(cars, &numEntries, "tests/carlist.cfg", specifier);
    if (numEntries != 3)
        rc |= 1;
    if(0 != strcmp(cars[0].name, "XC40"))
        rc |= (1 << 1);
    if(0 != strcmp(cars[1].name, "BMW"))
        rc |= (1 << 2);
    if(0 != strcmp(cars[2].name, "TOYOTA"))
        rc |= (1 << 3);
    for (idx = 0; idx < numEntries; idx++)
    {
        printf("%s %d %d\n", cars[idx].name, cars[idx].age, cars[idx].seat);
        if(cars[idx].age != 2019)
            rc |= (1 << (4 + idx));
        if(cars[idx].seat != (4+idx))
            rc |= (1 << (4 + idx));
    }
    if(rc)
        printf("Test %s FAIL 0x%x\n", __FUNCTION__, rc);
    return rc;
}