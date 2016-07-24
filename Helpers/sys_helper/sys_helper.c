#include "sys_helper.h"

void delay(uint64_t us) {
    uint64_t start = usec(), cur = usec();
    while (cur - start < us) {
        cur = usec();
    }
}
