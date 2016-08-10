#include "sys_helper.h"

void delay(uint64_t us) {
    uint64_t start = usec(), cur = usec();
    while (cur - start < us) {
        cur = usec();
    }
}

void delay_ms(uint32_t ms) {
    delay(ms * 1000);
}
