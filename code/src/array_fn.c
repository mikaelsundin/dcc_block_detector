#include "array_fn.h"

/**
 * @brief get minimum value from array
 * @param bfr pointer to array
 * @param offset offset between every element to check
 * @param count number of elements to check
 */
uint16_t get_array16_min(uint16_t *bfr, uint8_t offset, uint8_t count){
    uint16_t val = UINT16_MAX;

    while(count--){
        val = *bfr <= val ? *bfr : val;
        bfr += offset;
    }

    return val;
}

/**
 * @brief get maximum value from array
 * @param bfr pointer to array
 * @param offset offset between every element to check
 * @param count number of elements to check
 */
uint16_t get_array16_max(uint16_t *bfr, uint8_t offset, uint8_t count){
    uint16_t val = 0;

    while(count--){
        val = *bfr >= val ? *bfr : val;
        bfr += offset;
    }

    return val;
}

