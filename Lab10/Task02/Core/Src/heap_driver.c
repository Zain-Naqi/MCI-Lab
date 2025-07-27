#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used
static uint8_t block_map[BLOCK_COUNT];

void heap_init(void) {
    memset(block_map, 0, sizeof(block_map));
}

void* heap_alloc(size_t size) {
    size_t needed_blocks = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    for (size_t i = 0; i <= BLOCK_COUNT - needed_blocks; i++) {
        int found = 1;
        for (size_t j = 0; j < needed_blocks; j++) {
            if (block_map[i + j]) {
                found = 0;
                break;
            }
        }

        if (found) {
            for (size_t j = 0; j < needed_blocks; j++) {
                block_map[i + j] = 1;
            }
            return (void*)(HEAP_START_ADDR + i * BLOCK_SIZE);
        }
    }

    return NULL;  // No sufficient block found
}

void heap_free(void* ptr) {
    if (ptr == NULL) return;

    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t offset = addr - (uintptr_t)HEAP_START_ADDR;

    if (offset >= HEAP_SIZE || (offset % BLOCK_SIZE) != 0)
        return;  // Invalid pointer

    size_t start_block = offset / BLOCK_SIZE;

    // Free all consecutive used blocks until a free block is found
    for (size_t i = start_block; i < BLOCK_COUNT && block_map[i]; i++) {
        block_map[i] = 0;
    }
}
