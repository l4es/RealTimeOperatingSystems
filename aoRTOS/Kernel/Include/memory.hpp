#ifndef _MEMORY_HPP_
#define _MEMORY_HPP_

#include <stddef.h>
#include "commonDef.hpp"
#include "pc.hpp"

class MemoryManager {
  struct MemoryControlBlock {
    WORD is_allocated;
    WORD size;
  };
  private:
    BYTE *start, *new_alloc, *end;
    static BYTE memory[HEAP_MEMORY_SIZE];
  public:
           MemoryManager() { start = new_alloc = memory; end = start + HEAP_MEMORY_SIZE; };
    void * malloc(size_t sz);
    void   free(void*);
};

#endif /* _MEMORY_HPP_ */
