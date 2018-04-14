/*
   Copyright (C) 2007-2012 by krasnop@bellsouth.net (Alexei Krasnopolski)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "memory.hpp"
#include "pc.hpp"

BYTE MemoryManager::memory[HEAP_MEMORY_SIZE];
MemoryManager* mm;

int line; //only for debug - thread unsafe

void* operator new(size_t sz) {
  char out[80]; line = 2;
  Display::sprintf(out, ">>> new sz= %d    ", sz);
  debugPrint(line*80, out);
  void* m = mm->malloc(sz);
  Display::sprintf(out, "<<< new m= %d    ", m);
  debugPrint((line+3)*80, out);
  return m;
}

void operator delete(void* m) {
  char out[80]; line = 7;
  Display::sprintf(out, ">>> delete ptr= %d    ", m);
  debugPrint(line*80, out);
  mm->free(m);
  Display::sprintf(out, "<<< delete");
  debugPrint((line+3)*80, out);
}

void* operator new[](size_t sz) {
  char out[80]; line = 12;
  Display::sprintf(out, ">>> new[] sz= %d    ", sz);
  debugPrint(line*80, out);
  void* m = mm->malloc(sz);
  Display::sprintf(out, "<<< new[] m= %d    ", m);
  debugPrint((line+3)*80, out);
  return m;
}

void operator delete[](void* m) {
  char out[80]; line = 17;
  Display::sprintf(out, ">>> delete[] ptr= %d    ", m);
  debugPrint(line*80, out);
  mm->free(m);
  Display::sprintf(out, "<<< delete[]");
  debugPrint((line+3)*80, out);
}

void *
MemoryManager::malloc(size_t sz) {
  char out[80];
  Display::sprintf(out, ">>> malloc sz = %d, new_alloc = %d    ", sz, new_alloc);
  debugPrint((line+1)*80, out);

  WORD nbytes = (WORD) (sz + sizeof(MemoryControlBlock));
  MemoryControlBlock* mcb_location;
  BYTE *alloc = 0;
  BYTE *mem_location = start;
  while (mem_location < new_alloc) {
    mcb_location = (MemoryControlBlock*) mem_location;
    if (mcb_location->is_allocated == 0 && mcb_location->size >= nbytes) {
      mcb_location->is_allocated = 1;
      alloc = mem_location;
      break;
    }
    mem_location += mcb_location->size;
  }

  if (alloc == 0) {
    if (new_alloc + nbytes <= end) {
      alloc = new_alloc;
      new_alloc += nbytes;
      mcb_location = (MemoryControlBlock*) alloc;
      mcb_location->is_allocated = 1;
      mcb_location->size = nbytes;
      alloc += sizeof(MemoryControlBlock);
    }
  } else {
    alloc += sizeof(MemoryControlBlock);
  }
  Display::sprintf(out, "<<< malloc returns = %d", alloc);
  debugPrint((line+2)*80, out);

  return alloc;
};

void
MemoryManager::free(void *ptr) {
  char out[80];
  Display::sprintf(out, ">>> free ptr = %d, start = %d end = %d   ", ptr, start, end);
  debugPrint((line+1)*80, out);
  if (ptr > start && ptr < end) {
    MemoryControlBlock *msb = (MemoryControlBlock*) ((BYTE*) ptr - sizeof(MemoryControlBlock));
    msb->is_allocated = 0;
  }
  Display::sprintf(out, "<<< free ");
  debugPrint((line+2)*80, out);
};
