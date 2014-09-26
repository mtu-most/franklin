#ifdef DEFINE_MAIN
#include "firmware.ino"
#include "firmware.h"

// Memory handling
void _mem_alloc(uint16_t size, void **target) {
	if (size + sizeof(Memrecord) > sizeof(storage) - mem_used) {
		debug("Out of memory");
		*target = NULL;
		return;
	}
	Memrecord *record = reinterpret_cast <Memrecord *>(&storage[mem_used]);
	record->size = size;
	record->target = target;
	*record->target = &record[1];
	mem_used += sizeof(Memrecord) + size;
}

void _mem_retarget(void **target, void **newtarget) {
	if (*target == NULL) {
		*newtarget = NULL;
		return;
	}
	Memrecord *record = &reinterpret_cast <Memrecord *>(*target)[-1];
	*newtarget = *target;
	*target = NULL;
	record->target = newtarget;
}

/*
void _mem_dump() {
	uint16_t start = unsigned(&storage);
	debug("Memory dump.  Total size: %x, used %x, storage at %x", DYNAMIC_MEM_SIZE, mem_used, start);
	uint16_t addr = 0;
	while (addr < mem_used) {
		Memrecord *record = reinterpret_cast <Memrecord *>(&storage[addr]);
		if (unsigned(*record->target) - 4 == start + addr)
			debug("  Record at %x+%x=%x, size %x, pointer at %x", addr, start, start + addr, record->size, unsigned(record->target));
		else
			debug("  Record at %x+%x=%x, size %x, pointer at %x, pointing at %x+4", addr, start, start + addr, record->size, unsigned(record->target), unsigned(*record->target) - 4);
		addr += sizeof(Memrecord) + record->size;
	}
}
*/

void _mem_free(void **target) {
	if (*target == NULL) {
		return;
	}
	Memrecord *record = &reinterpret_cast <Memrecord *>(*target)[-1];
	*target = NULL;
	uint16_t oldsize = record->size;
	uint16_t start = reinterpret_cast <char *>(record) - storage + sizeof(Memrecord) + record->size;
	Memrecord *current = reinterpret_cast <Memrecord *>(&storage[start]);
	while (start < mem_used) {
		Memrecord *next = current->next();
		char *src = reinterpret_cast <char *>(current);
		char *dst = reinterpret_cast <char *>(record);
		uint16_t sz = current->size + sizeof(Memrecord);
		for (uint16_t i = 0; i < sz; ++i)
			dst[i] = src[i];
		*record->target = &record[1];
		// record is new location of moved part.
		// current is old location.
		// next is location of next part.
		for (Memrecord *m = reinterpret_cast <Memrecord *>(storage); m <= record; m = m->next()) {
			if (reinterpret_cast <char *>(m->target) >= reinterpret_cast <char *>(current) && reinterpret_cast <char *>(m->target) < reinterpret_cast <char *>(current) + sizeof(Memrecord) + record->size) {
				m->target = reinterpret_cast <void **>(&reinterpret_cast <char *>(m->target)[-oldsize - sizeof(Memrecord)]);
			}
		}
		for (Memrecord *m = reinterpret_cast <Memrecord *>(next); reinterpret_cast <char *>(m) < reinterpret_cast <char *>(&storage[mem_used]); m = m->next()) {
			if (reinterpret_cast <char *>(m->target) >= reinterpret_cast <char *>(current) && reinterpret_cast <char *>(m->target) < reinterpret_cast <char *>(current) + sizeof(Memrecord) + record->size) {
				m->target = reinterpret_cast <void **>(&reinterpret_cast <char *>(m->target)[-oldsize - sizeof(Memrecord)]);
			}
		}
		record = reinterpret_cast <Memrecord *>(&reinterpret_cast <char *>(record)[sz]);
		current = next;
		start += sz;
	}
	mem_used -= sizeof(Memrecord) + oldsize;
}

int main(int argc, char **argv) {
	setup();
	while(true)
		loop();
}

#endif
