#ifndef MESC_PERSIST_H
#define MESC_PERSIST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t const *storage_base;
    uint32_t storage_size;
    uint8_t const *dataset_base;
    uint32_t dataset_size;
    uint32_t revision;
    uint32_t entry_count;
    uint32_t crc;
} MESC_PersistDataset;

typedef struct {
    char const *name;
    uint32_t name_length;
    uint32_t flags;
    uint32_t type;
    uint16_t type_size;
    void const *value;
} MESC_PersistEntry;

bool MESC_PersistOpenLatest(MESC_PersistDataset *dataset, void const *storage_base, uint32_t storage_size);
bool MESC_PersistOpenRevision(MESC_PersistDataset *dataset, void const *storage_base, uint32_t storage_size, uint32_t revision);
bool MESC_PersistValidate(MESC_PersistDataset const *dataset);
bool MESC_PersistGetEntry(MESC_PersistDataset const *dataset, uint32_t index, MESC_PersistEntry *entry);
bool MESC_PersistFindEntry(MESC_PersistDataset const *dataset, char const *name, MESC_PersistEntry *entry);

#endif
