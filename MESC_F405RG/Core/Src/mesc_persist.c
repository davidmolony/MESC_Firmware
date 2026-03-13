#include "mesc_persist.h"

#include <string.h>

#define MESC_PERSIST_HEADER_START  UINT32_C(0xDEADBEEF)
#define MESC_PERSIST_FOOTER_END    UINT32_C(0xDEADC0DE)
#define MESC_PERSIST_HEADER_VER    UINT32_C(0x00000001)

#define MESC_PERSIST_FNV1A_PRIME   UINT32_C(0x01000193)
#define MESC_PERSIST_FNV1A_OFFSET  UINT32_C(0x811C9DC5)

typedef struct {
    uint32_t start;
    uint32_t num_entries;
    uint32_t size;
    uint32_t version;
    uint32_t revision;
#ifdef ALLIGNED_DOUBLE_WORD
    uint32_t padding;
#endif
} __attribute__((packed)) MESC_PersistFlashHeader;

typedef struct {
    uint32_t end;
    uint32_t crc;
} __attribute__((packed)) MESC_PersistFlashFooter;

typedef struct MESC_PersistFlashVariable MESC_PersistFlashVariable;

struct MESC_PersistFlashVariable {
    void *variable;
    uint32_t type;
    uint16_t type_size;
    char const *name;
    uint32_t name_length;
    uint32_t flags;
    MESC_PersistFlashVariable *next;
#ifdef ALLIGNED_DOUBLE_WORD
    uint8_t padding;
#endif
} __attribute__((packed));

static uint32_t mesc_persist_fnv1a_init(void) {
    return MESC_PERSIST_FNV1A_OFFSET;
}

static uint32_t mesc_persist_fnv1a_process(uint32_t hash, uint8_t byte) {
    return (hash ^ byte) * MESC_PERSIST_FNV1A_PRIME;
}

static uint32_t mesc_persist_fnv1a_process_data(uint32_t hash, void const *ptr, uint32_t len) {
    uint8_t const *bytes = (uint8_t const *)ptr;
    uint32_t i;

    for (i = 0; i < len; ++i) {
        hash = mesc_persist_fnv1a_process(hash, bytes[i]);
    }

    return hash;
}

#ifdef ALLIGNED_DOUBLE_WORD
static uint32_t mesc_persist_padding(uint32_t num, uint32_t alignment) {
    uint32_t remainder = num % alignment;

    return remainder == 0U ? 0U : alignment - remainder;
}
#endif

static bool mesc_persist_contains_range(MESC_PersistDataset const *dataset, void const *ptr, uint32_t len) {
    uintptr_t const base = (uintptr_t)dataset->storage_base;
    uintptr_t const end = base + dataset->storage_size;
    uintptr_t const start = (uintptr_t)ptr;
    uintptr_t const stop = start + len;

    if (start < base || start > end) {
        return false;
    }

    if (stop < start || stop > end) {
        return false;
    }

    return true;
}

static MESC_PersistFlashHeader const *mesc_persist_find_header(
    uint8_t const *storage_base,
    uint32_t storage_size,
    bool find_revision,
    uint32_t revision
) {
    MESC_PersistFlashHeader const *header = (MESC_PersistFlashHeader const *)storage_base;
    uint8_t const *section = storage_base;
    uint32_t last_size = 0U;

    while ((section < (storage_base + storage_size)) &&
           (header->start == MESC_PERSIST_HEADER_START) &&
           (header->size >= sizeof(*header) + sizeof(MESC_PersistFlashFooter))) {
        if (find_revision && header->revision == revision) {
            return header;
        }

        last_size = header->size;
        section += header->size;
        if (section >= (storage_base + storage_size)) {
            break;
        }
        header = (MESC_PersistFlashHeader const *)section;
    }

    if (find_revision || last_size == 0U) {
        return NULL;
    }

    return (MESC_PersistFlashHeader const *)(section - last_size);
}

static uint32_t mesc_persist_calculate_crc(MESC_PersistDataset const *dataset) {
    MESC_PersistFlashHeader const *header;
    MESC_PersistFlashVariable const *flash_var;
    MESC_PersistFlashFooter const *footer;
    uint32_t crc;
    uint32_t i;

    if (dataset == NULL || dataset->dataset_base == NULL) {
        return 0U;
    }

    header = (MESC_PersistFlashHeader const *)dataset->dataset_base;
    footer = (MESC_PersistFlashFooter const *)(dataset->dataset_base + dataset->dataset_size - sizeof(*footer));
    flash_var = (MESC_PersistFlashVariable const *)(header + 1);

    crc = mesc_persist_fnv1a_init();
    crc = mesc_persist_fnv1a_process_data(crc, header, sizeof(*header));

    for (i = 0; i < header->num_entries; ++i) {
        if (!mesc_persist_contains_range(dataset, flash_var, sizeof(*flash_var))) {
            return 0U;
        }

        crc = mesc_persist_fnv1a_process_data(crc, flash_var, sizeof(*flash_var));
        flash_var = flash_var->next;
    }

    flash_var = (MESC_PersistFlashVariable const *)(header + 1);

    for (i = 0; i < header->num_entries; ++i) {
        uint32_t payload_size = flash_var->name_length + 1U + flash_var->type_size;

#ifdef ALLIGNED_DOUBLE_WORD
        payload_size += mesc_persist_padding(payload_size, 8U);
#endif

        if (!mesc_persist_contains_range(dataset, flash_var->name, payload_size)) {
            return 0U;
        }

        crc = mesc_persist_fnv1a_process_data(crc, flash_var->name, payload_size);
        flash_var = flash_var->next;
    }

    return mesc_persist_fnv1a_process_data(crc, footer, sizeof(*footer) - sizeof(footer->crc));
}

static bool mesc_persist_open(
    MESC_PersistDataset *dataset,
    void const *storage_base,
    uint32_t storage_size,
    bool find_revision,
    uint32_t revision
) {
    MESC_PersistFlashHeader const *header;
    MESC_PersistFlashFooter const *footer;

    if (dataset == NULL || storage_base == NULL || storage_size == 0U) {
        return false;
    }

    memset(dataset, 0, sizeof(*dataset));
    dataset->storage_base = (uint8_t const *)storage_base;
    dataset->storage_size = storage_size;

    header = mesc_persist_find_header(dataset->storage_base, dataset->storage_size, find_revision, revision);
    if (header == NULL || header->start != MESC_PERSIST_HEADER_START || header->version != MESC_PERSIST_HEADER_VER) {
        return false;
    }

    if (!mesc_persist_contains_range(dataset, header, header->size)) {
        return false;
    }

    footer = (MESC_PersistFlashFooter const *)((uint8_t const *)header + header->size - sizeof(*footer));
    if (footer->end != MESC_PERSIST_FOOTER_END) {
        return false;
    }

    dataset->dataset_base = (uint8_t const *)header;
    dataset->dataset_size = header->size;
    dataset->revision = header->revision;
    dataset->entry_count = header->num_entries;
    dataset->crc = footer->crc;

    return true;
}

bool MESC_PersistOpenLatest(MESC_PersistDataset *dataset, void const *storage_base, uint32_t storage_size) {
    return mesc_persist_open(dataset, storage_base, storage_size, false, 0U);
}

bool MESC_PersistOpenRevision(MESC_PersistDataset *dataset, void const *storage_base, uint32_t storage_size, uint32_t revision) {
    return mesc_persist_open(dataset, storage_base, storage_size, true, revision);
}

bool MESC_PersistValidate(MESC_PersistDataset const *dataset) {
    MESC_PersistFlashHeader const *header;

    if (dataset == NULL || dataset->dataset_base == NULL) {
        return false;
    }

    header = (MESC_PersistFlashHeader const *)dataset->dataset_base;
    if (header->start != MESC_PERSIST_HEADER_START || header->version != MESC_PERSIST_HEADER_VER) {
        return false;
    }

    return mesc_persist_calculate_crc(dataset) == dataset->crc;
}

bool MESC_PersistGetEntry(MESC_PersistDataset const *dataset, uint32_t index, MESC_PersistEntry *entry) {
    MESC_PersistFlashHeader const *header;
    MESC_PersistFlashVariable const *flash_var;
    uint32_t i;

    if (dataset == NULL || entry == NULL || dataset->dataset_base == NULL) {
        return false;
    }

    header = (MESC_PersistFlashHeader const *)dataset->dataset_base;
    if (index >= header->num_entries) {
        return false;
    }

    flash_var = (MESC_PersistFlashVariable const *)(header + 1);
    for (i = 0; i < index; ++i) {
        if (!mesc_persist_contains_range(dataset, flash_var, sizeof(*flash_var))) {
            return false;
        }
        flash_var = flash_var->next;
    }

    if (!mesc_persist_contains_range(dataset, flash_var, sizeof(*flash_var))) {
        return false;
    }

    if (!mesc_persist_contains_range(dataset, flash_var->name, flash_var->name_length + 1U)) {
        return false;
    }

    if (!mesc_persist_contains_range(dataset, flash_var->variable, flash_var->type_size)) {
        return false;
    }

    entry->name = flash_var->name;
    entry->name_length = flash_var->name_length;
    entry->flags = flash_var->flags;
    entry->type = flash_var->type;
    entry->type_size = flash_var->type_size;
    entry->value = flash_var->variable;

    return true;
}

bool MESC_PersistFindEntry(MESC_PersistDataset const *dataset, char const *name, MESC_PersistEntry *entry) {
    MESC_PersistEntry current;
    uint32_t i;

    if (dataset == NULL || name == NULL) {
        return false;
    }

    for (i = 0; i < dataset->entry_count; ++i) {
        if (!MESC_PersistGetEntry(dataset, i, &current)) {
            return false;
        }

        if (strcmp(current.name, name) == 0) {
            if (entry != NULL) {
                *entry = current;
            }
            return true;
        }
    }

    return false;
}
