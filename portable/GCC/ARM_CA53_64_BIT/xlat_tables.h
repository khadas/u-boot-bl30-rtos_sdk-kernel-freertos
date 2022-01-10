/*
 * Copyright (c) 2021-2022 Amlogic, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __XLAT_TABLES_H__
#define __XLAT_TABLES_H__

/* Miscellaneous MMU related constants */
#define NUM_2MB_IN_GB		(1 << 9)
#define NUM_4K_IN_2MB		(1 << 9)
#define NUM_GB_IN_4GB		(1 << 2)

#define TWO_MB_SHIFT		21
#define ONE_GB_SHIFT		30
#define FOUR_KB_SHIFT		12

#define ONE_GB_INDEX(x)		((x) >> ONE_GB_SHIFT)
#define TWO_MB_INDEX(x)		((x) >> TWO_MB_SHIFT)
#define FOUR_KB_INDEX(x)	((x) >> FOUR_KB_SHIFT)

#define INVALID_DESC		0x0
#define BLOCK_DESC		0x1 /* Table levels 0-2 */
#define TABLE_DESC		0x3 /* Table levels 0-2 */
#define PAGE_DESC		0x3 /* Table level 3 */

#define FIRST_LEVEL_DESC_N	ONE_GB_SHIFT
#define SECOND_LEVEL_DESC_N	TWO_MB_SHIFT
#define THIRD_LEVEL_DESC_N	FOUR_KB_SHIFT

#define XN			(1ull << 2)
#define PXN			(1ull << 1)
#define CONT_HINT		(1ull << 0)

#define UPPER_ATTRS(x)		(x & 0x7) << 52
#define NON_GLOBAL		(1 << 9)
#define ACCESS_FLAG		(1 << 8)
#define NSH			(0x0 << 6)
#define OSH			(0x2 << 6)
#define ISH			(0x3 << 6)

#define PAGE_SIZE_SHIFT		FOUR_KB_SHIFT
#define PAGE_SIZE		(1 << PAGE_SIZE_SHIFT)
#define PAGE_SIZE_MASK		(PAGE_SIZE - 1)
#define IS_PAGE_ALIGNED(addr)	(((addr) & PAGE_SIZE_MASK) == 0)

#define XLAT_ENTRY_SIZE_SHIFT	3 /* Each MMU table entry is 8 bytes (1 << 3) */
#define XLAT_ENTRY_SIZE		(1 << XLAT_ENTRY_SIZE_SHIFT)

#define XLAT_TABLE_SIZE_SHIFT	PAGE_SIZE_SHIFT
#define XLAT_TABLE_SIZE		(1 << XLAT_TABLE_SIZE_SHIFT)

#define XLAT_TABLE_LEVEL_MIN	0

#define XLAT_TABLE_LEVEL_MAX	3

/* Values for number of entries in each MMU translation table */
#define XLAT_TABLE_ENTRIES_SHIFT (XLAT_TABLE_SIZE_SHIFT - XLAT_ENTRY_SIZE_SHIFT)
#define XLAT_TABLE_ENTRIES	(1 << XLAT_TABLE_ENTRIES_SHIFT)
#define XLAT_TABLE_ENTRIES_MASK	(XLAT_TABLE_ENTRIES - 1)

/* Values to convert a memory address to an index into a translation table */
#define L3_XLAT_ADDRESS_SHIFT	PAGE_SIZE_SHIFT
#define L2_XLAT_ADDRESS_SHIFT	(L3_XLAT_ADDRESS_SHIFT + XLAT_TABLE_ENTRIES_SHIFT)
#define L1_XLAT_ADDRESS_SHIFT	(L2_XLAT_ADDRESS_SHIFT + XLAT_TABLE_ENTRIES_SHIFT)
#define L0_XLAT_ADDRESS_SHIFT	(L1_XLAT_ADDRESS_SHIFT + XLAT_TABLE_ENTRIES_SHIFT)

/*
 * AP[1] bit is ignored by hardware and is
 * treated as if it is One in EL2/EL3
 */
#define AP_RO			(0x1 << 5)
#define AP_RW			(0x0 << 5)

#define NS				(0x1 << 3)
#define ATTR_NON_CACHEABLE_INDEX	0x2
#define ATTR_DEVICE_INDEX		0x1
#define ATTR_IWBWA_OWBWA_NTR_INDEX	0x0
#define LOWER_ATTRS(x)			(((x) & 0xfff) << 2)
#define ATTR_NON_CACHEABLE		(0x44)
#define ATTR_DEVICE			(0x4)
#define ATTR_IWBWA_OWBWA_NTR		(0xff)
#define MAIR_ATTR_SET(attr, index)	(attr << (index << 3))

/*
 * Flags to override default values used to program system registers while
 * enabling the MMU.
 */
#define DISABLE_DCACHE		(1 << 0)

#ifndef __ASSEMBLY__
#include <stddef.h>
#include <stdint.h>

/* Helper macro to define entries for mmap_region_t. It creates
 * identity mappings for each region.
 */
#define MAP_REGION_FLAT(adr, sz, attr) MAP_REGION(adr, adr, sz, attr)

/* Helper macro to define entries for mmap_region_t. It allows to
 * re-map address mappings from 'pa' to 'va' for each region.
 */
#define MAP_REGION(pa, va, sz, attr) {(pa), (va), (sz), (attr)}

/*
 * Shifts and masks to access fields of an mmap_attr_t
 */
#define MT_TYPE_MASK	0x7
#define MT_TYPE(_attr)	((_attr) & MT_TYPE_MASK)
/* Access permissions (RO/RW) */
#define MT_PERM_SHIFT	3
/* Security state (SECURE/NS) */
#define MT_SEC_SHIFT	4
/* Access permissions for instruction execution (EXECUTE/EXECUTE_NEVER) */
#define MT_EXECUTE_SHIFT	5

#if ENABLE_KASAN
/* kasan */
#define MT_KASAN_ZERO_SHIFT	6
#define MT_KASAN_SHIFT		7
#endif

/*
 * Memory mapping attributes
 */
typedef enum  {
	/*
	 * Memory types supported.
	 * These are organised so that, going down the list, the memory types
	 * are getting weaker; conversely going up the list the memory types are
	 * getting stronger.
	 */
	MT_DEVICE,
	MT_NON_CACHEABLE,
	MT_MEMORY,
	/* Values up to 7 are reserved to add new memory types in the future */

	MT_RO		= 0 << MT_PERM_SHIFT,
	MT_RW		= 1 << MT_PERM_SHIFT,

	MT_SECURE	= 0 << MT_SEC_SHIFT,
	MT_NS		= 1 << MT_SEC_SHIFT,

	/*
	 * Access permissions for instruction execution are only relevant for
	 * normal read-only memory, i.e. MT_MEMORY | MT_RO. They are ignored
	 * (and potentially overridden) otherwise:
	 *  - Device memory is always marked as execute-never.
	 *  - Read-write normal memory is always marked as execute-never.
	 */
	MT_EXECUTE		= 0 << MT_EXECUTE_SHIFT,
	MT_EXECUTE_NEVER	= 1 << MT_EXECUTE_SHIFT,
#if ENABLE_KASAN
	MT_KASAN_ZERO	= 1 << MT_KASAN_ZERO_SHIFT,
	MT_KASAN	= 1 << MT_KASAN_SHIFT,
#endif
} mmap_attr_t;

#define MT_CODE		(MT_MEMORY | MT_RO | MT_EXECUTE)
#define MT_RO_DATA	(MT_MEMORY | MT_RO | MT_EXECUTE_NEVER)

#define KASAN_REGION_ATTR (MT_KASAN| MT_MEMORY | MT_RW | MT_SECURE)
#define KASAN_REGION_ZERO_ATTR (MT_KASAN|MT_KASAN_ZERO| MT_MEMORY | MT_RO | MT_SECURE)

/*
 * Structure for specifying a single region of memory.
 */
typedef struct mmap_region {
	unsigned long long	base_pa;
	uintptr_t		base_va;
	size_t			size;
	mmap_attr_t		attr;
} mmap_region_t;

/* Generic translation table APIs */
void init_xlat_tables(void);
void mmap_add_region(unsigned long long base_pa, uintptr_t base_va,
				size_t size, unsigned int attr);
void mmap_add_region_ex(mmap_region_t *mmtbl, int mmcnt,
			unsigned long long base_pa, uintptr_t base_va,
			size_t size, unsigned int attr);
void mmap_add(const mmap_region_t *mm);

/* AArch64 specific translation table APIs */
void enable_mmu_el1(unsigned int flags);
void enable_mmu_el3(unsigned int flags);
void print_mmap(void);
#if ENABLE_KASAN
void init_kasan_xlation_table(mmap_region_t *mm,
					uintptr_t base_va,
					uintptr_t max_va,
					uint64_t *table,
					int level);
void init_kasan_xlat_tables(mmap_region_t *mm,
			uintptr_t base_va,
			uintptr_t max_va);
#endif
#endif /*__ASSEMBLY__*/
#endif /* __XLAT_TABLES_H__ */
