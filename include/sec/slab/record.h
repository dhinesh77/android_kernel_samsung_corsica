/**
 * COPYRIGHT (C)  SAMSUNG Electronics CO., LTD (Suwon, Korea). 2009
 * All rights are reserved. Reproduction and redistiribution in whole or
 * in part is prohibited without the written consent of the copyright owner.
 */

/**
 * Slab record.
 * 
 * @author kumhyun.cho@samsung.com
 * @since 2012.03.19
 */

#ifndef __SEC_MM_SLAB_H__
#define __SEC_MM_SLAB_H__

#ifndef SEC_SLAB_RECORD_SIZE
#define SEC_SLAB_RECORD_SIZE 1024
#endif

typedef struct {
	void* obj;
	void* caller;
} sec_slab_record_entry;

typedef struct {
	sec_slab_record_entry entries[SEC_SLAB_RECORD_SIZE];
	int index;
} sec_slab_record;

int sec_slab_record_index_last(int index);

int sec_slab_record_add(sec_slab_record* list, void* obj, void* caller);

int sec_slab_record_del(sec_slab_record* rec, void* obj);

#ifdef SEC_SLAB_RECORD_LOG

void sec_slab_record_log(sec_slab_record* list);

#endif

#endif
