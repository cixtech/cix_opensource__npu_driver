// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/types.h>
#include <asm/cacheflush.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_iommu.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/iommu.h>
#include <linux/bitmap.h>
#include <linux/version.h>
#include <linux/iova.h>
#include <linux/dma-mapping.h>
#include <linux/acpi.h>
#include "config.h"
#include "aipu_priv.h"
#include "aipu_mm.h"
#include "aipu_common.h"
#include "aipu_dma_buf.h"
#include "v2.h"

/*
 * DEBUG: V3 custom-IOVA lifecycle tracing (DPTSW-23453 SMMU event 0x10).
 * The V3_IOVA_DEBUG toggle and the v3dbg() macro now live in aipu_mm.h so
 * aipu_job_manager.c can share them. Logs ALLOC_IOVA/MAP/UNMAP/FREE here and
 * job dispatch / hold-TCB chain in the job manager. grep dmesg for "V3IOVA".
 *
 * aipu_mm_v3_dbg_iova_phys(): returns the backing phys for @iova, or 0 if the
 * IOVA is not currently mapped -- lets the job manager tell whether a TCB the
 * command pool is about to walk still has a live IOMMU mapping.
 */
phys_addr_t aipu_mm_v3_dbg_iova_phys(struct aipu_memory_manager *mm, u64 iova)
{
#if V3_IOVA_DEBUG
	if (!mm || !mm->iommu_domain || !mm->use_v3_custom_iova)
		return 0;
	return iommu_iova_to_phys(mm->iommu_domain, iova);
#else
	return 0;
#endif
}

#if V3_IOVA_DEBUG
/* DEBUG: short name for an aipu_mm_data_type value (see armchina_aipu.h). */
static const char *aipu_mm_v3_dtype_str(u32 dt)
{
	static const char * const names[] = {
		"NONE", "TEXT", "RODATA", "STACK",
		"STATIC", "REUSE", "TCB", "WEIGHT"
	};

	return dt < ARRAY_SIZE(names) ? names[dt] : "?";
}
#endif

/*
 * DEBUG: translate a custom-path IOVA to the kernel VA that backs it (each
 * physical block is vmap()'d at alloc time).  Must run in process context --
 * it takes buffer_mutex/phy_mutex -- so callers in atomic context (e.g. the
 * job-manager dispatch spinlock) must defer to after unlock.
 */
void *aipu_mm_v3_dbg_iova_to_va(struct aipu_memory_manager *mm, u64 iova)
{
#if V3_IOVA_DEBUG
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;
	void *va = NULL;

	if (!mm || !mm->use_v3_custom_iova)
		return NULL;

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (iova < buffer->iova_start ||
		    iova >= buffer->iova_start + buffer->iova_size)
			continue;

		mutex_lock(&buffer->phy_mutex);
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			if (block->va && iova >= block->iova_start &&
			    iova < block->iova_start + block->size) {
				va = (u8 *)block->va + (iova - block->iova_start);
				break;
			}
		}
		mutex_unlock(&buffer->phy_mutex);
		break;
	}
	mutex_unlock(&mm->buffer_mutex);
	return va;
#else
	return NULL;
#endif
}

/*
 * DEBUG: dump the pointer fields of a TCB and whether each is currently mapped.
 * The data-pointer fields (sp/pp/dp/cp/...) are 32-bit ASID-relative offsets;
 * we treat them as ASID-0 IOVAs for the mapped check, which is correct for the
 * demo (all buffers land in ASID 0).  A field that equals the faulting address
 * with m0 is the stale/dangling pointer.  Must run in process context.
 */
void aipu_mm_v3_dbg_dump_tcb(struct aipu_memory_manager *mm, u64 tcb_pa,
			     const char *tag)
{
#if V3_IOVA_DEBUG
	struct aipu_tcb *tcb;
	void *va;

	if (!mm || !mm->use_v3_custom_iova || !tcb_pa)
		return;

	va = aipu_mm_v3_dbg_iova_to_va(mm, tcb_pa);
	if (!va) {
		v3dbg(mm, "TCBDUMP %-4s pa=0x%llx <no kernel VA / unmapped>\n",
		      tag, tcb_pa);
		return;
	}

	tcb = (struct aipu_tcb *)va;
	if (IS_TASK_TCB(tcb->flag)) {
		v3dbg(mm, "TCBDUMP %-4s pa=0x%llx TASK flag=0x%x sp=0x%x(m%d) pp=0x%x(m%d) dp=0x%x(m%d) cp=0x%x(m%d) tcbp=0x%x(m%d)\n",
		      tag, tcb_pa, tcb->flag,
		      tcb->task.sp, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.sp),
		      tcb->task.pp, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.pp),
		      tcb->task.dp, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.dp),
		      tcb->task.cp, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.cp),
		      tcb->task.tcbp,
		      !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.tcbp));
		v3dbg(mm, "TCBDUMP %-4s      fmdp=0x%x(m%d) tap=0x%x(m%d) dap=0x%x(m%d) pap=0x%x(m%d) idp=0x%x(m%d) pprint=0x%x pprof=0x%x gp=0x%x\n",
		      tag,
		      tcb->task.fmdp, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.fmdp),
		      tcb->task.tap, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.tap),
		      tcb->task.dap, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.dap),
		      tcb->task.pap, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.pap),
		      tcb->task.idp, !!aipu_mm_v3_dbg_iova_phys(mm, tcb->task.idp),
		      tcb->task.pprint, tcb->task.pprofiler,
		      tcb->task.global_param);
	} else {
		v3dbg(mm, "TCBDUMP %-4s pa=0x%llx INIT/GRID flag=0x%x asid0=0x%llx asid1=0x%llx asid2=0x%llx asid3=0x%llx\n",
		      tag, tcb_pa, tcb->flag,
		      tcb->grid.asids[0].v64, tcb->grid.asids[1].v64,
		      tcb->grid.asids[2].v64, tcb->grid.asids[3].v64);
	}
#endif
}

/*
 * DEBUG: walk the whole TCB chain (head .. ltsk) following tcb->next, and for
 * every non-INIT TCB report any pointer field that is nonzero AND currently
 * unmapped (m0) -- that is the stale/dangling address the NPU faults on. Only
 * offending TCBs are printed, plus a one-line summary, to bound dmesg volume.
 * Walk is via tcb->next (a base-0 ASID-0 IOVA). Process context only.
 */
void aipu_mm_v3_dbg_dump_tcb_chain(struct aipu_memory_manager *mm, u64 head_pa,
				   u64 ltsk_pa, const char *tag)
{
#if V3_IOVA_DEBUG
	const u32 max_walk = 8192;
	u32 i, dangle_tcbs = 0, dangle_fields = 0;
	u64 pa = head_pa;

	if (!mm || !mm->use_v3_custom_iova || !head_pa)
		return;

	for (i = 0; i < max_walk && pa; i++) {
		struct aipu_tcb *tcb = aipu_mm_v3_dbg_iova_to_va(mm, pa);
		u64 next;

		if (!tcb) {
			v3dbg(mm, "CHAINWALK %-4s #%u pa=0x%llx <unmapped TCB - stop>\n",
			      tag, i, pa);
			break;
		}

		if (!IS_INIT_TCB(tcb->flag)) {
			char b[224];
			int n = 0;

#define CHK(f) do { \
	u32 _v = tcb->task.f; \
	if (_v && !aipu_mm_v3_dbg_iova_phys(mm, _v)) { \
		n += scnprintf(b + n, sizeof(b) - n, " " #f "=0x%x", _v); \
		dangle_fields++; \
	} \
} while (0)
			CHK(sp); CHK(pp); CHK(dp); CHK(cp);
			CHK(fmdp); CHK(tap); CHK(dap); CHK(pap);
			CHK(idp); CHK(pprint); CHK(pprofiler);
			CHK(global_param);
#undef CHK
			if (n) {
				dangle_tcbs++;
				v3dbg(mm, "CHAINWALK %-4s #%u pa=0x%llx flag=0x%x DANGLING:%s\n",
				      tag, i, pa, tcb->flag, b);
			}
		}

		if (pa == ltsk_pa || IS_GRID_END(tcb->flag))
			break;

		next = tcb->next;
		if (!next || next == pa)
			break;
		pa = next;
	}

	v3dbg(mm, "CHAINWALK %-4s walked %u tcbs, %u dangling tcbs, %u dangling fields\n",
	      tag, i, dangle_tcbs, dangle_fields);
#endif
}

#if V3_IOVA_DEBUG
/*
 * DEBUG: scan @max_bytes of the buffer at @iova for 32-bit words that look like
 * an ASID-0 IOVA [0x1000, 3GB) but are currently unmapped (m0) -- i.e. a stale
 * absolute pointer baked into the content. Page VA is fetched once per page.
 */
static void aipu_mm_v3_dbg_scan_region(struct aipu_memory_manager *mm, u64 iova,
				       u32 max_bytes, const char *tag,
				       const char *field)
{
	u64 base = iova & ~((u64)PAGE_SIZE - 1);
	const u32 max_hits = 8;
	u32 poff, hits = 0;

	for (poff = 0; poff < max_bytes && hits < max_hits; poff += PAGE_SIZE) {
		u8 *pg = aipu_mm_v3_dbg_iova_to_va(mm, base + poff);
		u32 w;

		if (!pg)
			break;	/* past end of this buffer's mapping */
		for (w = 0; w + 4 <= PAGE_SIZE && hits < max_hits; w += 4) {
			u32 v = *(u32 *)(pg + w);

			/*
			 * Only consider values in the high IOVA band where the
			 * top-down allocator actually places buffers; this
			 * filters out small constants/strides/packed fields that
			 * merely look like a low IOVA.
			 */
			if (v >= 0x90000000 && v < 0xC0000000 &&
			    !aipu_mm_v3_dbg_iova_phys(mm, v)) {
				v3dbg(mm, "PTRSCAN %-4s %s base=0x%llx +0x%x = 0x%x (m0 dangling)\n",
				      tag, field, base, poff + w, v);
				hits++;
			}
		}
	}
}
#endif

void aipu_mm_v3_dbg_scan_task_ptrs(struct aipu_memory_manager *mm, u64 ftsk_pa,
				   const char *tag)
{
#if V3_IOVA_DEBUG
	const u32 win = 0x80000;	/* up to 512KB scan window per buffer */
	struct aipu_tcb *tcb;

	if (!mm || !mm->use_v3_custom_iova || !ftsk_pa)
		return;

	tcb = aipu_mm_v3_dbg_iova_to_va(mm, ftsk_pa);
	if (!tcb || IS_INIT_TCB(tcb->flag))
		return;

	if (tcb->task.cp)
		aipu_mm_v3_dbg_scan_region(mm, tcb->task.cp, win, tag, "cp");
	if (tcb->task.pp)
		aipu_mm_v3_dbg_scan_region(mm, tcb->task.pp, win, tag, "pp");
	if (tcb->task.dp)
		aipu_mm_v3_dbg_scan_region(mm, tcb->task.dp, win, tag, "dp");
#endif
}

/*
 * DEBUG: scan EVERY mapped buffer's content for a 32-bit word that looks like a
 * PAGE-ALIGNED ASID-0 IOVA in the band the top-down allocator uses
 * [0x90000000, 3GB) but is NOT currently mapped -- i.e. a dangling/stale base
 * pointer the NPU will fault on. Page-agnostic on purpose: the faulting IOVA
 * moves between sessions (0xba780000 vs 0xb9fb2000) because it tracks the live
 * layout, so a hardcoded target page misses it. @page_base is ignored (ABI).
 *
 * Two filters keep the signal high:
 *   - page-aligned ((v & 0xfff)==0): a real base pointer is page-aligned;
 *     quantized-weight data that merely lands in the band is not, so this drops
 *     the flood of false positives seen with the unfiltered scan.
 *   - per-buffer hit cap: a single noisy buffer can't exhaust the global budget
 *     before the scan reaches cp/pp/rodata/the big reuse buffer.
 *
 * Reads via the cached linear mapping (page_address) so a multi-hundred-MB sweep
 * is fast; the owning buffer IOVA/dtype lets us correlate to its ALLOC_IOVA.
 * Process context (takes buffer/phy mutex; iova_to_phys takes no extra lock).
 */
void aipu_mm_v3_dbg_scan_all_for(struct aipu_memory_manager *mm, u32 page_base,
				 const char *tag)
{
#if V3_IOVA_DEBUG
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;
	const u32 max_hits = 128;
	const u32 max_hits_per_buf = 8;
	u32 hits = 0;
	u32 exact_hits = 0;

	(void)page_base;

	if (!mm || !mm->use_v3_custom_iova)
		return;

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		u32 buf_hits = 0;

		mutex_lock(&buffer->phy_mutex);
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			u32 pi;

			for (pi = 0; pi < block->page_count &&
			     hits < max_hits && buf_hits < max_hits_per_buf;
			     pi++) {
				u32 *p = page_address(block->pages[pi]);
				u32 w;

				if (!p)
					continue;
				for (w = 0; w < PAGE_SIZE / 4; w++) {
					u32 v = p[w];

					/*
					 * FREED-BASE scan: the NPU reaches the fault
					 * addr via (base + runtime offset). The base
					 * is one of the three freed load-time
					 * intermediates' iova_start (yolox 150MB /
					 * arcface 49MB / scrfd 36MB), stored as a
					 * 32-bit word inside live content. A hit pins
					 * the exact carrier buffer + offset that bakes
					 * the base the NPU dereferences into the freed
					 * hole. 0xba780244 = arcface_base(0xb781f000)
					 * + 0x2961244 -- inside arcface's freed span.
					 */
					if (exact_hits < 256 &&
					    (v == 0xb1265000u ||
					     v == 0xb781f000u ||
					     v == 0xb85b7000u)) {
						v3dbg(mm, "FREEBASE   %s buf_iova=0x%llx blk=0x%llx dtype=%s(%u) off=0x%x val=0x%x\n",
						      tag, buffer->iova_start,
						      block->iova_start,
						      aipu_mm_v3_dtype_str(buffer->data_type),
						      buffer->data_type,
						      (pi << PAGE_SHIFT) + (w << 2), v);
						exact_hits++;
					}

					/*
					 * Page-aligned, in the allocator's high
					 * band, and currently unmapped: the
					 * dangling base-pointer signature.
					 */
					if (v & 0xfff)
						continue;
					if (v < 0x90000000 || v >= 0xC0000000)
						continue;
					if (aipu_mm_v3_dbg_iova_phys(mm, v))
						continue;

					v3dbg(mm, "DANGLEPTR %s buf_iova=0x%llx blk=0x%llx dtype=%s(%u) off=0x%x val=0x%x (m0 unmapped)\n",
					      tag, buffer->iova_start,
					      block->iova_start,
					      aipu_mm_v3_dtype_str(buffer->data_type),
					      buffer->data_type,
					      (pi << PAGE_SHIFT) + (w << 2), v);
					if (++hits >= max_hits)
						break;
					if (++buf_hits >= max_hits_per_buf)
						break;
				}
			}
			if (hits >= max_hits || buf_hits >= max_hits_per_buf)
				break;
		}
		mutex_unlock(&buffer->phy_mutex);
		if (hits >= max_hits)
			break;
	}
	mutex_unlock(&mm->buffer_mutex);

	v3dbg(mm, "DANGLEPTR %s done: %u dangling pointer(s) found, %u freed-base hit(s) (0xb1265000/0xb781f000/0xb85b7000)\n",
	      tag, hits, exact_hits);
#endif
}

/*
 * DEBUG: raw u32 dump of a TCB header. The init/grid/group fields live in a
 * union (asids / gm_ctrl+gm_rgnx_addr / seg-mmu smmu_conf) so a typed dump can
 * misread them; the raw words let us spot a 0xba780-derived seg-mmu/GM base
 * regardless of the union variant. 8 words/line, @n_words capped at 32.
 */
void aipu_mm_v3_dbg_dump_tcb_raw(struct aipu_memory_manager *mm, u64 tcb_pa,
				 u32 n_words, const char *tag)
{
#if V3_IOVA_DEBUG
	u32 *w;
	u32 i;

	if (!mm || !mm->use_v3_custom_iova || !tcb_pa)
		return;

	w = aipu_mm_v3_dbg_iova_to_va(mm, tcb_pa);
	if (!w) {
		v3dbg(mm, "TCBRAW %-4s pa=0x%llx <no kernel VA>\n", tag, tcb_pa);
		return;
	}

	if (n_words > 32)
		n_words = 32;

	for (i = 0; i < n_words; i += 8)
		v3dbg(mm, "TCBRAW %-4s pa=0x%llx +0x%02x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		      tag, tcb_pa, i << 2,
		      w[i + 0], w[i + 1], w[i + 2], w[i + 3],
		      w[i + 4], w[i + 5], w[i + 6], w[i + 7]);
#endif
}

/* Forward declarations for V3 custom IOVA support */
static int aipu_mm_v3_reserve_never_map_zones(struct aipu_memory_manager *mm);

#define EXIT_ENCODE_BUF_SIZE (1024)
#define IOVA_FREE  false
#define IOVA_ALLOC true

enum iommu_dma_cookie_type {
	IOMMU_DMA_IOVA_COOKIE,
	IOMMU_DMA_MSI_COOKIE,
};

#if KERNEL_VERSION(6, 15, 0) > LINUX_VERSION_CODE
struct iommu_dma_cookie {
	enum iommu_dma_cookie_type	type;
	union {
		struct iova_domain	iovad;
		dma_addr_t		msi_iova;
	};
	struct list_head		msi_page_list;
	struct iommu_domain		*fq_domain;
};
#else
/*
 * Partial redefinition of the kernel-private struct iommu_dma_cookie
 * (defined in drivers/iommu/dma-iommu.c). Only the iovad field is accessed
 * by this driver; it must be at offset 0 to match the kernel's layout.
 */
struct iommu_dma_cookie {
	struct iova_domain      iovad;
};
#endif

struct exporter_dma_buf {
	void *vaddr;
	dma_addr_t paddr;
	size_t size;
	struct dma_buf *dmabuf;
	struct list_head list;
	struct page **pages;
	int fd;
};
static struct aipu_mem_region *aipu_mm_find_region_no_lock(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str);
static struct aipu_tcb_buf *aipu_mm_find_tbuf_in_region_no_lock(struct aipu_memory_manager *mm,
								struct aipu_mem_region *reg,
								u64 iova);
static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str);
static struct aipu_iova_buffer *aipu_mm_v3_find_buffer_by_iova(struct aipu_memory_manager *mm,
							      u64 iova);
static struct aipu_tcb_buf *aipu_mm_v3_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
							    u64 pa);
static int init_cache_ops(struct aipu_memory_manager *mm);
static void aipu_debug_print_buffers(struct aipu_memory_manager *mm);
static int aipu_free_all_dma_iova_phy(struct aipu_memory_manager *mm, struct file *filp);
static struct aipu_iova_buffer *aipu_get_iova_buffer(struct aipu_memory_manager *mm, u64 iova, char* str);
static struct device *aipu_mm_create_child_dev(struct device *dev, u32 idx)
{
	struct device *child = NULL;

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
	if (idx) {
		dev_err(dev, "multiple mem regions are not supported (idx %d)\n", idx);
		return NULL;
	}
	return dev;
#endif

	child = devm_kzalloc(dev, sizeof(*child), GFP_KERNEL);
	if (!child)
		return NULL;

	device_initialize(child);
	dev_set_name(child, "%s:%s%u", dev_name(dev), "reserved_", idx);
	child->parent = dev;
	child->coherent_dma_mask = dev->coherent_dma_mask;
	child->dma_mask = dev->dma_mask;
	child->dma_parms = devm_kzalloc(dev, sizeof(*child->dma_parms),
					GFP_KERNEL);
	child->bus = dev->bus;
	if (!child->dma_parms)
		goto err;

#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
	child->dma_pfn_offset = dev->dma_pfn_offset;
#else
	child->dma_range_map = dev->dma_range_map;
#endif

	return child;

err:
	put_device(child);
	return NULL;
}

int aipu_mm_hold_tcb_buf_alloc(struct aipu_memory_manager *mm, struct aipu_job *kern_job)
{
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_tcb_buf *prev_tbuf = NULL;
	struct aipu_tcb_buf *curr_tbuf = NULL;
	struct aipu_mem_region *reg = NULL;
	struct aipu_buf_request buf;
	unsigned long flags;
	int ret = 0;
	struct aipu_job_desc *desc = &kern_job->desc;
	struct aipu_tcb *tcb = NULL;
	struct aipu_tcb *prev = NULL;
	char *va = NULL;
	u64 encode_offset = sizeof(struct aipu_tcb) * 2;
	int malloc_flag = 0;
	bool find_htbuf = false;
	unsigned char exit_encode[] = {
		0x00, 0x80, 0x4f, 0x01,
		0xff, 0x7f, 0x0c, 0x40,
		0xff, 0x7f, 0x0c, 0x40,
		0xff, 0x7f, 0x0c, 0x40
	};

	// find idle hold tcb buffer
	spin_lock_irqsave(&mm->shlock, flags);
	list_for_each_entry(htbuf, &htbuf_head->node, node) {
		if (htbuf->status == AIPU_MEM_HOLD_TYPE_IDLE &&
		    htbuf->index != htbuf_head->hold_index) {
			htbuf->status = AIPU_MEM_HOLD_TYPE_LINKING;
			find_htbuf = true;
			dev_dbg(mm->dev, "found hold index %d tcb  0x%llx from list.\n",
				htbuf->index, htbuf->head);
			break;
		}
	}
	spin_unlock_irqrestore(&mm->shlock, flags);

	// malloc new hold tcb buffer
	if (!find_htbuf) {
		htbuf = kmem_cache_zalloc(mm->hold_tbuf_cache, GFP_KERNEL);
		if (!htbuf) {
			ret = -ENOMEM;
			dev_err(mm->dev, "alloc cache hold tcb buffer failed");
			goto CACHE_FAIL;
		}

		INIT_LIST_HEAD(&htbuf->node);
		memset(&buf, 0, sizeof(buf));
		buf.bytes = sizeof(struct aipu_tcb) + EXIT_ENCODE_BUF_SIZE;
		buf.align_in_page = 1;
		buf.region = AIPU_BUF_REGION_DEFAULT;
		buf.data_type = AIPU_MM_DATA_TYPE_TCB;
		buf.asid = AIPU_BUF_ASID_0;
		buf.alloc_mode = AIPU_DMA_BUF_MALLOC_DEFAULT;
		ret = aipu_mm_alloc(mm, &buf, NULL);
		if (ret) {
			dev_err(mm->dev, "alloc hold tcb buffer failed");
			ret = -ENOMEM;
			goto MALLOC_TCB_FAIL;
		}

		malloc_flag = 1;
		spin_lock_irqsave(&mm->shlock, flags);
		htbuf->desc = buf.desc;
		htbuf->head = buf.desc.pa;
		htbuf->next_head = 0;
		htbuf->prev_tail = 0;
		htbuf->prev_head = 0;
		htbuf->prev_hold_tcb = 0;
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINKING;
		htbuf->index = htbuf_head->nums;
		htbuf_head->nums++;
		list_add(&htbuf->node, &htbuf_head->node);
		spin_unlock_irqrestore(&mm->shlock, flags);

		va = aipu_mm_get_va(mm, htbuf->head + encode_offset);
		if (!va) {
			dev_err(mm->dev, "get encode va failed.");
			ret = -EFAULT;
			goto VA_FAIL;
		}

		memcpy(va, exit_encode, 16);
		dev_dbg(mm->dev, "malloc hold index %d\n", htbuf->index);
	}

	tcb = aipu_mm_get_tcb(mm, htbuf->head);
	if (!tcb) {
		dev_err(mm->dev, "get hold tcb buffer failed");
		ret = -EFAULT;
		goto VA_FAIL;
	}

	prev = aipu_mm_get_tcb(mm, desc->last_task_tcb_pa);
	if (!prev) {
		dev_err(mm->dev, "get prev tcb buffer failed for pa 0x%llx", desc->last_task_tcb_pa);
		ret = -EFAULT;
		goto VA_FAIL;
	}

	// Modify hold tcb data
	spin_lock_irqsave(&mm->shlock, flags);
	memcpy(tcb, prev, sizeof(struct aipu_tcb));
	tcb->flag = TCB_FLAG_TASK_TYPE_TASK;
	tcb->flag |= TCB_FLAG_END_TYPE_GROUP_END;
	tcb->flag |= TCB_FLAG_END_TYPE_GRID_END;
	tcb->next = 0;
	tcb->task.interrupt_en &= ~INTERRUPT_TEC;
	tcb->task.spc = lower_32_bits(htbuf->head + encode_offset -
					aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0));
	tcb->task.tcbp = lower_32_bits(htbuf->head - aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0));
	spin_unlock_irqrestore(&mm->shlock, flags);

	/* Find TCB buffer descriptors for hold TCB and task TCB */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		/* V3 Custom IOVA: search in the global V3 TCB buffer list */
		spin_lock_irqsave(&mm->slock, flags);
		curr_tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, htbuf->head);
		if (!curr_tbuf) {
			dev_err(mm->dev, "V3: no TCB buffer found for hold iova 0x%llx",
				htbuf->head);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		prev_tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, desc->last_task_tcb_pa);
		if (!prev_tbuf) {
			dev_err(mm->dev, "V3: no TCB buffer found for task iova 0x%llx",
				desc->last_task_tcb_pa);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		spin_unlock_irqrestore(&mm->slock, flags);
	} else {
		reg = aipu_mm_find_region(mm, htbuf->head, "find_tcb hold");
		if (!reg) {
			dev_err(mm->dev, "Can't found region at iova 0x%llx", htbuf->head);
			ret = -EFAULT;
			goto VA_FAIL;
		}

		spin_lock_irqsave(&mm->slock, flags);
		curr_tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, htbuf->head);
		if (!curr_tbuf) {
			dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx", htbuf->head);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		spin_unlock_irqrestore(&mm->slock, flags);

		reg = aipu_mm_find_region(mm, desc->last_task_tcb_pa, "find_tcb task");
		if (!reg) {
			dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx",
				desc->last_task_tcb_pa);
			ret = -EFAULT;
			goto VA_FAIL;
		}

		spin_lock_irqsave(&mm->slock, flags);
		prev_tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, desc->last_task_tcb_pa);
		if (!prev_tbuf) {
			dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx",
				desc->last_task_tcb_pa);
			ret = -EFAULT;
			spin_unlock_irqrestore(&mm->slock, flags);
			goto VA_FAIL;
		}
		spin_unlock_irqrestore(&mm->slock, flags);
	}

	spin_lock_irqsave(&mm->shlock, flags);
	curr_tbuf->tail = htbuf->head;
	curr_tbuf->tail_tcb = tcb;
	curr_tbuf->dep_job_id = 0;
	curr_tbuf->pinned = false;
	prev_tbuf->tail = desc->last_task_tcb_pa;
	prev_tbuf->tail_tcb = prev;
	prev->next = htbuf->head;
	htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_PREV;
	htbuf->hold_tcb = tcb;
	htbuf->prev_tail = desc->last_task_tcb_pa;
	htbuf->prev_head = desc->head_tcb_pa;
	kern_job->curr_hold_tcb = htbuf->head;
	htbuf->prev_tbuf = prev_tbuf;
	spin_unlock_irqrestore(&mm->shlock, flags);

	return ret;

VA_FAIL:
	if (malloc_flag == 1) {
		aipu_mm_free(mm, &buf.desc, NULL, true);
		htbuf_head->nums--;
		kmem_cache_free(mm->hold_tbuf_cache, htbuf);
	}
	return ret;
MALLOC_TCB_FAIL:
	kmem_cache_free(mm->hold_tbuf_cache, htbuf);
CACHE_FAIL:
	return ret;
}

static void aipu_mm_hold_tcb_buf_free(struct aipu_memory_manager *mm)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *next = NULL;
	struct aipu_buf_desc buf;

	if (!htbuf_head)
		return;

	list_for_each_entry_safe(htbuf, next, &htbuf_head->node, node) {
		dev_dbg(mm->dev, "t %d i %d s %d f %d ch 0x%llx pht 0x%llx"
			" pt 0x%llx ph 0x%llx nh 0x%llx\n", htbuf_head->nums,
			htbuf->index,  htbuf->status, htbuf_head->hold_index,
			htbuf->head, htbuf->prev_hold_tcb, htbuf->prev_tail,
			htbuf->prev_head, htbuf->next_head);
		memset(&buf, 0, sizeof(buf));
		buf.pa = htbuf->head;
		buf.region = AIPU_BUF_REGION_DEFAULT;
		buf.asid = AIPU_BUF_ASID_0;
		aipu_mm_free(mm, &buf, NULL, true);
		list_del(&htbuf->node);
		kmem_cache_free(mm->hold_tbuf_cache, htbuf);
		htbuf_head->nums--;
	}
	htbuf_head->hold_index = -1;
	htbuf_head->hold_tcb = NULL;
}

struct aipu_hold_tcb_buf *aipu_mm_get_hold_htbuf(struct aipu_memory_manager *mm, u64 hold_tcb_pa)
{
	struct aipu_hold_tcb_buf *htbuf_head;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *ret = NULL;
	unsigned long flags = 0;

	if (!mm || !mm->hold_tcb_head)
		return NULL;

	htbuf_head = mm->hold_tcb_head;
	spin_lock_irqsave(&mm->shlock, flags);
	list_for_each_entry(htbuf, &htbuf_head->node, node) {
		if (htbuf->head == hold_tcb_pa) {
			ret = htbuf;
			break;
		}
	}
	spin_unlock_irqrestore(&mm->shlock, flags);

	return ret;
}

void aipu_mm_set_final_htbuf_index(struct aipu_memory_manager *mm, int index)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	unsigned long flags = 0;

	spin_lock_irqsave(&mm->shlock, flags);
	htbuf_head->hold_index = index;
	spin_unlock_irqrestore(&mm->shlock, flags);
}

static struct aipu_tcb_buf *create_tcb_buf(struct aipu_memory_manager *mm,
					   struct aipu_mem_region *reg)
{
	struct aipu_tcb_buf *tbuf = NULL;

	tbuf = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
	if (tbuf) {
		INIT_LIST_HEAD(&tbuf->node);
		tbuf->reg = reg;
	}
	return tbuf;
}

static void destroy_tcb_buf(struct aipu_memory_manager *mm, struct aipu_tcb_buf *tbuf)
{
	unsigned long flags = 0;

	if (tbuf) {
		spin_lock_irqsave(&mm->slock, flags);
		list_del(&tbuf->node);
		spin_unlock_irqrestore(&mm->slock, flags);
		kmem_cache_free(mm->tbuf_cache, tbuf);
	}
}

static struct aipu_mem_region *aipu_mm_find_region_no_lock(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_iova_buffer *buffer = NULL;
	u32 asid;

	/* First search traditional regions */
	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;
		if (reg && iova >= reg->base_iova && (iova - reg->base_iova < reg->bytes))
			return reg;
	}

	/* V3 Custom IOVA: search in IOVA buffers */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			u64 iova_addr = aipu_mm_v3_pa_to_iova(mm, iova, asid);
			if (!iova_addr)
				continue;
			buffer = aipu_mm_v3_find_buffer_by_iova(mm, iova_addr);
			if (buffer)
				return NULL; /* V3 buffer found but no traditional region */
		}
	}

	dev_err(mm->dev, "[%s] region not found: invalid address 0x%llx",
		log_str ? log_str : "find_reg", iova);
	return NULL;
}

static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm,
						   u64 iova, char *log_str)
{
	struct aipu_mem_region *reg = NULL;

	mutex_lock(&mm->lock);
	reg = aipu_mm_find_region_no_lock(mm, iova, log_str);
	mutex_unlock(&mm->lock);
	return reg;
}

static struct aipu_tcb_buf *aipu_mm_find_tbuf_in_region_no_lock(struct aipu_memory_manager *mm,
								struct aipu_mem_region *reg,
								u64 iova)
{
	struct aipu_tcb_buf *curr = NULL;

	list_for_each_entry(curr, &reg->tcb_buf_head->node, node) {
		if (iova >= curr->head && iova <= curr->tail)
			return curr;
	}

	return NULL;
}

static struct aipu_tcb_buf *aipu_mm_find_tcb_buf_no_lock(struct aipu_memory_manager *mm, u64 iova,
							 char *log_str)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;

	if (!mm)
		return NULL;

	/* V3 custom IOVA: search in the global V3 TCB buffer list */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, iova);
		if (log_str && !tbuf)
			dev_dbg(mm->dev, "[%s] V3: no TCB buffer at iova 0x%llx", log_str, iova);
		return tbuf;
	}

	reg = aipu_mm_find_region_no_lock(mm, iova, "find_tcb");
	if (!reg)
		return NULL;

	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, iova);
	if (log_str && !tbuf)
		dev_dbg(mm->dev, "[%s] no TCB buffer is found at iova 0x%llx", log_str, iova);

	return tbuf;
}

static unsigned long pa_to_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
				     u64 pa)
{
	return (pa - reg->base_iova) >> PAGE_SHIFT;
}

static int aipu_mm_init_pages(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	if (!mm || !reg)
		return -EINVAL;

	reg->count = reg->bytes >> PAGE_SHIFT;
	reg->bitmap = devm_kzalloc(reg->dev,
				   BITS_TO_LONGS(reg->count) * sizeof(long), GFP_KERNEL);
	if (!reg->bitmap)
		return -ENOMEM;

	reg->pages = vzalloc(reg->count * sizeof(struct aipu_virt_page *));
	if (!reg->pages)
		return -ENOMEM;

	reg->base_pfn = PFN_DOWN(reg->base_iova);

	return 0;
}

static void aipu_mm_destroy_region(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_tcb_buf *next = NULL;

	if (!mm || !reg)
		return;

	/* release allocation to system */
	if (reg->base_va) {
		dma_free_attrs(reg->dev, reg->bytes, reg->base_va, reg->base_pa, reg->attrs);
		reg->base_va = NULL;
		reg->base_iova = 0;
	}

	if (reg->reserved) {
		of_reserved_mem_device_release(reg->dev);
		reg->base_pa = 0;
	}

	if (reg->pages) {
		vfree(reg->pages);
		reg->pages = NULL;
		reg->count = 0;
	}

	if (reg->bitmap) {
		devm_kfree(reg->dev, reg->bitmap);
		reg->bitmap = NULL;
	}

	if (reg->tcb_buf_head) {
		list_for_each_entry_safe(tbuf, next, &reg->tcb_buf_head->node, node)
			destroy_tcb_buf(mm, tbuf);

		destroy_tcb_buf(mm, reg->tcb_buf_head);
		reg->tcb_buf_head = NULL;
	}

	reg->bytes = 0;
	kmem_cache_free(mm->reg_cache, reg);
}

static struct aipu_mem_region *aipu_mm_create_region(struct aipu_memory_manager *mm,
						     struct aipu_mem_region_obj *obj,
						     enum aipu_mem_region_type type,
						     u64 size, u64 offset, u32 idx,
						     bool reserved)
{
	int ret = 0;
	void *va = NULL;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !obj)
		return NULL;

	reg = kmem_cache_zalloc(mm->reg_cache, GFP_KERNEL);
	if (!reg)
		return NULL;

	reg->type = type;
	reg->bytes = size;
	reg->host_aipu_offset = offset;
	reg->reserved = reserved;
	reg->obj = obj;

	/* we use child dev to support multiple reserved regions */
	if (reserved && !mm->has_iommu)
		reg->dev = aipu_mm_create_child_dev(mm->dev, idx);
	else
		reg->dev = mm->dev;

	if (!reg->dev)
		goto err;

	/* only head of the list is created; for v3; */
	reg->tcb_buf_head = create_tcb_buf(mm, reg);
	if (!reg->tcb_buf_head)
		goto err;

	/* create a list head, not a real region */
	if (!size)
		return reg;

	if (reserved && !mm->has_iommu) {
		ret = of_reserved_mem_device_init_by_idx(reg->dev, mm->dev->of_node, idx);
		if (ret) {
			dev_err(reg->dev, "init reserved mem failed: idx %d (%d)\n",
				idx, ret);
			goto err;
		}
	}

	if (mm->has_iommu) {
		ret = dma_set_mask_and_coherent(mm->dev, DMA_BIT_MASK(mm->dma_mask));
		if (ret) {
			dev_err(reg->dev, "DMA set coherent mask failed: idx %d (%d)!\n", idx, ret);
			goto err;
		}
	}

	va = dma_alloc_attrs(reg->dev, reg->bytes, &reg->base_pa, GFP_KERNEL, reg->attrs);
	if (!va) {
		dev_err(reg->dev, "dma_alloc_attrs failed: idx %d (bytes: 0x%llx, attrs %ld)\n",
			idx, reg->bytes, reg->attrs);
		goto err;
	}
	reg->base_iova = reg->base_pa - reg->host_aipu_offset;
	reg->base_va = va;

	/* reserve memory region use page structs to maintain the allocations in this reg */
	if (reserved) {
		ret = aipu_mm_init_pages(mm, reg);
		if (ret)
			goto err;

		dev_info(reg->dev, "init %s region done: %s [0x%llx, 0x%llx]\n",
			 type == AIPU_MEM_REGION_TYPE_MEMORY ? "MEMORY" :
			 (type == AIPU_MEM_REGION_TYPE_SRAM ? "SRAM" :
			  (type == AIPU_MEM_REGION_TYPE_DTCM ? "DTCM" : "GM")),
			 mm->has_iommu ? "iova" : "pa",
			 reg->base_iova, reg->base_iova + reg->bytes - 1);
	}

	/* works for non-reserved regions only */
	reg->locked = true;
	return reg;

err:
	aipu_mm_destroy_region(mm, reg);
	return NULL;
}

static struct aipu_mem_region_obj *aipu_mm_create_region_object(struct aipu_memory_manager *mm,
								enum aipu_mem_region_type type,
								u64 size, u64 offset, u32 idx,
								struct file *filp, bool reserved)
{
	struct aipu_mem_region_obj *obj = NULL;

	obj = kmem_cache_zalloc(mm->obj_cache, GFP_KERNEL);
	if (!obj)
		return NULL;

	obj->reg = aipu_mm_create_region(mm, obj, type, size, offset, idx, reserved);
	if (!obj->reg) {
		kmem_cache_free(mm->obj_cache, obj);
		return NULL;
	}

	obj->reg->filp = filp;
	INIT_LIST_HEAD(&obj->list);

	return obj;
}

static void aipu_mm_destroy_region_object(struct aipu_memory_manager *mm,
					  struct aipu_mem_region_obj *obj)
{
	if (obj) {
		aipu_mm_destroy_region(mm, obj->reg);
		obj->reg = NULL;
		list_del(&obj->list);
		kmem_cache_free(mm->obj_cache, obj);
	}
}

static int init_region_list(struct aipu_memory_manager *mm, struct aipu_mem_region_list *list)
{
	struct aipu_mem_region_obj *obj = NULL;

	obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY, 0, 0, 0, NULL, false);
	if (!obj)
		return -ENOMEM;

	list->head = obj;
	list->cnt = 0;
	list->valid_cnt = 0;
	list->base = U64_MAX;
	list->range = 0;
	return 0;
}

static int get_free_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
			      struct aipu_buf_request *buf_req)
{
	unsigned long alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	unsigned long align_order = order_base_2(buf_req->align_in_page);
	unsigned long mask = (1UL << align_order) - 1;
	unsigned long offset = reg->base_pfn & ((1UL << align_order) - 1);

	return bitmap_find_next_zero_area_off(reg->bitmap, reg->count, 0, alloc_nr, mask, offset);
}

static int aipu_mm_alloc_in_region_no_lock(struct aipu_memory_manager *mm,
					   struct aipu_buf_request *buf_req,
					   struct aipu_mem_region *reg, struct file *filp,
					   struct aipu_tcb_buf **tbuf_alloc)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct aipu_tcb_buf *tbuf = NULL;
	u64 dev_pa = 0;
	u32 dev_size = 0;

	/* this allocator is called in both reserved & smmu cases */

	/* filp == NULL means GM initialization or exit TCB allocation */
	if (!mm || !buf_req || !reg || reg->invalid || !tbuf_alloc)
		return -EINVAL;

	if (buf_req->data_type == AIPU_MM_DATA_TYPE_TCB) {
		tbuf = create_tcb_buf(mm, reg);
		if (!tbuf)
			return -ENOMEM;
	}

	alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	dev_size = alloc_nr * PAGE_SIZE;
	if (reg->reserved) {
		bitmap_no = get_free_bitmap_no(mm, reg, buf_req);
		if (bitmap_no >= reg->count) {
			dev_dbg(reg->dev, "alloc in region failed: no free buffer");
			goto fail;
		}

		if (!reg->pages[bitmap_no]) {
			reg->pages[bitmap_no] =
				devm_kzalloc(reg->dev, sizeof(struct aipu_virt_page), GFP_KERNEL);
			if (!reg->pages[bitmap_no])
				goto fail;
		}

		/* success */
		reg->pages[bitmap_no]->contiguous_alloc_len = alloc_nr;
		reg->pages[bitmap_no]->filp = filp;
		reg->pages[bitmap_no]->tid = task_pid_nr(current);
		reg->pages[bitmap_no]->locked = true;
		reg->pages[bitmap_no]->tcb = NULL;
		bitmap_set(reg->bitmap, bitmap_no, alloc_nr);

		dev_pa = reg->base_iova + (bitmap_no << PAGE_SHIFT);
	} else {
		dev_pa = reg->base_iova;
	}

	if (tbuf) {
		if (reg->reserved) {
			reg->pages[bitmap_no]->tcb = tbuf;
			tbuf->pfn = bitmap_no;
		}

		/**
		 * when head/tail is updated in job manager, all the following fields
		 * should be updated together.
		 *
		 * NOTE: all the nodes of the TCB chain scheduled in a job should be within
		 * this buf!
		 */
		tbuf->head = dev_pa;
		tbuf->tail = tbuf->head + dev_size - sizeof(struct aipu_tcb);
		tbuf->tail_tcb = NULL;
		tbuf->dep_job_id = 0;
		tbuf->pinned = false;
		*tbuf_alloc = tbuf;
	}

	buf_req->desc.dev_offset = dev_pa;
	buf_req->desc.pa = dev_pa;
	buf_req->desc.bytes = dev_size;
	buf_req->desc.asid = buf_req->asid;
	buf_req->desc.region = reg->type;

	dev_dbg(reg->dev, "allocation in region done: asid %d iova 0x%llx, bytes 0x%llx, type %d\n",
		buf_req->asid, buf_req->desc.pa, buf_req->desc.bytes, buf_req->data_type);
	return 0;

fail:
	if (tbuf)
		destroy_tcb_buf(mm, tbuf);
	return -ENOMEM;
}

static int aipu_mm_free_in_region(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf,
				  struct aipu_mem_region *reg, bool unlock)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct aipu_virt_page *page = NULL;
	struct aipu_tcb_buf *tbuf = NULL;

	if (!mm || !buf || !reg || reg->invalid)
		return -EINVAL;

	/* this __free__ function is only called for reserved cases */
	bitmap_no = pa_to_bitmap_no(mm, reg, buf->pa);
	if (bitmap_no >= reg->count) {
		dev_err(reg->dev,
			"free in region failed: no such an allocated buffer: pa 0x%llx",
			buf->pa);
		return -EINVAL;
	}

	page = reg->pages[bitmap_no];
	if (!page) {
		dev_err(reg->dev, "free in region failed: null page at bitmap_no %lu",
			bitmap_no);
		return -EINVAL;
	}

	alloc_nr = page->contiguous_alloc_len;
	if (!alloc_nr) {
		dev_err(reg->dev, "free in region failed: zero alloc_nr is invalid, 0x%llx\n",
			buf->pa);
		return -EINVAL;
	}

	/* do not update the __locked__ flag if unlock == false */
	if (unlock)
		page->locked = false;
	else if (page->locked)
		return DEFERRED_FREE;

	tbuf = page->tcb;

	/* do not free if this tcb is linked by a job or pinned;
	 * marked as unlocked and will be released at an appropriate point;
	 */
	if (tbuf && (tbuf->dep_job_id || tbuf->pinned)) {
		dev_info(reg->dev, "deferred free TCB: iova 0x%llx dep_job_id 0x%x ping %d\n",
			 buf->pa, tbuf->dep_job_id, tbuf->pinned);
		return DEFERRED_FREE;
	}

	/* do free */
	destroy_tcb_buf(mm, tbuf);
	bitmap_clear(reg->bitmap, bitmap_no, alloc_nr);
	memset(page, 0, sizeof(struct aipu_virt_page));

	dev_dbg(reg->dev, "free in region done: iova 0x%llx, bytes 0x%llx\n", buf->pa, buf->bytes);

	return 0;
}

static void aipu_mm_free_filp_in_region(struct aipu_memory_manager *mm,
					struct aipu_mem_region *reg, struct file *filp)
{
	unsigned long i = 0;
	unsigned long offset = 0;
	struct aipu_tcb_buf *tbuf = NULL;

	if (!mm || !reg || !reg->bitmap || !filp)
		return;

	mutex_lock(&mm->lock);
	while ((i = find_next_bit(reg->bitmap, reg->count, offset)) != reg->count) {
		offset = i + reg->pages[i]->contiguous_alloc_len;
		if (reg->pages[i] && reg->pages[i]->filp == filp) {
			reg->pages[i]->locked = false;
			tbuf = reg->pages[i]->tcb;
			if (tbuf && (tbuf->dep_job_id || tbuf->pinned))
				continue;

			/* do free */
			bitmap_clear(reg->bitmap, i, reg->pages[i]->contiguous_alloc_len);
			memset(reg->pages[i], 0, sizeof(struct aipu_virt_page));
			destroy_tcb_buf(mm, tbuf);
		}
	}
	mutex_unlock(&mm->lock);
}

static ssize_t aipu_gm_policy_sysfs_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	struct aipu_memory_manager *mm = &partition->priv->mm;

	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		return snprintf(buf, 128, "[%d] AIPU GM is shared by tasks of all QoS level.\n",
				AIPU_GM_POLICY_SHARED);
	} else if (mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED) {
		return snprintf(buf, 128,
				"[%d] GM is divided half-by-half for QoS slow & fast tasks.\n",
				AIPU_GM_POLICY_HALF_DIVIDED);
	}

	return snprintf(buf, 128, "[%d] AIPU has no GM (or GM is disabled).\n",
			AIPU_GM_POLICY_NONE);
}

static ssize_t aipu_gm_policy_sysfs_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	struct aipu_memory_manager *mm = &partition->priv->mm;

	mutex_lock(&mm->lock);
	if ((strncmp(buf, "0", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_NONE;
	else if ((strncmp(buf, "1", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_SHARED;
	else if ((strncmp(buf, "2", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_HALF_DIVIDED;
	else
		dev_err(mm->dev, "[sysfs] invalid GM policy: gm_policy should be 0/1/2");
	mutex_unlock(&mm->lock);

	return count;
}

static void add_region_list(struct aipu_memory_manager *mm, int asid,
			    struct aipu_mem_region *reg)
{
	struct aipu_mem_region_obj *obj = kmem_cache_zalloc(mm->obj_cache, GFP_KERNEL);

	obj->reg = reg;
	INIT_LIST_HEAD(&obj->list);

	list_add(&obj->list, &mm->ase[asid].head->list);
	mm->ase[asid].cnt++;
}

static int aipu_mm_reserved_iova_for_never_map(struct aipu_memory_manager *mm, bool flag)
{
	struct iommu_domain *iommu_domain = NULL;
	struct iommu_dma_cookie *cookie = NULL;
	struct iova_domain *iovad = NULL;
	u64 bus_dma_limit;
	unsigned long lo, hi;
	struct iova *iova;
	unsigned long iova_start;
	unsigned long  iova_len = SZ_1G;

	if (!mm)
		return -EINVAL;

	iommu_domain = mm->iommu_domain;
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3_2) {
		iova_len = SZ_512M;

		iovad = &mm->iova_domain;
	} else {
	cookie = iommu_domain->iova_cookie;
	iovad = &cookie->iovad;
	}

#if (KERNEL_VERSION(5, 5, 0) <= LINUX_VERSION_CODE)
	bus_dma_limit = mm->dev->bus_dma_limit;
#elif (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
	bus_dma_limit = mm->dev->bus_dma_mask;
#endif
	if (bus_dma_limit == 0xc0000000 || bus_dma_limit == 0xe0000000)
		bus_dma_limit = 0x100000000;

	do {
		iova_start = bus_dma_limit - iova_len;
		lo = iova_start >> PAGE_SHIFT;
		hi = (iova_start + iova_len) >> PAGE_SHIFT;
		iova = reserve_iova(iovad, lo, hi - 1);
		if (!flag)
			__free_iova(iovad, iova);
		bus_dma_limit -= 0x100000000;
	} while (bus_dma_limit > 0);

	return 0;
}

static __maybe_unused int aipu_mm_add_iova_region(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	int iova_region = 1;
	unsigned long iova_region_size = 0xc0000000;
	int asid_idx = 0;
	int region_idx = 0;

	if (!mm)
		return -EINVAL;

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3_2)
		iova_region_size = 0xe0000000;

#if (KERNEL_VERSION(5, 5, 0) <= LINUX_VERSION_CODE)
	mm->dev->bus_dma_limit = 0x800000000;
#elif (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
	mm->dev->bus_dma_mask = 0x800000000;
#endif
	mm->dma_mask = 35;

	if (aipu_mm_reserved_iova_for_never_map(mm, IOVA_ALLOC)) {
		dev_err(mm->dev, "fail to reserve iova region for never map.");
		mm->dma_mask = 32;
#if (KERNEL_VERSION(5, 5, 0) <= LINUX_VERSION_CODE)
		mm->dev->bus_dma_limit = 0xc0000000;
#elif (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
		mm->dev->bus_dma_mask = 0xc0000000;
#endif
		goto FINISH;
	}

	do {
		obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY,
						   iova_region_size, 0, asid_idx, NULL, true);
		if (!obj) {
			dev_err(mm->dev, "create iova region failed (ret = %ld)", PTR_ERR(reg));
			mm->dma_mask = 32;
#if (KERNEL_VERSION(5, 5, 0) <= LINUX_VERSION_CODE)
			mm->dev->bus_dma_limit = 0xc0000000;
#elif (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
			mm->dev->bus_dma_mask = 0xc0000000;
#endif
			break;
		}

		reg = obj->reg;

		list_add(&obj->list, &mm->mem.head->list);
		if (mm->version > AIPU_ISA_VERSION_ZHOUYI_V1)
			add_region_list(mm, asid_idx, reg);
		else
			add_region_list(mm, AIPU_BUF_ASID_0, reg);
		asid_idx++;
		region_idx++;
	} while (region_idx < iova_region);

FINISH:
	return region_idx;
}

static int aipu_mm_add_reserved_regions(struct aipu_memory_manager *mm)
{
	int ret = 0;
	int idx = 0;
	int asid_idx = 0;
	struct device_node *np = NULL;
	enum aipu_mem_region_type type;
	struct resource res;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	bool asid_set = false;
	u32 dtcm_cnt = 0;
	u64 offset = 0;
	u32 res_cnt = 0;

	do {
		u64 size = 0;

		np = of_parse_phandle(mm->dev->of_node, "memory-region", idx);
		if (!np)
			break;

		/* get type of this region */
		if (!strcmp(np->name, "memory")) {
			type = AIPU_MEM_REGION_TYPE_MEMORY;
		} else if (!strcmp(np->name, "sram")) {
			type = AIPU_MEM_REGION_TYPE_SRAM;
		} else if (!strcmp(np->name, "dtcm")) {
			type = AIPU_MEM_REGION_TYPE_DTCM;
		} else {
			dev_err(mm->dev, "invalid memory region name: %s\n", np->name);
			continue;
		}

		/* bypass mapping SoC SRAM if CPU cannot access it */
		if (type == AIPU_MEM_REGION_TYPE_SRAM && !AIPU_CONFIG_HOST_MAP_SRAM) {
			dev_info(mm->dev, "dts: bypass this sram region (cpu cannot access it)\n");
			continue;
		}

		if (type == AIPU_MEM_REGION_TYPE_DTCM && dtcm_cnt == mm->dtcm_max_cnt) {
			dev_err(mm->dev, "dts: bypass this dtcm region for the max_cnt limit\n");
			continue;
		}

		if (type == AIPU_MEM_REGION_TYPE_DTCM &&
		    mm->version == AIPU_ISA_VERSION_ZHOUYI_V1) {
			dev_err(mm->dev, "dts: bypass because v1 does not support dtcm\n");
			continue;
		}

		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			of_node_put(np);
			dev_err(mm->dev, "of_address_to_resource failed (ret = %d)", ret);
			continue;
		}

		size = res.end - res.start + 1;
		if (type == AIPU_MEM_REGION_TYPE_DTCM && size > ZHOUYI_V2_DTCM_MAX_BYTES) {
			dev_info(mm->dev,
				 "the DTCM will be clipped to the maximum configurable value\n");
			size = ZHOUYI_V2_DTCM_MAX_BYTES;
		}

		if (of_property_read_u64(np, "host-aipu-offset", &offset))
			offset = 0;

		obj = aipu_mm_create_region_object(mm, type, size, offset, idx, NULL, true);
		if (!obj) {
			dev_err(mm->dev, "create new region failed (ret = %ld)", PTR_ERR(reg));
			continue;
		}

		reg = obj->reg;
		list_add(&obj->list, &mm->mem.head->list);

		/* get asid of this region */
		if (mm->version > AIPU_ISA_VERSION_ZHOUYI_V1) {
			for (asid_idx = 0; asid_idx < ZHOUYI_ASID_COUNT; asid_idx++) {
				u32 asid = 0;

				if (!of_property_read_u32_index(np, "asid", asid_idx, &asid)) {
					if (asid >= ZHOUYI_ASID_COUNT) {
						dev_err(mm->dev, "dts: invalid asid: %u\n", asid);
						continue;
					}

					add_region_list(mm, asid, reg);
					if (asid >= mm->valid_asid_cnt)
						mm->valid_asid_cnt = asid + 1;

					asid_set = true;
				} else {
					break;
				}
			}

			if (!asid_set) {
				dev_err(mm->dev, "dts: reg asid is not set, use default asid\n");
				mm->valid_asid_cnt = 2;
				add_region_list(mm, AIPU_BUF_ASID_0, reg);
				add_region_list(mm, AIPU_BUF_ASID_1, reg);
			}
		} else {
			/* use ASID 0 for V1 */
			add_region_list(mm, AIPU_BUF_ASID_0, reg);
		}

		of_node_put(np);
		if (type == AIPU_MEM_REGION_TYPE_DTCM)
			dtcm_cnt++;
		res_cnt++;
	} while (++idx < AIPU_CONFIG_MAX_RESERVED_REGIONS);

	return res_cnt;
}

static void aipu_mm_set_asid_base(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	int asid = AIPU_BUF_ASID_0;

	for (asid = AIPU_BUF_ASID_0; asid < ZHOUYI_ASID_COUNT; asid++) {
		u64 first_base = 0;
		u64 asid_base = 0;
		u64 range = 0;
		u64 high = 0;
		u64 end  = 0;

		list_for_each_entry(obj, &mm->ase[asid].head->list, list) {
			reg = obj->reg;
			end = reg->base_iova + reg->bytes;

			if (!range) {
				high = end;
				first_base = reg->base_iova;
				asid_base = first_base;
				range = reg->bytes;
			}

			if (reg->base_iova >> 32 != first_base >> 32) {
				dev_err(mm->dev, "invalid region: [0x%llx, 0x%llx]",
					reg->base_iova, reg->base_iova + reg->bytes - 1);
				dev_err(mm->dev, "one asid should use the same hi32 base");
				reg->invalid = true;
				continue;
			}

			high = end > high ? end : high;

			if (reg->base_iova != first_base) {
				asid_base = ((first_base >> 32) << 32);
				range = high;
			}

			mm->ase[asid].valid_cnt++;
		}

		if (range) {
			mm->ase[asid].base = asid_base;
			mm->ase[asid].range = range;
			dev_info(mm->dev, "set ASID %d done: base: 0x%llx, range 0x%llx\n",
				 asid, asid_base, range);
		}
	}
}

/**
 * @aipu_init_mm() - initialize mm module during driver probe phase
 * @mm:      pointer to memory manager struct to be initialized
 * @p_dev:   pointer to the platform device struct
 * @version: AIPU ISA version
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version)
{
	int ret = 0;
	struct iommu_group *group = NULL;
	int asid = 0;

	if (!mm || !p_dev)
		return -EINVAL;

	memset(mm, 0, sizeof(*mm));
	mm->version = version;
	mm->dev = &p_dev->dev;
	mutex_init(&mm->lock);
	mm->sram_disable_head = devm_kzalloc(mm->dev, sizeof(*mm->sram_disable_head), GFP_KERNEL);
	if (!mm->sram_disable_head)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->sram_disable_head->list);
	mm->importer_bufs = devm_kzalloc(mm->dev, sizeof(*mm->importer_bufs), GFP_KERNEL);
	if (!mm->importer_bufs)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->importer_bufs->node);
	spin_lock_init(&mm->slock);
	spin_lock_init(&mm->shlock);
	mm->default_asid_base = 0;
	mm->default_asid_size = 0xC0000000;
	mm->valid_asid_cnt = 0;

	mm->obj_cache = kmem_cache_create("aipu_obj_cache", sizeof(struct aipu_mem_region_obj),
					  0, SLAB_PANIC, NULL);
	mm->reg_cache = kmem_cache_create("aipu_reg_cache", sizeof(struct aipu_mem_region),
					  0, SLAB_PANIC, NULL);
	mm->tbuf_cache = kmem_cache_create("aipu_tbuf_cache", sizeof(struct aipu_tcb_buf),
					   0, SLAB_PANIC, NULL);
	mm->hold_tbuf_cache = kmem_cache_create("aipu_hold_tbuf_cache",
						sizeof(struct aipu_hold_tcb_buf),
						0, SLAB_PANIC, NULL);
	if (!mm->obj_cache || !mm->reg_cache || !mm->tbuf_cache || !mm->hold_tbuf_cache)
		return -ENOMEM;

	for (asid = AIPU_BUF_ASID_0; asid < ZHOUYI_ASID_COUNT; asid++) {
		ret = init_region_list(mm, &mm->ase[asid]);
		if (ret)
			goto finish;
	}

	ret = init_region_list(mm, &mm->mem);
	if (ret)
		goto finish;

	/* we accept at maximum 2 GM regions: for QoS fast & slow */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3)
		mm->gm_max_cnt = 2;

	/* currently, we only support one DTCM region in v2_2 */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V2_2)
		mm->dtcm_max_cnt = 1;

	if (version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		mm->hold_tcb_head = kmem_cache_zalloc(mm->hold_tbuf_cache, GFP_KERNEL);
		if (!mm->hold_tcb_head)
			goto finish;
		INIT_LIST_HEAD(&mm->hold_tcb_head->node);
		mm->hold_tcb_head->nums = 0;
	}
	/**
	 * Device tree binding for Zhouyi V3:
	 *
	 *    gm-policy = <POLICY_NO>;
	 *
	 * where POLICY_NO should be one of the following values:
	 * 0: no GM;
	 * 1: GM is shared by all tasks in 1 cluster (by default if this attribute is not provided)
	 * 2: GM is divided half-by-half: for QoS slow & fast tasks, respectively
	 */
	if (version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		ret = device_property_read_u32(mm->dev, "gm-policy", &mm->gm_policy);
		if (ret || mm->gm_policy > AIPU_GM_POLICY_HALF_DIVIDED)
			mm->gm_policy = AIPU_GM_POLICY_SHARED;

		if (IS_ERR(aipu_common_create_attr(mm->dev, &mm->gm_policy_attr, "gm_policy", 0644,
						   aipu_gm_policy_sysfs_show,
						   aipu_gm_policy_sysfs_store))) {
			mm->gm_policy_attr = NULL;
			dev_err(mm->dev, "create gm_policy attr failed");
		}

		dev_info(mm->dev, "GM policy is %s",
			 mm->gm_policy == AIPU_GM_POLICY_SHARED ? "shared" : "half-by-half");
	} else {
		mm->gm_policy = AIPU_GM_POLICY_NONE;
		mm->gm_policy_attr = NULL;
	}

	group = iommu_group_get(mm->dev);
	if (group) {
		mm->has_iommu = true;
		mm->iommu_domain = iommu_get_domain_for_dev(mm->dev);
		if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3_2) {
			mm->valid_asid_cnt = 0;
			mm->iova_base = 0x100000000ULL;   // start from 4GB
			mm->iova_size = 0x800000000ULL;  // total szie 32GB

#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
			init_iova_domain(&mm->iova_domain, PAGE_SIZE,
					mm->iova_base >> PAGE_SHIFT);
#else
			init_iova_domain(&mm->iova_domain, PAGE_SIZE,
					mm->iova_base >> PAGE_SHIFT,
					DMA_BIT_MASK(36)>>PAGE_SHIFT);
#endif
			mm->buffer_count = 0;
			mutex_init(&mm->buffer_mutex);
			INIT_LIST_HEAD(&mm->buffer_list);
			mm->host_aipu_offset = 0;
			init_cache_ops(mm);
			mm->dma_mask = 36;
		} else if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
			/*
			 * V3 + IOMMU: Use custom IOVA domain to support per-ASID 3GB range
			 * Enable discrete physical page allocation while keeping TCB
			 * relative addresses within 32-bit range.
			 */
			ret = aipu_mm_v3_init_iova_domain(mm);
			if (ret) {
				dev_err(mm->dev, "V3: failed to init custom IOVA domain (%d), falling back\n", ret);
				mm->use_v3_custom_iova = false;
				mm->valid_asid_cnt = 1;
			} else {
				mm->use_v3_custom_iova = true;
				mm->valid_asid_cnt = 4;
			}
		} else {
			mm->valid_asid_cnt = 1;
		}
	}
	iommu_group_put(group);
	dev_info(mm->dev, "AIPU is%s behind an IOMMU.\n", mm->has_iommu ? "" : " not");

	/*
	 * If AIPU is behind an IOMMU, in devicetree, memory-region attribute is optional;
	 * otherwise the device specific or system global DMA/CMA region is used;
	 *
	 * KMD can accept multiple DRAM regions and/or multiple SRAM regions;
	 */
	if (!mm->has_iommu)
		mm->res_cnt = aipu_mm_add_reserved_regions(mm);
	else if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && !mm->use_v3_custom_iova) {
		/*
		 * V3 + IOMMU (fallback mode): constrain DMA to 32-bit / 3GB
		 * This is the original behavior when custom IOVA fails.
		 */
#if (KERNEL_VERSION(5, 5, 0) <= LINUX_VERSION_CODE)
		mm->dev->bus_dma_limit = 0xc0000000;
#elif (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
		mm->dev->bus_dma_mask = 0xc0000000;
#endif
		mm->dma_mask = 32;
		mm->default_asid_size = SZ_2G;
		if (aipu_mm_reserved_iova_for_never_map(mm, IOVA_ALLOC))
			dev_warn(mm->dev, "failed to reserve iova forbidden zones");
	}
#if AIPU_USE_STANDARD_DMA_API_FOR_V3_2
	else if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3_2)
		mm->res_cnt = aipu_mm_add_iova_region(mm);
#endif

	if (version > AIPU_ISA_VERSION_ZHOUYI_V1 && mm->res_cnt)
		aipu_mm_set_asid_base(mm);

	dev_info(mm->dev, "driver mem management is %s\n", mm->res_cnt ? "enabled" : "disabled");

finish:
	if (ret)
		aipu_deinit_mm(mm);
	return ret;
}

/**
 * @aipu_deinit_mm() - de-initialize mm module while kernel module unloading
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 */
int aipu_deinit_mm(struct aipu_memory_manager *mm)
{
	int id = AIPU_BUF_ASID_0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region_obj *next = NULL;

	if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (mm->gm_policy_attr) {
			aipu_common_destroy_attr(mm->dev, &mm->gm_policy_attr);
			mm->gm_policy_attr = NULL;
		}
	}

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (mm->hold_tbuf_cache) {
			aipu_mm_hold_tcb_buf_free(mm);
			kmem_cache_free(mm->hold_tbuf_cache, mm->hold_tcb_head);
			mm->hold_tcb_head = NULL;
			kmem_cache_destroy(mm->hold_tbuf_cache);
			mm->hold_tbuf_cache = NULL;
		}
	}

	if (mm->mem.head) {
		list_for_each_entry_safe(obj, next, &mm->mem.head->list, list)
			aipu_mm_destroy_region_object(mm, obj);

		aipu_mm_destroy_region_object(mm, mm->mem.head);
		mm->mem.head = NULL;
	}

	for (id = AIPU_BUF_ASID_0; id < ZHOUYI_ASID_COUNT; id++) {
		if (mm->ase[id].head) {
			list_for_each_entry_safe(obj, next, &mm->ase[id].head->list, list) {
				list_del(&obj->list);
				kmem_cache_free(mm->obj_cache, obj);
				/* reg has been freed when destroying mm->mem */
			}

			kmem_cache_free(mm->reg_cache, mm->ase[id].head->reg);
			kmem_cache_free(mm->obj_cache, mm->ase[id].head);
			mm->ase[id].head = NULL;
		}
	}

	if (mm->has_iommu && mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (mm->use_v3_custom_iova)
			aipu_mm_v3_deinit_iova_domain(mm);
		else
			aipu_mm_reserved_iova_for_never_map(mm, IOVA_FREE);
	}
	mm->res_cnt = 0;
	kmem_cache_destroy(mm->obj_cache);
	mm->obj_cache = NULL;
	kmem_cache_destroy(mm->reg_cache);
	mm->reg_cache = NULL;
	kmem_cache_destroy(mm->tbuf_cache);
	mm->tbuf_cache = NULL;
	mm->iommu_domain = NULL;
	return 0;
}

static int aipu_mm_direct_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
				struct file *filp, struct aipu_tcb_buf **tcb_alloc)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_tcb_buf *tcb = NULL;

	obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY, buf_req->bytes, 0, 0,
					   filp, false);
	if (!obj || !obj->reg)
		return -ENOMEM;

	ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, obj->reg, filp, &tcb);
	if (ret) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_lock(&mm->lock);
	list_add(&obj->list, &mm->mem.head->list);
	mutex_unlock(&mm->lock);

	if (tcb_alloc)
		*tcb_alloc = tcb;
	return 0;

err:
	aipu_mm_destroy_region_object(mm, obj);
	return ret;
}

static bool aipu_mm_check_address_validity(struct aipu_memory_manager *mm,
					   struct aipu_buf_desc *desc)
{
	u64 asid_size = 0;
	u64 asid_base = 0;
	u64 remainder = 0;
	u64 lower_bound = 0;
	u64 upper_bound = 0;
	u64 desc_end = 0;
	u64 asid0_base = 0;
	u64 size_4G = 0x100000000ULL;
	u64 relative_offset;

	/* V3 with custom IOVA: check relative offset is within 3GB */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->use_v3_custom_iova) {
		asid_base = aipu_mm_get_asid_base(mm, desc->asid);
		
		/* Check pa is in valid ASID range */
		if (desc->pa < asid_base ||
		    desc->pa >= asid_base + mm->asid_iova[desc->asid].iova_size) {
			dev_err(mm->dev, "V3: pa 0x%llx outside ASID %d range [0x%llx, 0x%llx)\n",
				desc->pa, desc->asid, asid_base,
				asid_base + mm->asid_iova[desc->asid].iova_size);
			return false;
		}
		
		/* Check relative offset within 3GB */
		relative_offset = desc->pa - asid_base;
		if (relative_offset >= 0xC0000000ULL) {
			dev_err(mm->dev, "V3: relative offset 0x%llx >= 3GB\n", relative_offset);
			return false;
		}
		
		/* Check doesn't cross 3GB boundary */
		if (relative_offset + desc->bytes > 0xC0000000ULL) {
			dev_err(mm->dev, "V3: buffer crosses 3GB boundary\n");
			return false;
		}
		
		return true;
	}

	/* Original V3 fallback mode */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		remainder = desc->pa % size_4G;
		if ((remainder + desc->bytes) >= size_4G)
			return false;
	}

	asid_size = aipu_mm_get_asid_size(mm, desc->asid);
	if (asid_size <= SZ_2G)
		return true;

	asid_base = aipu_mm_get_asid_base(mm, desc->asid);
	if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		lower_bound = asid_base + size_4G - SZ_1G;
		upper_bound = asid_base + size_4G;

		if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3_2) {
			lower_bound += SZ_512M;
		} else {
			if (desc->asid > AIPU_BUF_ASID_0) {
				asid0_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0);
				if (asid0_base != asid_base)
					lower_bound += SZ_512M;
			}
		}

		desc_end = desc->pa + desc->bytes;
		if (desc->pa < upper_bound && desc_end >= lower_bound)
			return false;
	}

	return true;
}

/**
 * @aipu_mm_alloc() - alloc memory buffer for user request
 * @mm:      pointer to memory manager struct initialized in aipu_init_mm()
 * @buf_req: pointer to buffer request struct from userland
 * @filp:    pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	int type;
	unsigned long flags;
	struct aipu_tcb_buf *tbuf = NULL;
	bool allocated = false;
	bool fall_back = false;

	if (!mm || !buf_req)
		return -EINVAL;

	if (!buf_req->bytes || !is_power_of_2(buf_req->align_in_page) ||
	    buf_req->asid >= ZHOUYI_ASID_COUNT) {
		dev_err(mm->dev, "[malloc] invalid alloc request: bytes 0x%llx, align %u, asid %u",
			buf_req->bytes, buf_req->align_in_page, buf_req->asid);
		return -EINVAL;
	}

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V1)
		buf_req->asid = AIPU_BUF_ASID_0;

	if (buf_req->region < AIPU_MEM_REGION_TYPE_MAX) {
		type = buf_req->region;
	} else {
		dev_err(mm->dev, "[malloc] invalid alloc request: region type %d",
			buf_req->region);
		return -EINVAL;
	}

	/* fall back to SRAM if DTCM/GM is not applicable for certain archs */
	if (mm->version < AIPU_ISA_VERSION_ZHOUYI_V2_2 && type == AIPU_MEM_REGION_TYPE_DTCM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	if ((mm->version < AIPU_ISA_VERSION_ZHOUYI_V3 || mm->gm_policy == AIPU_GM_POLICY_NONE) &&
	    type == AIPU_MEM_REGION_TYPE_GM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	/* Support the smmu dynamic weight buffer allocation. */
	if ((mm->has_iommu && type == AIPU_MEM_REGION_TYPE_MEMORY &&
	    (buf_req->data_type == AIPU_MM_DATA_TYPE_WEIGHT ||
	    buf_req->asid > AIPU_BUF_ASID_0)) || !mm->res_cnt) {
		/* V3 with custom IOVA: use dedicated IOVA allocation */
		if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
		    mm->use_v3_custom_iova) {
			ret = aipu_alloc_dma_iova_phy(mm, buf_req, filp);
			allocated = !ret;
			goto tcb_handle;
		}
		ret = aipu_mm_direct_alloc(mm, buf_req, filp, &tbuf);
		allocated = !ret;
		goto tcb_handle;
	}

	mutex_lock(&mm->lock);

	/* should check after lock */
	if (type == AIPU_MEM_REGION_TYPE_SRAM && mm->sram_disable)
		type = AIPU_MEM_REGION_TYPE_MEMORY;

	/* V3 with custom IOVA: use dedicated IOVA allocation for all memory types */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova &&
	    type == AIPU_MEM_REGION_TYPE_MEMORY) {
		ret = aipu_alloc_dma_iova_phy(mm, buf_req, filp);
		allocated = !ret;
		goto tcb_handle;
	}

alloc:
	list_for_each_entry(obj, &mm->ase[buf_req->asid].head->list, list) {
		if (obj->reg->type == type) {
			ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, obj->reg, filp, &tbuf);
			if (!ret) {
				allocated = true;
				break;
			}
		}
	}

	/* fall back to main memory */
	if (!allocated && !fall_back) {
		if (type != AIPU_MEM_REGION_TYPE_MEMORY) {
			type = AIPU_MEM_REGION_TYPE_MEMORY;
			fall_back = true;
			goto alloc;
		} else if (buf_req->asid > AIPU_BUF_ASID_0) {
			dev_dbg(mm->dev, "fail to malloc memory size 0x%llx from asid %d ",
				buf_req->bytes, buf_req->asid);
			buf_req->asid++;
			dev_dbg(mm->dev, "change new asid %d\n", buf_req->asid);
			if (buf_req->asid < mm->valid_asid_cnt)
				goto alloc;
			else
				dev_err(mm->dev, "Inalid asid parameter.\n");
		}
	}

	WARN_ON(buf_req->desc.pa % (buf_req->align_in_page << PAGE_SHIFT));
	mutex_unlock(&mm->lock);

tcb_handle:
	if (tbuf) {
		spin_lock_irqsave(&mm->slock, flags);
		list_add_tail(&tbuf->node, &tbuf->reg->tcb_buf_head->node);
		spin_unlock_irqrestore(&mm->slock, flags);
	}

	if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (!aipu_mm_check_address_validity(mm, &buf_req->desc)) {
			aipu_mm_free(mm, &buf_req->desc, filp, true);
			allocated = false;
			ret = -ENOMEM;
		}
	}

	if (!allocated) {
		dev_err(mm->dev,
			"failed allocating: asid %d iova 0x%llx, bytes 0x%llx,"
			" page align %d, type %d\n",
			buf_req->asid, buf_req->desc.pa,
			buf_req->bytes, buf_req->align_in_page,
			buf_req->data_type);
	} else {
		dev_dbg(mm->dev,
			"allocate done (%s): asid %d iova 0x%llx, type %d, "
			"bytes 0x%llx (al %d, rg %d)\n",
			fall_back ? "fall back to memory" : "as requested",
			buf_req->asid, buf_req->desc.pa, buf_req->data_type,
			buf_req->bytes, buf_req->align_in_page,
			buf_req->desc.region);
	}
	return ret;
}

static int aipu_mm_direct_free(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
			       bool unlock)
{
	struct aipu_tcb_buf *tbuf = NULL;
	u64 iova = 0;

	if (!mm || !reg)
		return -EINVAL;

	if (unlock)
		reg->locked = false;
	else if (reg->locked)
		return DEFERRED_FREE;

	iova = reg->base_iova;

	/* do not return tbuf for deferred free cases */
	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, iova);
	if (tbuf && (tbuf->dep_job_id || tbuf->pinned)) {
		dev_info(reg->dev, "deferred free (direct) TCB: iova 0x%llx\n", iova);
		return DEFERRED_FREE;
	}

	/* do free */
	aipu_mm_destroy_region_object(mm, reg->obj);

	dev_dbg(mm->dev, "direct free done: iova 0x%llx\n", iova);
	return 0;
}

/**
 * @aipu_mm_free() - free buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be released
 * @filp: pointer to the file struct
 * @unlock: unlock the page or not
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp,
		 bool unlock)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !buf)
		return -EINVAL;

	/* V3 Custom IOVA: use dedicated free path */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		return aipu_free_dma_iova_phy(mm, buf, filp);
	}

	reg = aipu_mm_find_region(mm, buf->pa, "free");
	if (!reg)
		return -EINVAL;

	mutex_lock(&mm->lock);
	if (reg->reserved)
		ret = aipu_mm_free_in_region(mm, buf, reg, unlock);
	else
		ret = aipu_mm_direct_free(mm, reg, unlock);
	mutex_unlock(&mm->lock);

	if (ret == DEFERRED_FREE)
		ret = 0;

	return ret;
}

static void tensor_dcache_inval_poc(void *start, void *end) {
    uintptr_t line_size, addr_start, addr_end, tmp;

    line_size = cache_line_size();
    tmp = line_size - 1;

    addr_start = (uintptr_t)start;
    addr_end = (uintptr_t)end;

    if (addr_end & tmp) {
        addr_end &= ~tmp;
        __asm__ __volatile__("dc civac, %0" : : "r" (addr_end) : "memory");
    }

    if (addr_start & tmp) {
        addr_start &= ~tmp;
        __asm__ __volatile__("dc civac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
    }

    while (addr_start < addr_end) {
        __asm__ __volatile__("dc ivac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
    }

    __asm__ __volatile__("dsb sy" : : : "memory");
}

static void tensor_flush_dcache_range(void *start, void *end) {
	uintptr_t addr_start, addr_end;
	const uint64_t line_size = cache_line_size();

	addr_start = (uintptr_t)start;
	addr_end = (uintptr_t)end;

	// make sure addr_start and addr_end are cache line aligned
	addr_start = addr_start & ~(line_size - 1);
	addr_end = (addr_end + line_size - 1) & ~(line_size - 1);

	while (addr_start < addr_end) {
		__asm__ volatile ("dc cvac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
	}

	// Ensure all cache operations complete
	__asm__ volatile ("dsb sy" : : : "memory");

}

/**
 * @aipu_mm_cache_flush() - invald buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be invalid
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_cache_flush(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block;
	u64 iova;
	u32 asid;

	if (!mm || !buf)
		return -EINVAL;

	/* V3 custom IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, buf->pa, asid);
			if (!iova)
				continue;
			
			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block && block->va) {
				aipu_cache_flush(mm, block->va, block->size);
				return 0;
			}
		}
		dev_dbg(mm->dev, "V3: cache_flush no block for pa 0x%llx\n", buf->pa);
		return 0;
	}

	reg = aipu_mm_find_region(mm, buf->pa, "flush");
	if (!reg)
	{
		dev_err(mm->dev, "The region of buf->pa 0x%llx not found\n", buf->pa);
		return 0;
	}

	if(mm->has_iommu) {
		mutex_lock(&mm->lock);
		tensor_flush_dcache_range(reg->base_va, (void *)reg->base_va + reg->bytes);
		mutex_unlock(&mm->lock);
	}

	return ret;
}

/**
 * @aipu_mm_cache_invalid() - invald buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be invalid
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_cache_invalid(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block;
	u64 iova;
	u32 asid;

	if (!mm || !buf)
		return -EINVAL;

	/* V3 custom IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, buf->pa, asid);
			if (!iova)
				continue;
			
			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block && block->va) {
				aipu_cache_invalidate(mm, block->va, block->size);
				return 0;
			}
		}
		dev_dbg(mm->dev, "V3: cache_invalid no block for pa 0x%llx\n", buf->pa);
		return 0;
	}

	reg = aipu_mm_find_region(mm, buf->pa, "invalid");
	if (!reg)
	{
		dev_err(mm->dev, "The region of buf->pa 0x%llx not found\n", buf->pa);
		return 0;
	}

	if(mm->has_iommu) {
		mutex_lock(&mm->lock);
		tensor_dcache_inval_poc(reg->base_va, (void *)reg->base_va + reg->bytes);
		mutex_unlock(&mm->lock);
	}

	return ret;
}

/**
 * @aipu_mm_free_buffers() - free all the buffers allocated from one fd
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 */
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region_obj *next = NULL;

	/* V3 custom IOVA cleanup */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		aipu_mm_v3_free_buffers_by_filp(mm, filp);
		return;
	}

	if (mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3_2 && mm->has_iommu)
		aipu_free_all_dma_iova_phy(mm, filp);

	mutex_lock(&mm->lock);
	list_for_each_entry_safe(obj, next, &mm->mem.head->list, list) {
		if (obj->reg->filp == filp)
			aipu_mm_direct_free(mm, obj->reg, true);
			/* --- reg should not be used below --- */
	}
	mutex_unlock(&mm->lock);

	if (mm->res_cnt) {
		list_for_each_entry_safe(obj, next, &mm->mem.head->list, list)
			aipu_mm_free_filp_in_region(mm, obj->reg, filp);
	}
}

static int aipu_vma_map_pages(struct vm_area_struct *vma, struct page **pages,
				unsigned long num)
{
	unsigned long count = vma_pages(vma);
	unsigned long uaddr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff;
	int ret, i;

	/* Fail if the user requested offset is beyond the end of the object */
	if (offset >= num)
		return -ENXIO;

	/* Fail if the user requested size exceeds available object size */
	if (count > num - offset)
		return -ENXIO;

	for (i = 0; i < count; i++) {
		ret = vm_insert_page(vma, uaddr, pages[offset + i]);
		if (ret < 0)
			return ret;
		uaddr += PAGE_SIZE;
	}

	return 0;
}

/**
 * @aipu_mm_mmap_buf() - mmap an allocated buffer for user thread
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @vma: pointer to the vm_area_struct
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_mmap_buf(struct aipu_memory_manager *mm, struct vm_area_struct *vma, struct file *filp)
{
	int ret = 0;
	dma_addr_t iova = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block = NULL;
	dma_addr_t dev_pa;
	u32 asid;

	if (!mm || !vma)
		return -EINVAL;

	dev_pa = vma->vm_pgoff << PAGE_SHIFT;

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE || \
	(defined(__ANDROID_COMMON_KERNEL__) && KERNEL_VERSION(6, 1, 43) <= LINUX_VERSION_CODE)
	vm_flags_set(vma, VM_IO);
#else
	vma->vm_flags |= VM_IO;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);
#else
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif

	/* V3 custom IOVA: dev_pa is NPU view, convert to IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, dev_pa, asid);
			if (!iova)
				continue;
			
			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block)
				break;
		}
		
		if (!block) {
			dev_err(mm->dev, "V3 mmap: no block found for pa 0x%llx\n", dev_pa);
			return -EINVAL;
		}

		vma->vm_pgoff = 0;
		ret = aipu_vma_map_pages(vma, block->pages, block->page_count);
		if (ret)
			dev_err(mm->dev, "V3 mmap failed at pa 0x%llx (ret = %d)", dev_pa, ret);
	}
	/* V3.2 path */
	else if(mm->version >= AIPU_ISA_VERSION_ZHOUYI_V3_2 && mm->has_iommu) {
		iova = dev_pa;
		block = aipu_get_block_buffer(mm, iova, "mmap");
		if(!block)
			return -EINVAL;

		vma->vm_pgoff = 0;
		ret = aipu_vma_map_pages(vma, block->pages, block->page_count);
		if (ret)
			dev_err(mm->dev, "dma mmap failed at iova 0x%llx (ret = %d)", iova, ret);
	} else {
		iova = dev_pa;
		reg = aipu_mm_find_region(mm, iova, "mmap");
		if (!reg)
			return -EINVAL;
		vma->vm_pgoff = (iova - reg->base_iova) >> PAGE_SHIFT;
		ret = dma_mmap_attrs(reg->dev, vma, reg->base_va, reg->base_pa,
				reg->bytes, reg->attrs);
		if (ret)
			dev_err(mm->dev, "dma_mmap_attrs failed at iova 0x%llx (ret = %d)", iova, ret);
	}

	vma->vm_pgoff = dev_pa >> PAGE_SHIFT;

	return ret;
}

/**
 * @aipu_mm_disable_sram_allocation() - disable buffer allocations from soc sram
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_disable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm || !mm->res_cnt)
		return -EINVAL;

	mutex_lock(&mm->lock);
	/* If SRAM is under using by driver & AIPU, it cannot be disabled. */
	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;

		if (reg->type == AIPU_MEM_REGION_TYPE_SRAM &&
		    !bitmap_empty(reg->bitmap, reg->count)) {
			dev_err(mm->dev, "the SRAM region to be disabled is under using");
			ret = -EPERM;
			break;
		}
	}

	if (!ret) {
		int found = 0;

		list_for_each_entry(sram_disable_per_fd, &mm->sram_disable_head->list, list) {
			if (sram_disable_per_fd->filp == filp) {
				sram_disable_per_fd->cnt++;
				found = 1;
				break;
			}
		}
		if (!found) {
			sram_disable_per_fd = kzalloc(sizeof(*sram_disable_per_fd), GFP_KERNEL);
			if (!sram_disable_per_fd) {
				ret = -ENOMEM;
				goto unlock;
			}
			sram_disable_per_fd->cnt++;
			sram_disable_per_fd->filp = filp;
			list_add(&sram_disable_per_fd->list, &mm->sram_disable_head->list);
		}
		mm->sram_disable++;
	}
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

/**
 * @aipu_mm_enable_sram_allocation() - enable buffer allocations from soc sram (disabled before)
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_enable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp)
{
	int ret = 0;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm || !mm->res_cnt)
		return -EINVAL;

	mutex_lock(&mm->lock);
	if (mm->sram_disable == 0) {
		ret = -EPERM;
		goto unlock;
	}

	list_for_each_entry(sram_disable_per_fd, &mm->sram_disable_head->list, list) {
		if (sram_disable_per_fd->filp == filp) {
			if (sram_disable_per_fd->cnt)
				sram_disable_per_fd->cnt--;
			break;
		}
	}
	mm->sram_disable--;
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

/**
 * @aipu_mm_get_va() - get the kernel virtual address of an allocated buffer.
 *                     currently do not support non-tcb va from importers.
 * @mm:     pointer to memory manager struct initialized in aipu_init_mm()
 * @dev_pa: device physical address returned by aipu_mm_alloc()
 *
 * Return: 0 on success and error code otherwise.
 */
char *aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_phy_block *block;
	u64 iova;
	u32 asid = 0;  /* TODO: need to pass asid or determine from context */

	if (!mm)
		return NULL;

	/* V3 custom IOVA path */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		/* Try each ASID to find the buffer */
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, dev_pa, asid);
			if (!iova)
				continue;
			
			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block && block->va) {
				/* Calculate offset within block */
				return block->va + (iova - block->iova_start);
			}
		}
		dev_dbg(mm->dev, "V3: get_va failed for pa 0x%llx\n", dev_pa);
		return NULL;
	}

	/* Original path */
	reg = aipu_mm_find_region(mm, dev_pa, "get_va");
	if (!reg)
		return NULL;

	return (char *)((unsigned long)reg->base_va + dev_pa - reg->base_iova);
}

/**
 * @aipu_mm_get_tcb() - get the pointer to the TCB of a TCB list.
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @pa: address of the TCB
 *
 * Return: tcb va on success or NULL otherwise.
 */
struct aipu_tcb *aipu_mm_get_tcb(struct aipu_memory_manager *mm, u64 pa)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_phy_block *block = NULL;
	unsigned long flags;
	struct aipu_tcb *tcb = NULL;
	char *va = NULL;
	u64 iova;
	u32 asid;

	if (!mm)
		return NULL;

	/* V3 custom IOVA: pa is NPU view, need to convert to IOVA and find block */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, pa, asid);
			if (!iova)
				continue;

			block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
			if (block && block->va) {
				va = block->va + (iova - block->iova_start);
				return (struct aipu_tcb *)va;
			}
		}
		dev_err(mm->dev, "[get_tcb] V3: no TCB buffer at pa 0x%llx\n", pa);
		return NULL;
	}

	/* Original path */
	reg = aipu_mm_find_region(mm, pa, "find_tcb");
	if (!reg)
		return NULL;
	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, pa);
	if (!tbuf) {
		dev_err(mm->dev, "[get_tcb] no TCB buffer is found at iova 0x%llx", pa);
		goto unlock;
	}

	reg = tbuf->reg;
	tcb = (struct aipu_tcb *)((char *)(reg->base_va) + pa - reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return tcb;
}

/**
 * @aipu_mm_set_tcb_tail() - set the pointer to the tail TCB of a TCB list.
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @tail: address of the tail TCB of a TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
struct aipu_tcb *aipu_mm_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_phy_block *block;
	unsigned long flags;
	struct aipu_tcb *ret = NULL;
	char *va = NULL;
	u64 iova;
	u32 asid;

	/* V3 custom IOVA */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
			iova = aipu_mm_v3_pa_to_iova(mm, tail, asid);
			if (!iova)
				continue;
			
			spin_lock_irqsave(&mm->slock, flags);
			tbuf = aipu_mm_find_tcb_buf_no_lock(mm, iova, "set_tail");
			if (tbuf) {
				block = aipu_mm_v3_find_block_by_iova(mm, iova, asid);
				if (block && block->va) {
					va = block->va + (iova - block->iova_start);
					tbuf->tail = tail;
					tbuf->tail_tcb = (struct aipu_tcb *)va;
					ret = tbuf->tail_tcb;
				}
			}
			spin_unlock_irqrestore(&mm->slock, flags);
			if (ret)
				return ret;
		}
		return NULL;
	}

	/* Original path */
	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, tail, "set_tail");
	if (!tbuf)
		goto unlock;

	reg = tbuf->reg;
	tbuf->tail = tail;
	tbuf->tail_tcb = (struct aipu_tcb *)((char *)(reg->base_va) + tail - reg->base_iova);
	ret = tbuf->tail_tcb;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

/**
 * @aipu_mm_link_tcb() - link a TCB list to the tail of an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be linked
 * @next_head_32: head address of the next TCB list
 * @next_job_id:  Job ID of the next TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id)
{
	struct aipu_hold_tcb_buf *htbuf = NULL;
	unsigned long flags;
	int ret = 0;
	u32 temp = 0;

	htbuf = aipu_mm_get_hold_htbuf(mm, prev_tail);
	if (!htbuf)
		return -EFAULT;

	spin_lock_irqsave(&mm->shlock, flags);
	htbuf->hold_tcb->next = next_head_32;
	htbuf->hold_tcb->flag &= ~TCB_FLAG_END_TYPE_GRID_END;
	temp = htbuf->hold_tcb->next;
	htbuf->next_head = next_head_32;
	if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_PREV)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINKED;
	else if (htbuf->status == AIPU_MEM_HOLD_TYPE_IDLE)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_NEXT;
	else
		dev_info(mm->dev, "ABNORMAL STATUS link hold tcb index %d status %d\n",
			 htbuf->index, htbuf->status);

	dev_dbg(mm->dev, "new tcb task link prev htcb index %d status %d\n",
		htbuf->index, htbuf->status);

	spin_unlock_irqrestore(&mm->shlock, flags);
	return ret;
}

/**
 * @aipu_mm_unlink_tcb() - unlink a TCB list from an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be unlinked
 * @free_tcb:     free TCB or not
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 curr_hold, bool free_tcb)
{
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *prev_htbuf = NULL;
	struct aipu_buf_desc buf;
	unsigned long flags;
	int ret = 0;
	u32 temp = 0;
	int prev_status = -1, prev_index = -1;

	if (!mm)
		return -EINVAL;

	htbuf = aipu_mm_get_hold_htbuf(mm, curr_hold);
	if (!htbuf)
		return -EFAULT;
	if (htbuf->prev_hold_tcb != 0) {
		prev_htbuf = aipu_mm_get_hold_htbuf(mm, htbuf->prev_hold_tcb);
		if (!prev_htbuf)
			dev_info(mm->dev, "PREV HOLD TCB 0x%llx IS NOT SET.", htbuf->prev_hold_tcb);
	}
	spin_lock_irqsave(&mm->shlock, flags);

	if (!htbuf->hold_tcb) {
		dev_err(mm->dev, "hold TCB was not set before unlinking it");
		ret = -EINVAL;
		goto unlock;
	}

	/*
	 * Job teardown vs IRQ bottom-half can race: prev_tbuf / tail_tcb or the
	 * previous hold buffer may already be cleared/freed. Dereferencing NULL
	 * here caused kernel Oops (e.g. NULL + 0x4) in aipu_mm_unlink_tcb.
	 */
	if (htbuf->prev_hold_tcb != 0) {
		if (!prev_htbuf || !prev_htbuf->hold_tcb) {
			dev_err(mm->dev,
				"unlink: invalid prev hold chain (prev_pa 0x%llx hold idx %d)\n",
				htbuf->prev_hold_tcb, htbuf->index);
			ret = -EINVAL;
			goto unlock;
		}
	}

	tbuf = htbuf->prev_tbuf;
	if (!tbuf || !tbuf->tail_tcb) {
		dev_err(mm->dev,
			"unlink: prev_tbuf or tail_tcb is NULL (hold idx %d)\n",
			htbuf->index);
		ret = -EINVAL;
		goto unlock;
	}

	if (prev_htbuf) {
		prev_htbuf->hold_tcb->next = 0;
		temp = prev_htbuf->hold_tcb->next;
		if (prev_htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_NEXT) {
			prev_htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
		} else if (prev_htbuf->status == AIPU_MEM_HOLD_TYPE_LINKED) {
			prev_htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_PREV;
		} else {
			dev_info(mm->dev, "ABNORMAL UNLINK PREV HOLD TCB STATUS %d\n",
				 prev_htbuf->status);
		}
		htbuf->prev_hold_tcb = 0;
		prev_status = prev_htbuf->status;
		prev_index = prev_htbuf->index;
	}

	tbuf->tail_tcb->next = 0;
	tbuf->dep_job_id = 0;
	tbuf->pinned = false;
	temp = tbuf->tail_tcb->next;
	htbuf->prev_head = 0;
	htbuf->prev_tail = 0;

	if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINKED)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_NEXT;
	else if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_PREV)
		htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
	else
		dev_info(mm->dev, "ABNORMAL UNLINK CUR HOLD TCB STATUS %d\n", htbuf->status);

	dev_dbg(mm->dev, "unlink prev hold index %d status %d, "
		"unlink curr hold tcb index %d status %d final holder index %d",
		prev_index, prev_status, htbuf->index, htbuf->status,
		mm->hold_tcb_head->hold_index);

unlock:
	spin_unlock_irqrestore(&mm->shlock, flags);
	if (!ret && free_tcb) {
		memset(&buf, 0, sizeof(buf));
		buf.pa = tbuf->head;
		aipu_mm_free(mm, &buf, NULL, false);
	}
	htbuf->prev_tbuf = NULL;
	return ret;
}

/**
 * @aipu_mm_pin_tcb() - pin a TCB list until it is ready to be freed in a future moment
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @tail: address of the tail TCB of a TCB list
 */
void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_tcb_buf *tbuf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, tail, NULL);
	if (!tbuf)
		goto unlock;

	tbuf->pinned = true;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
}

u64 aipu_mm_get_asid_base(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	if (mm->res_cnt)
		return mm->ase[asid].base;

	return mm->default_asid_base;
}

u64 aipu_mm_get_asid_size(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	if (mm->res_cnt)
		return mm->ase[asid].range;

	return mm->default_asid_size;
}

void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	int i;

	if (!mm || !cap)
		return;

	for (i = 0; i < mm->valid_asid_cnt; i++)
		cap->asid_base[i] = aipu_mm_get_asid_base(mm, i);
}

u32 aipu_mm_get_asid_cnt(struct aipu_memory_manager *mm)
{
	if (!mm)
		return 0;
	return mm->valid_asid_cnt;
}

int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes)
{
	if (!mm || !bytes || mm->gm_policy == AIPU_GM_POLICY_NONE)
		return -EINVAL;

	mm->gm_bytes = bytes;
	return 0;
}

int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next)
{
	int ret = 0;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return -EINVAL;

	if (next == mm->gm_policy)
		return ret;

	mutex_lock(&mm->lock);
	mm->gm_policy = next;
	mutex_unlock(&mm->lock);
	return ret;
}

void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	if (!mm || !cap)
		return;

	cap->gm0_size = 0;
	cap->gm1_size = 0;

	mutex_lock(&mm->lock);
	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		cap->gm0_size = mm->gm_bytes;
	} else {
		cap->gm0_size = mm->gm_bytes >> 1;
		cap->gm1_size = cap->gm0_size;
	}
	mutex_unlock(&mm->lock);
}

void get_dtcm(struct aipu_memory_manager *mm, u64 *base, u32 *size)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_mem_region_obj *obj = NULL;

	if (!mm || !base || !size || !mm->res_cnt)
		return;

	/* no lock because we only support reserved regions for DTCM */
	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;
		if (reg->type == AIPU_MEM_REGION_TYPE_DTCM) {
			*base = reg->base_iova;
			*size = reg->bytes;
			return;
		}
	}

	*base = 0;
	*size = 0;
}

static void aipu_debug_print_buffers(struct aipu_memory_manager *mm)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;
	int count = 0, index = 0;

	if (!mm)
		return;
	dev_info(mm->dev, "------------------------S-------------------------------------\n");

	//mutex_lock(&mm->buffer_mutex);
	dev_info(mm->dev, "Total buffers: %u\n", mm->buffer_count);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		dev_info(mm->dev, "Buffer %d: exec id 0x%llx IOVA=0x%llx, size=0x%llx, allocated=0x%llx ref_count %d\n",
			count++, buffer->exec_id, buffer->iova_start, buffer->iova_size, buffer->allocated_size, buffer->ref_count);
		index = 0;
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			if (block) {
				dev_info(mm->dev, "block %d: IOVA=0x%llx, size=0x%llx\n",
					index++, block->iova_start, block->size);
			}
		}
	}
	//mutex_unlock(&mm->buffer_mutex);
	dev_info(mm->dev, "------------------------E-------------------------------------\n");
	return;
}
static inline size_t get_cache_line_size(void)
{
#ifdef CONFIG_ARM64
	unsigned long cache_line_size;
	asm volatile("mrs %0, ctr_el0" : "=r" (cache_line_size));
	cache_line_size = 4 << ((cache_line_size >> 16) & 0xf);
    	return cache_line_size;
#elif defined(CONFIG_ARM)
	return L1_CACHE_BYTES;
#elif defined(CONFIG_X86_64)
	return boot_cpu_data.x86_cache_alignment;
#elif defined(CONFIG_RISCV)
	return L1_CACHE_BYTES;
#else
	return L1_CACHE_BYTES;
#endif
}

// For ARM64 ISA
#ifdef CONFIG_ARM64
static void arm64_cache_barrier(void)
{
	asm volatile("dsb sy" ::: "memory");
}

/* Clean+invalidate a range WITHOUT the trailing barrier. Callers that flush
 * many ranges back-to-back (e.g. per-page loops) should call this and then
 * issue arm64_cache_barrier() exactly once, instead of paying a full-system
 * dsb per range. */
static void arm64_flush_cache_range_nb(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc civac, %0" : : "r" (addr) : "memory");
	}
}

static void arm64_flush_cache_range(void *vaddr, size_t size)
{
	arm64_flush_cache_range_nb(vaddr, size);
	arm64_cache_barrier();
}

static void arm64_clean_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc cvac, %0" : : "r" (addr) : "memory");
	}
	asm volatile("dsb sy" ::: "memory");
}

static void arm64_invalidate_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;
	unsigned long cache_line_size = get_cache_line_size();
	unsigned long addr;

	//pr_info("cache_line_size %ld\n", cache_line_size);
	start &= ~(cache_line_size - 1);
	end = (end + cache_line_size - 1) & ~(cache_line_size - 1);

	for (addr = start; addr < end; addr += cache_line_size) {
		asm volatile("dc ivac, %0" : : "r" (addr) : "memory");
	}
	asm volatile("dsb sy" ::: "memory");
}

// For ARM32 ISA
#elif defined(CONFIG_ARM)
static void arm_flush_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;

	__cpuc_flush_dcache_area(vaddr, size);
	outer_flush_range(__pa(start), __pa(end));
}

static void arm_clean_cache_range(void *vaddr, size_t size)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end = start + size;

	__cpuc_clean_dcache_area(vaddr, size);
	outer_clean_range(__pa(start), __pa(end));
}

static void arm_invalidate_cache_range(void *vaddr, size_t size)
{
	__cpuc_flush_dcache_area(vaddr, size);
	outer_inv_range(__pa(vaddr), __pa(vaddr) + size);
}

// For x86_64 ISA
#elif defined(CONFIG_X86_64)
static void x86_flush_cache_range(void *vaddr, size_t size)
{
	clflush_cache_range(vaddr, size);
}

static void x86_clean_cache_range(void *vaddr, size_t size)
{
	clflush_cache_range(vaddr, size);
}

static void x86_invalidate_cache_range(void *vaddr, size_t size)
{
	clflush_cache_range(vaddr, size);
}

// For RISC-V ISA
#elif defined(CONFIG_RISCV)
static void riscv_flush_cache_range(void *vaddr, size_t size)
{
	flush_icache_range((unsigned long)vaddr, (unsigned long)vaddr + size);
}

static void riscv_clean_cache_range(void *vaddr, size_t size)
{
	flush_icache_range((unsigned long)vaddr, (unsigned long)vaddr + size);
}

static void riscv_invalidate_cache_range(void *vaddr, size_t size)
{
	flush_icache_range((unsigned long)vaddr, (unsigned long)vaddr + size);
}
#else
	#error "Unsupported architecture."
#endif

// Initialize cache operation pointer
static int __init init_cache_ops(struct aipu_memory_manager *mm)
{
	struct __cache_ops *cache_ops = &mm->cache_ops;
	cache_ops->cache_line_size = get_cache_line_size();

#ifdef CONFIG_ARM64
	cache_ops->flush_range = arm64_flush_cache_range;
	cache_ops->clean_range = arm64_clean_cache_range;
	cache_ops->invalidate_range = arm64_invalidate_cache_range;
	cache_ops->flush_range_nb = arm64_flush_cache_range_nb;
	cache_ops->flush_barrier = arm64_cache_barrier;
#elif defined(CONFIG_ARM)
	cache_ops->flush_range = arm_flush_cache_range;
	cache_ops->clean_range = arm_clean_cache_range;
	cache_ops->invalidate_range = arm_invalidate_cache_range;
	cache_ops->flush_range_nb = NULL;
	cache_ops->flush_barrier = NULL;
#elif defined(CONFIG_X86_64)
	cache_ops->flush_range = x86_flush_cache_range;
	cache_ops->clean_range = x86_clean_cache_range;
	cache_ops->invalidate_range = x86_invalidate_cache_range;
	cache_ops->flush_range_nb = NULL;
	cache_ops->flush_barrier = NULL;
#elif defined(CONFIG_RISCV)
	cache_ops->flush_range = riscv_flush_cache_range;
	cache_ops->clean_range = riscv_clean_cache_range;
	cache_ops->invalidate_range = riscv_invalidate_cache_range;
	cache_ops->flush_range_nb = NULL;
	cache_ops->flush_barrier = NULL;
#else
	#error "Unsupported architecture"
#endif

	return 0;
}
//For external api
void aipu_cache_flush(struct aipu_memory_manager *mm, void *vaddr, size_t size)
{
	if (!mm || !vaddr || !size)
		return;

	if(mm->cache_ops.flush_range)
		mm->cache_ops.flush_range(vaddr, size);
	return;
}

void aipu_cache_clean(struct aipu_memory_manager *mm, void *vaddr, size_t size)
{
	if (!mm || !vaddr || !size)
		return;

	if(mm->cache_ops.clean_range)
 		mm->cache_ops.clean_range(vaddr, size);
	return;
}

void aipu_cache_invalidate(struct aipu_memory_manager *mm, void *vaddr, size_t size)
{
	if (!mm || !vaddr || !size)
		return;

	if(mm->cache_ops.invalidate_range)
		mm->cache_ops.invalidate_range(vaddr, size);
	return;
}

static struct aipu_iova_buffer *aipu_get_iova_buffer(struct aipu_memory_manager *mm, u64 iova, char* str)
{
	//struct rb_node *node = mm->buffer_tree.rb_node;
	struct aipu_iova_buffer *buffer = NULL;

	if (!mm) {
		dev_err(mm->dev, "Invalid parameter to get iova buffer %s.\n", str);
		return NULL;
	}

	//pr_info("%s iova 0x%llx +\n", __FUNCTION__, iova);
	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if ((iova - buffer->iova_start) < buffer->iova_size) {
			mutex_unlock(&mm->buffer_mutex);
			return buffer;
		}
	}
	dev_err(mm->dev, "tid %d pid %d can't find iova 0x%llx %s\n",
		task_tgid_nr(current), task_pid_nr(current), iova, str);
	aipu_debug_print_buffers(mm);
	mutex_unlock(&mm->buffer_mutex);
	return NULL;
}

struct aipu_phy_block *aipu_get_block_buffer(struct aipu_memory_manager *mm, u64 iova, char* str)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;
	bool find = false;

	if (!mm) {
		dev_err(mm->dev, "Invalid parameter to get iova buffer %s.\n", str);
		return NULL;
	}

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if ((iova - buffer->iova_start) < buffer->iova_size) {
			find = true;
			break;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	if (find) {
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			if (block) {
				if ((iova - block->iova_start) < block->size)
					return block;
			}
		}
	}

	return NULL;
}

static struct aipu_iova_buffer *aipu_get_iova_buffer_by_exec_id(struct aipu_memory_manager *mm, u64 exec_id, char* str)
{
	struct aipu_iova_buffer *buffer = NULL;

	if (!mm) {
		dev_err(mm->dev, "Invalid parameter to get iova buffer %s.\n", str);
		return NULL;
	}

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (buffer->exec_id == exec_id) {
			mutex_unlock(&mm->buffer_mutex);
			return buffer;
		}
	}
	dev_err(mm->dev, "tid %d pid %d can't find exec_id 0x%llx %s\n",
		 task_tgid_nr(current), task_pid_nr(current), exec_id, str);
	mutex_unlock(&mm->buffer_mutex);
	return NULL;
}

static int aipu_insert_buffer_safe(struct aipu_memory_manager *mm,
                                   struct aipu_iova_buffer *new_buffer)
{
	if (!mm || !new_buffer) {
		if (mm && mm->dev)
		dev_err(mm->dev, "Invalid parameters for buffer insertion\n");
		return -EINVAL;
	}

	mutex_lock(&mm->buffer_mutex);
	list_add_tail(&new_buffer->node, &mm->buffer_list);
	mm->buffer_count++;
	mutex_unlock(&mm->buffer_mutex);
	//dev_info(mm->dev, "Successfully inserted buffer: IOVA=0x%llx\n",
	//	new_buffer->iova_start);

	return 0;
}

static struct aipu_iova_buffer* aipu_alloc_iova(struct aipu_memory_manager *mm,
						struct aipu_buf_request *buf_req,
			   			struct file *filp)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct aipu_iova_buffer* buffer = NULL;
	unsigned long pfn_hi;
	u64 reserved_iova;
	u64 iova_addr;
	u64 iova_size = 0;
	struct iova *iova;

	//Parameter check
	iova_size = ALIGN(buf_req->bytes, PAGE_SIZE);
	reserved_iova = ALIGN(buf_req->reserve_iova_size, PAGE_SIZE);
	pfn_hi = (mm->iova_base + mm->iova_size - 1) >> PAGE_SHIFT;

	if ((iova_size + reserved_iova) > mm->iova_size) {
		dev_err(mm->dev, "iova_size is larger than total size.\n");
		return NULL;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;

	iova = alloc_iova(iovad, (iova_size + reserved_iova) >> PAGE_SHIFT, pfn_hi, true);
	if (!iova) {
		mutex_lock(&mm->buffer_mutex);
		aipu_debug_print_buffers(mm);
		mutex_unlock(&mm->buffer_mutex);
		kfree(buffer);
		dev_err(mm->dev, "Failed to allocate IOVA\n");
		return NULL;
	}

	// init iova buffer
	iova_addr = iova->pfn_lo << PAGE_SHIFT;
	buffer->iova_start = iova_addr;
	buffer->iova_size = iova_size + reserved_iova;
	buffer->allocated_size = 0;
	buffer->region = buf_req->region;
	buffer->asid = buf_req->asid;
	buffer->data_type = buf_req->data_type;
	buffer->ref_count = 0;
	buffer->filp = filp;
	buffer->tgid = task_tgid_nr(current);
	buffer->pid = task_pid_nr(current);
	buffer->exec_id = buf_req->exec_id;
	INIT_LIST_HEAD(&buffer->phy_blocks);
	mutex_init(&buffer->phy_mutex);

	if(aipu_insert_buffer_safe(mm, buffer)){
		dev_err(mm->dev, "Failed to insert IOVA buffer.\n");
		free_iova(iovad, iova->pfn_lo);
		kfree(buffer);
		return NULL;
	}

	buf_req->desc.pa = iova_addr - mm->host_aipu_offset;
	buf_req->desc.region = AIPU_BUF_REGION_DEFAULT;
	buf_req->desc.dev_offset = iova_addr - mm->host_aipu_offset;
	buf_req->desc.asid = buf_req->asid;
	buf_req->desc.bytes = (iova->pfn_hi - iova->pfn_lo + 1) << PAGE_SHIFT;
	buf_req->desc.exec_id = buf_req->exec_id;
	//dev_info(mm->dev, "Allocated IOVA: 0x%llx, size: 0x%llx\n", iova_addr, buf_req->desc.bytes);

	return buffer;
}

static void aipu_free_iova(struct aipu_memory_manager *mm, struct aipu_iova_buffer* buffer)
{
	unsigned long pfn_lo = (buffer->iova_start + mm->host_aipu_offset) >> PAGE_SHIFT;

	mutex_lock(&mm->buffer_mutex);
	if (buffer->ref_count) {
		dev_info(mm->dev, "Warning, buffer 0x%llx ref_count %d still is using buffer.\n", buffer->iova_start, buffer->ref_count);
	}

	list_del(&buffer->node);
	mm->buffer_count--;
	free_iova(&mm->iova_domain, pfn_lo);
	kfree(buffer);
	mutex_unlock(&mm->buffer_mutex);
	//dev_info(mm->dev, "Freed IOVA: 0x%llx, size: 0x%llx ref_count %d\n", iova_addr, size, buffer->ref_count);
}

static void aipu_dma_free_pages(struct page **pages, int count)
{
	if (!pages)
		return;

	while (count--)
		__free_page(pages[count]);
	kvfree(pages);
}

static struct page **aipu_dma_alloc_pages(struct device *dev,
		unsigned int count, unsigned long order_mask, gfp_t gfp)
{
	struct page **pages;
	unsigned int i = 0, nid = dev_to_node(dev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	order_mask &= (2U << MAX_PAGE_ORDER) - 1;
#else
	order_mask &= (2U << MAX_ORDER) - 1;
#endif
	if (!order_mask)
		return NULL;

	pages = kvzalloc(count * sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;

	/* IOMMU can map any pages, so himem can also be used here */
	gfp |= __GFP_NOWARN | GFP_KERNEL;

	/* It makes no sense to muck about with huge pages */
	gfp &= ~__GFP_COMP;

	while (count) {
		struct page *page = NULL;
		unsigned int order_size;

		/*
		 * Higher-order allocations are a convenience rather
		 * than a necessity, hence using __GFP_NORETRY until
		 * falling back to minimum-order allocations.
		 */
		for (order_mask &= (2U << __fls(count)) - 1;
		     order_mask; order_mask &= ~order_size) {
			unsigned int order = __fls(order_mask);
			gfp_t alloc_flags = gfp;

			order_size = 1U << order;
			if (order_mask > order_size)
				alloc_flags |= __GFP_NORETRY;

			page = alloc_pages_node(nid, alloc_flags, order);
			if (!page)
				continue;
			if (order)
				split_page(page, order);
			break;
		}
		if (!page) {
			aipu_dma_free_pages(pages, i);
			return NULL;
		}
		count -= order_size;
		while (order_size--)
			pages[i++] = page++;
	}

	return pages;
}

static int aipu_add_block_into_buffer(struct aipu_iova_buffer *buffer, struct aipu_phy_block *block)
{
	struct aipu_dma_buf_priv *priv;
	if (!buffer || !block)
		return -EINVAL;

	mutex_lock(&buffer->phy_mutex);
	buffer->ref_count++;
	block->iova_start = buffer->iova_start + buffer->allocated_size;
	buffer->allocated_size += block->size;
	if (block->dmabuf) {
		priv = (struct aipu_dma_buf_priv *)block->dmabuf->priv;
		priv->dev_pa = block->iova_start;
		priv->dma_pa = block->iova_start;
		priv->bytes = block->size;
	}
	list_add_tail(&block->list, &buffer->phy_blocks);
	mutex_unlock(&buffer->phy_mutex);

	return 0;
}

static struct aipu_phy_block *aipu_remove_block_node(struct aipu_iova_buffer *buffer, u64 pa)
{
	struct aipu_phy_block *block = NULL;
	struct aipu_phy_block *tmp;

	if (!buffer)
		return NULL;

	mutex_lock(&buffer->phy_mutex);
	list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
		if (block->iova_start == pa) {
			list_del(&block->list);
			buffer->ref_count--;
			buffer->allocated_size -= block->size;
			mutex_unlock(&buffer->phy_mutex);
			return block;
		}
	}
	mutex_unlock(&buffer->phy_mutex);

	return NULL;
}

static struct aipu_phy_block *aipu_alloc_phy_block(struct aipu_memory_manager *mm,
						   struct aipu_iova_buffer *buffer,
						   struct aipu_buf_request *buf_req)
{
	struct aipu_phy_block *block;
	u32 page_count;
	u64 mem_size = 0;
	u64 iova_addr = 0;
	size_t alloc_sizes = PAGE_SIZE;

	iova_addr = buffer->iova_start + buffer->allocated_size;
	mem_size = ALIGN(buf_req->bytes, PAGE_SIZE);
	page_count = mem_size >> PAGE_SHIFT;

	/* if (page_count < 32) //0-128K
		alloc_sizes = PAGE_SIZE; //4KB
	else //if (page_count < 4096) //128k-16MB
		alloc_sizes = PAGE_SIZE*16; //128KB
	else //if (page_count < 16384) //16MB-64MB
		alloc_sizes = PAGE_SIZE*256; //1MB
	else //above 64MB
		alloc_sizes = PAGE_SIZE*512; //2MB
	*/

	if ((buffer->allocated_size + buf_req->bytes) > buffer->iova_size) {
		dev_err(mm->dev, "iova space is not enough for phy memory allcation.");
		return NULL;
	}

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return NULL;

	// alloc physical pages
	block->pages = aipu_dma_alloc_pages(mm->dev, page_count, alloc_sizes>>PAGE_SHIFT, GFP_KERNEL | __GFP_ZERO);
	if (!block->pages) {
		dev_err(mm->dev, "Failed to allocate %u pages\n", page_count);
		kfree(block);
		return NULL;
	}

	//block->iova_start = iova_addr;
	block->size = mem_size;
	block->page_count = page_count;
	block->sgt.nents = block->page_count;
	block->dmabuf = NULL;
	INIT_LIST_HEAD(&block->list);
	block->bind_memory = false;

	//Add block into buffer's list
	aipu_add_block_into_buffer(buffer, block);

	//back map pa and size
	buf_req->desc.pa = iova_addr;
	buf_req->desc.dev_offset = iova_addr;
	buf_req->desc.bytes = mem_size;

	//dev_info(mm->dev, "Allocated phy block: iova=0x%llx size=0x%llx\n",
	//	iova_addr, mem_size);

	return block;
}

static void aipu_free_phy_block(struct aipu_memory_manager *mm, struct aipu_phy_block *block)
{
	if (block) {
		vunmap(block->va);
		if(!block->bind_memory)
			aipu_dma_free_pages(block->pages, block->page_count);
		kfree(block);
	}
}

static int aipu_map_sg_to_iommu(struct aipu_memory_manager *mm,
                                struct aipu_phy_block *block)
{
	int nents_mapped;

	if (!mm || !block)
		return -EINVAL;

	// build scatter-gather table
	if (sg_alloc_table_from_pages(&block->sgt, block->pages, block->page_count, 0, block->size, GFP_KERNEL)) {
		dev_err(mm->dev, "Failed to allocate sg table from pages.\n");
		return -EINVAL;
	}

	// map to iommu
	//mapped_size = iommu_map_sg(mm->iommu_domain, block->iova_start, block->sgt.sgl, block->sgt.nents, IOMMU_READ | IOMMU_WRITE);
	nents_mapped = dma_map_sg(mm->dev, block->sgt.sgl, block->sgt.nents, DMA_BIDIRECTIONAL);
	if (nents_mapped <=0 ) {
		dev_err(mm->dev, "IOMMU mapping failed: nents_mapped=%d\n", nents_mapped);
		sg_free_table(&block->sgt);
		return -ENOMEM;
	}

	block->iova_start = sg_dma_address(block->sgt.sgl);
	block->va = vmap(block->pages, block->page_count, VM_MAP, pgprot_noncached(PAGE_KERNEL));
	sg_free_table(&block->sgt);
	//dev_info(mm->dev, "Mapped 0x%llx bytes to IOVA 0x%llx\n",
	//	block->size, block->iova_start);
	return 0;
}

int aipu_alloc_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp)
{
	struct aipu_iova_buffer *buffer = NULL;
	struct aipu_phy_block *block = NULL;
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	unsigned long flags;
	int ret = 0;

	if (!mm || !buf_req)
		return -EINVAL;

	/* V3 + IOMMU with custom IOVA domain */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		dev_dbg(mm->dev, "V3: alloc_dma_iova_phy mode=%d asid=%d bytes=0x%llx type=%d\n",
			buf_req->alloc_mode, buf_req->asid, buf_req->bytes, buf_req->data_type);
		switch (buf_req->alloc_mode) {
		case AIPU_DMA_BUF_MALLOC_DEFAULT:
		case AIPU_DMA_BUF_MALLOC_IOVA_PHY:
			// 1. Allocate IOVA in ASID range
			buffer = aipu_mm_v3_alloc_iova(mm, buf_req, filp);
			if (!buffer) {
				ret = -ENOMEM;
				goto OUT;
			}
			
			// 2. Allocate physical pages and map
			ret = aipu_mm_v3_alloc_phy_and_map(mm, buffer, buf_req);
			if (ret) {
				/* alloc-failure path: buffer never became visible to
				 * the NPU, so destroy immediately (no stale-ref risk). */
				aipu_mm_v3_destroy_buffer_now(mm, buffer);
				goto OUT;
			}
			
			// 3. Convert IOVA to NPU view PA (asid_base + offset)
			buf_req->desc.pa = aipu_mm_v3_iova_to_pa(mm, buffer->iova_start, buf_req->asid);
			buf_req->desc.dev_offset = buf_req->desc.pa;
			buf_req->desc.exec_id = buf_req->exec_id;
			buf_req->desc.asid = buf_req->asid;
			break;
			
		case AIPU_DMA_BUF_MALLOC_IOVA:
			// IOVA only
			buffer = aipu_mm_v3_alloc_iova(mm, buf_req, filp);
			if (!buffer) {
				ret = -ENOMEM;
				goto OUT;
			}
			// Return NPU view PA
			buf_req->desc.pa = aipu_mm_v3_iova_to_pa(mm, buffer->iova_start, buf_req->asid);
			buf_req->desc.dev_offset = buf_req->desc.pa;
			buf_req->desc.asid = buf_req->asid;
			break;
			
		case AIPU_DMA_BUF_MALLOC_PHY:
			// Find buffer by exec_id and allocate physical
			buffer = aipu_get_iova_buffer_by_exec_id(mm, buf_req->exec_id, "v3 malloc phy");
			if (!buffer) {
				ret = -EINVAL;
				goto OUT;
			}
			ret = aipu_mm_v3_alloc_phy_and_map(mm, buffer, buf_req);
			if (ret)
				goto OUT;
			buf_req->desc.pa = aipu_mm_v3_iova_to_pa(mm, 
				buffer->iova_start + buffer->allocated_size - buf_req->desc.bytes,
				buf_req->asid);
			buf_req->desc.asid = buf_req->asid;
			break;
			
		default:
			ret = -EINVAL;
			goto OUT;
		}

		/* Register TCB buffer for TCB-type allocations so hold_tcb can find it */
		if (buf_req->data_type == AIPU_MM_DATA_TYPE_TCB && mm->v3_tcb_buf_head) {
			struct aipu_tcb_buf *tbuf;

			tbuf = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
			if (tbuf) {
				INIT_LIST_HEAD(&tbuf->node);
				tbuf->reg = NULL;
				tbuf->head = buf_req->desc.pa;
				tbuf->tail = buf_req->desc.pa + buf_req->desc.bytes -
					     sizeof(struct aipu_tcb);
				tbuf->tail_tcb = NULL;
				tbuf->dep_job_id = 0;
				tbuf->pinned = false;

				spin_lock_irqsave(&mm->slock, flags);
				list_add_tail(&tbuf->node, &mm->v3_tcb_buf_head->node);
				spin_unlock_irqrestore(&mm->slock, flags);
			}
		}

		return 0;
	}

	/* V3.2 path (original) */
	if (buf_req->alloc_mode != AIPU_DMA_BUF_MALLOC_DEFAULT) {
		domain = mm->iommu_domain;
		iovad = &mm->iova_domain;
	}

	switch (buf_req->alloc_mode) {
	case AIPU_DMA_BUF_MALLOC_DEFAULT:
		//support standard dma api and reserved memory
		ret = aipu_mm_alloc(mm, buf_req, filp);
		break;

	case AIPU_DMA_BUF_MALLOC_IOVA:
		// alloc iova adress only
		buffer = aipu_alloc_iova(mm, buf_req, filp);
		if (!buffer) {
			dev_err(mm->dev, "Malloc iova FAILED size 0x%llx\n", buf_req->bytes + buf_req->reserve_iova_size);
			ret = -ENOMEM;
			goto OUT;
		}
		break;

	case AIPU_DMA_BUF_MALLOC_PHY:
		// alloc phy memory and map allocated iova address
		buffer = aipu_get_iova_buffer_by_exec_id(mm, buf_req->exec_id, "malloc phy");
		if (!buffer) {
			dev_err(mm->dev, "Can't find pa 0x%llx in iova domain.\n", buf_req->exec_id);
			ret = -EINVAL;
			goto OUT;
		}

		block = aipu_alloc_phy_block(mm, buffer, buf_req);
		if (!block) {
			dev_err(mm->dev, "OOM iova 0x%llx phy size 0x%llx FAILED.\n", buf_req->exec_id, buf_req->bytes);
			ret = -ENOMEM;
			goto OUT;
		}

		//map sg table to iommu
		ret = aipu_map_sg_to_iommu(mm, block);
		if (ret) {
			dev_err(mm->dev, "Map block iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
			aipu_free_phy_block(mm, block);
			goto OUT;
		}
		aipu_cache_invalidate(mm, block->va, block->size);
		break;

	case AIPU_DMA_BUF_MALLOC_IOVA_PHY:
		buffer = aipu_alloc_iova(mm, buf_req, filp);
		if (!buffer) {
			dev_err(mm->dev, "Malloc iova size 0x%llx FAILED.\n",  buf_req->bytes + buf_req->reserve_iova_size);
			ret = -ENOMEM;
			goto OUT;
		}

		block = aipu_alloc_phy_block(mm, buffer, buf_req);
		if (!block) {
			dev_err(mm->dev, "OOM iova 0x%llx phy size 0x%llx FAILD.\n", buffer->iova_start, buf_req->bytes);
			ret = -ENOMEM;
			goto err_free_iova;
		}

		//map iova and phy memory
		ret = aipu_map_sg_to_iommu(mm, block);
		if (ret) {
			dev_err(mm->dev, "Map block iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
			goto err_free_phy;
		}
		aipu_cache_invalidate(mm, block->va, block->size);
		break;

	default:
		ret = -EINVAL;
		goto OUT;
	}

	return 0;

err_free_phy:
	aipu_free_phy_block(mm, block);

err_free_iova:
	aipu_free_iova(mm, buffer);
OUT:
	return ret;
}


/**
 * aipu_mm_v3_find_buffer_by_iova - Find IOVA buffer by IOVA
 * @mm: memory manager
 * @iova: IOVA address
 * 
 * Return: buffer pointer or NULL
 */
static struct aipu_iova_buffer *aipu_mm_v3_find_buffer_by_iova(struct aipu_memory_manager *mm,
							      u64 iova)
{
	struct aipu_iova_buffer *buffer;

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (iova >= buffer->iova_start &&
		    iova < buffer->iova_start + buffer->iova_size) {
			mutex_unlock(&mm->buffer_mutex);
			return buffer;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	return NULL;
}

/**
 * aipu_mm_v3_find_tcb_buf_no_lock - Find TCB buffer by NPU view PA in V3 custom IOVA list
 * @mm: memory manager
 * @pa: NPU view physical address
 *
 * Must be called with mm->slock held.
 * Return: tcb_buf pointer or NULL
 */
static struct aipu_tcb_buf *aipu_mm_v3_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
							    u64 pa)
{
	struct aipu_tcb_buf *curr;

	if (!mm->v3_tcb_buf_head)
		return NULL;

	list_for_each_entry(curr, &mm->v3_tcb_buf_head->node, node) {
		if (pa >= curr->head && pa <= curr->tail)
			return curr;
	}

	return NULL;
}

int aipu_free_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_desc *desc, struct file *filp)
{
	u64 exec_id = desc->exec_id;
	struct aipu_iova_buffer* buffer = NULL;
	struct aipu_phy_block *block, *tmp;
	bool free_iova = true;
	u64 iova;

	if (!desc || !mm)
		return -EINVAL;

	/* V3 custom IOVA path */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && 
	    mm->has_iommu && mm->use_v3_custom_iova) {
		/*
		 * desc->asid is a [kmd] output field of struct aipu_buf_desc.
		 * User space does not have to fill it in for AIPU_IOCTL_FREE_BUF
		 * (Arm China's libaipudrv leaves it uninitialised), so don't trust
		 * it. The ASID slots do not overlap (ASID n starts at n * 4 GB),
		 * so the device PA names its ASID.
		 */
		if (!aipu_mm_v3_pa_to_iova(mm, desc->pa, desc->asid) &&
		    (desc->pa >> 32) < ZHOUYI_ASID_COUNT)
			desc->asid = desc->pa >> 32;

		/* Convert PA to IOVA */
		iova = aipu_mm_v3_pa_to_iova(mm, desc->pa, desc->asid);
		if (!iova) {
			dev_err(mm->dev, "V3: invalid PA 0x%llx for ASID %d\n", desc->pa, desc->asid);
			return -EINVAL;
		}
		
		// Find and free buffer
		buffer = aipu_mm_v3_find_buffer_by_iova(mm, iova);
		if (!buffer) {
			dev_err(mm->dev, "V3: can't find buffer for IOVA 0x%llx (PA 0x%llx, ASID %d)\n",
				iova, desc->pa, desc->asid);
			return -EINVAL;
		}

		/* Remove associated TCB buffer descriptor if any */
		{
			struct aipu_tcb_buf *tbuf;
			unsigned long tflags;
			bool had_tcb = false;

			spin_lock_irqsave(&mm->slock, tflags);
			tbuf = aipu_mm_v3_find_tcb_buf_no_lock(mm, desc->pa);
			if (tbuf) {
				had_tcb = true;
				list_del(&tbuf->node);
				kmem_cache_free(mm->tbuf_cache, tbuf);
			}
			spin_unlock_irqrestore(&mm->slock, tflags);

			v3dbg(mm, "FREE_REQ    pa=0x%llx iova=0x%llx asid=%u dtype=%s(%u) exec=0x%llx is_tcb=%d ref=%d tgid=%d\n",
			      desc->pa, iova, desc->asid,
			      aipu_mm_v3_dtype_str(buffer->data_type),
			      buffer->data_type, desc->exec_id, had_tcb,
			      buffer->ref_count, task_tgid_nr(current));
		}

		aipu_mm_v3_free_iova_buffer(mm, buffer);
		return 0;
	}

	/* Original V3.2 path */
	if (!desc->exec_id)
		return aipu_mm_free(mm, desc, filp, true);

	if (desc->exec_id >= AIPU_EXEC_ID)
		buffer = aipu_get_iova_buffer_by_exec_id(mm, exec_id, "free dma iova phy");

	if(!buffer) {
		buffer = aipu_get_iova_buffer(mm, desc->pa, "free iova phy");
		if(!buffer) {
			dev_err(mm->dev, "can't find iova buffer by iova 0x%llx exec id 0x%llx\n", desc->pa, exec_id);
			return -EINVAL;
		}
	}

	//rebind/bind dma buffer free one by one in iova buffer
	if (desc->exec_id == DMA_BUF_EXEC_ID){
		list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
			if (block->iova_start == desc->pa) {
				list_del(&block->list);
				buffer->ref_count--;
				buffer->allocated_size -= block->size;
				iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
				if (block->bind_memory)
					free_iova = false;
				aipu_free_phy_block(mm, block);
				break;
			}
		}
	//free all for job buffer once
	} else {
		list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
			if (!block->bind_memory) {
				list_del(&block->list);
				buffer->ref_count--;
				buffer->allocated_size -= block->size;
				iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
				aipu_free_phy_block(mm, block);
			} else {
				free_iova = false;
			}
		}
	}

	if (free_iova || !buffer->ref_count)
		aipu_free_iova(mm, buffer);

	return 0;
}

int aipu_rebind_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_rebind_buf_desc *desc, struct file *filp)
{
	u64 exec_id = desc->exec_id;
	u64 pa_unmap = desc->pa_unmap;
	struct aipu_iova_buffer* buffer;
	struct aipu_phy_block *block;
	int ret = 0;
	size_t mapped_size;

	if (!mm || !desc)
		return -EINVAL;

	//handle before iova buffer;
	buffer = aipu_get_iova_buffer(mm, pa_unmap, "rebind dma buffer");
	if (!buffer) {
		dev_err(mm->dev, "Cant find iova buffer 0x%llx.\n", desc->pa_unmap);
		return -EINVAL;
	}

	block = aipu_remove_block_node(buffer, pa_unmap);
	if (!block) {
		dev_err(mm->dev, "Can't unbind block.\n");
		return -ENXIO;
	}

	mapped_size = iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
	 if (mapped_size != block->size) {
	 	dev_err(mm->dev, "Can't unmap block 0x%llx ret = .\n", block->iova_start);
		return -ENXIO;
	}

	//iova buffer free
	if (!buffer->ref_count)
		aipu_free_iova(mm, buffer);

	//rebind new iova buffer
	buffer = aipu_get_iova_buffer_by_exec_id(mm, exec_id, "rebind dma buf");
	if (!buffer)
		return -ENXIO;

	aipu_add_block_into_buffer(buffer, block);
	block->bind_memory = true;
	ret = aipu_map_sg_to_iommu(mm, block);
	if (ret) {
		dev_err(mm->dev, "Map block to new iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
		aipu_free_phy_block(mm, block);
		goto OUT;
	}

	desc->pa = block->iova_start;
	aipu_cache_invalidate(mm, block->va, block->size);

OUT:
	return ret;
}

int aipu_bind_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_bind_buf_desc *desc, struct file *filp)
{
	u64 exec_id = desc->exec_id;
	struct aipu_iova_buffer* buffer;
	struct aipu_phy_block *block;
	int ret = 0;
	struct dma_buf *dmabuf = NULL;
	struct exporter_dma_buf *exp_buf = dmabuf->priv;

	if (!mm || !desc)
		return -EINVAL;

	//bind new iova buffer
	buffer = aipu_get_iova_buffer_by_exec_id(mm, exec_id, "rebind dma buf");
	if (!buffer) {
		return -EINVAL;
	}

	//handle before iova buffer;
	dmabuf = dma_buf_get(desc->fd);
	if(IS_ERR(dmabuf)) {
		dev_err(mm->dev, "fail to get dmabuf from fd.\n");
		return -EINVAL;
	}

	if ((buffer->allocated_size + dmabuf->size) > buffer->iova_size) {
		dev_err(mm->dev, "iova space is not enough for phy memory allcation.");
		return -EINVAL;
	}

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return -ENOMEM;

	exp_buf = dmabuf->priv;

	block->size = dmabuf->size;
	block->page_count = dmabuf->size>>PAGE_SHIFT;
	block->sgt.nents = block->page_count;
	//block->dmabuf = dmabuf;
	block->dmabuf = NULL;
	block->pages = exp_buf->pages;
	block->bind_memory = true;

	INIT_LIST_HEAD(&block->list);

	aipu_add_block_into_buffer(buffer, block);

	ret = aipu_map_sg_to_iommu(mm, block);
	if (ret) {
		dev_err(mm->dev, "Map block to new iova 0x%llx size 0x%llx FAILED.\n", block->iova_start, block->size);
		aipu_free_phy_block(mm, block);
		goto OUT;
	}
	desc->pa = block->iova_start;
	aipu_cache_invalidate(mm, block->va, block->size);

OUT:
	return ret;
}

static int aipu_free_all_dma_iova_phy(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_iova_buffer* buffer, *tmp1;
	struct aipu_phy_block *block, *tmp;
	struct aipu_iova_buffer** pbuf;
	int buffer_count = 0;
	int index = 0;

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer,&mm->buffer_list, node) {
		if(buffer->filp == filp)
			buffer_count++;
	}
	mutex_unlock(&mm->buffer_mutex);

	if (buffer_count == 0)
		return 0;

	pbuf = kzalloc(buffer_count * sizeof(*buffer), GFP_KERNEL);
	if (!pbuf)
		return -ENOMEM;

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry_safe(buffer, tmp1, &mm->buffer_list, node) {
		if(buffer->filp == filp) {
			pbuf[index] = buffer;
			index++;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	//unmap sg table and free phy memory
	pr_info("free buffer count %d for CTRL + C\n", buffer_count);
	for (index = 0; index < buffer_count; index++) {
		list_for_each_entry_safe(block, tmp, &pbuf[index]->phy_blocks, list) {
			if (block) {
				list_del(&block->list);
				iommu_unmap(mm->iommu_domain, block->iova_start, block->size);
				aipu_free_phy_block(mm, block);
			}
		}
		aipu_free_iova(mm, pbuf[index]);
	}

	kfree(pbuf);

	return 0;
}


/* ============================================================================
 * V3 Custom IOVA Domain Support
 * 
 * Enable per-ASID 3GB IOVA ranges for V3 with IOMMU:
 * - ASID 0: IOVA [0, 3GB)      -> NPU view: asid_base + offset
 * - ASID 1: IOVA [3GB, 6GB)    -> NPU view: asid_base + offset
 * - ASID 2: IOVA [6GB, 9GB)    -> NPU view: asid_base + offset
 * - ASID 3: IOVA [9GB, 12GB)   -> NPU view: asid_base + offset
 * ============================================================================ */

/**
 * aipu_mm_v3_init_iova_domain - Initialize IOVA domain for V3 per-ASID support
 * @mm: memory manager
 * 
 * Return: 0 on success, error code otherwise
 */
int aipu_mm_v3_init_iova_domain(struct aipu_memory_manager *mm)
{
	int asid;
	u64 total_iova_size;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return -EINVAL;

	/*
	 * Per-ASID layout: 4GB slot, of which 3GB is usable (cacheable region)
	 * and the top 1GB is reserved as never-map (non-cacheable/core/cluster).
	 * The 1GB never-map zone also serves as a barrier preventing alloc_iova
	 * (which has no lower-bound parameter) from leaking into adjacent ASIDs.
	 *
	 * ASID 0: IOVA [0,    3GB) usable + [3GB,  4GB) never-map
	 * ASID 1: IOVA [4GB,  7GB) usable + [7GB,  8GB) never-map
	 * ASID 2: IOVA [8GB, 11GB) usable + [11GB,12GB) never-map
	 * ASID 3: IOVA [12GB,15GB) usable + [15GB,16GB) never-map
	 */
	#define V3_ASID_SLOT_SIZE	0x100000000ULL	/* 4GB per ASID slot */
	#define V3_ASID_USABLE_SIZE	0xC0000000ULL	/* 3GB usable per ASID */

	/* Initialize per-ASID IOVA info */
	for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
		mm->asid_iova[asid].iova_base = (u64)asid * V3_ASID_SLOT_SIZE;
		mm->asid_iova[asid].iova_size = V3_ASID_USABLE_SIZE;
		mm->asid_iova[asid].iova_used = 0;
		/* NPU view base: 0, 4GB, 8GB, 12GB for ASID 0-3 */
		mm->asid_iova[asid].asid_base = (u64)asid * V3_ASID_SLOT_SIZE;

		/* Set mm->ase[asid].base so aipu_mm_get_asid_base() returns correct value to UMD */
		mm->ase[asid].base = mm->asid_iova[asid].asid_base;
		mm->ase[asid].range = V3_ASID_USABLE_SIZE;

		dev_dbg(mm->dev, "V3 ASID %d: IOVA [0x%llx, 0x%llx), NPU base 0x%llx\n",
			asid, mm->asid_iova[asid].iova_base,
			mm->asid_iova[asid].iova_base + V3_ASID_USABLE_SIZE,
			mm->asid_iova[asid].asid_base);
	}

	/* Total IOVA space: 16GB for 4 ASIDs (4GB slots) */
	mm->iova_base = 0;
	mm->iova_size = (u64)ZHOUYI_ASID_COUNT * V3_ASID_SLOT_SIZE;
	total_iova_size = mm->iova_size;

	/* Initialize IOVA domain */
#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
	init_iova_domain(&mm->iova_domain, PAGE_SIZE, 0);
#else
	init_iova_domain(&mm->iova_domain, PAGE_SIZE, 0, total_iova_size >> PAGE_SHIFT);
#endif

	/*
	 * Reserve page 0 so IOVA 0x0 is never allocated.
	 * PA 0x0 (ASID 0 base + offset 0) would be indistinguishable from the
	 * error sentinel used by aipu_mm_v3_pa_to_iova() and callers.
	 */
	if (!reserve_iova(&mm->iova_domain, 0, 0))
		dev_warn(mm->dev, "V3: failed to reserve IOVA page 0\n");

	/* Reserve "never map" zones at each 3GB boundary to prevent wrap-around */
	/* These are the top 1GB of each 4GB window: [3GB,4GB), [7GB,8GB), etc. */
	if (aipu_mm_v3_reserve_never_map_zones(mm))
		dev_warn(mm->dev, "V3: failed to reserve some never-map zones\n");

	/* Initialize buffer management */
	mm->buffer_count = 0;
	mutex_init(&mm->buffer_mutex);
	spin_lock_init(&mm->asid_lock);
	INIT_LIST_HEAD(&mm->buffer_list);
	mm->host_aipu_offset = 0;
	/* 16GB total IOVA needs 34-bit DMA mask */
	mm->dma_mask = 34;
	init_cache_ops(mm);

	/* Initialize V3 TCB buffer tracking list */
	mm->v3_tcb_buf_head = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
	if (!mm->v3_tcb_buf_head)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->v3_tcb_buf_head->node);
	mm->v3_tcb_buf_head->reg = NULL;

	dev_info(mm->dev, "V3 IOVA domain: total 0x%llx (%d ASIDs x 4GB slots, 3GB usable)\n",
		 total_iova_size, ZHOUYI_ASID_COUNT);

	return 0;
}

/**
 * aipu_mm_v3_reserve_never_map_zones - Reserve zones that should never be mapped
 * @mm: memory manager
 *
 * Each ASID has a 4GB IOVA slot.  The top 1GB [3GB, 4GB) of each slot
 * corresponds to the NPU's non-cacheable / core / cluster address range
 * and must never be used for DMA.  Reserving it also acts as a barrier
 * that prevents alloc_iova (top-down, no lower-bound) from leaking into
 * the adjacent ASID's usable range.
 */
static int aipu_mm_v3_reserve_never_map_zones(struct aipu_memory_manager *mm)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct iova *iova;
	u64 never_map_start, never_map_end;
	int asid;
	int ret = 0;

	for (asid = 0; asid < ZHOUYI_ASID_COUNT; asid++) {
		/* Top 1GB of each 4GB slot: [asid*4GB + 3GB, asid*4GB + 4GB) */
		never_map_start = (u64)asid * V3_ASID_SLOT_SIZE + V3_ASID_USABLE_SIZE;
		never_map_end   = (u64)(asid + 1) * V3_ASID_SLOT_SIZE;

		iova = reserve_iova(iovad,
				    never_map_start >> PAGE_SHIFT,
				    (never_map_end - 1) >> PAGE_SHIFT);
		if (!iova) {
			dev_warn(mm->dev, "V3: failed to reserve never-map zone at 0x%llx\n",
				 never_map_start);
			ret = -ENOMEM;
		} else {
			dev_dbg(mm->dev, "V3: ASID %d never-map [0x%llx, 0x%llx)\n",
				asid, never_map_start, never_map_end);
		}
	}

	return ret;
}

/**
 * aipu_mm_v3_deinit_iova_domain - Cleanup V3 IOVA domain
 * @mm: memory manager
 */
void aipu_mm_v3_deinit_iova_domain(struct aipu_memory_manager *mm)
{
	struct aipu_tcb_buf *tbuf, *tmp;
	unsigned long flags;

	if (!mm || !mm->use_v3_custom_iova)
		return;

	/* Free V3 TCB buffer list */
	if (mm->v3_tcb_buf_head) {
		spin_lock_irqsave(&mm->slock, flags);
		list_for_each_entry_safe(tbuf, tmp, &mm->v3_tcb_buf_head->node, node) {
			list_del(&tbuf->node);
			kmem_cache_free(mm->tbuf_cache, tbuf);
		}
		spin_unlock_irqrestore(&mm->slock, flags);
		kmem_cache_free(mm->tbuf_cache, mm->v3_tcb_buf_head);
		mm->v3_tcb_buf_head = NULL;
	}

	put_iova_domain(&mm->iova_domain);
	mm->use_v3_custom_iova = false;
}


/*
 * aipu_mm_v3_dump_asid_state - dump per-ASID accounting summary.
 *
 * Diagnostic only, called when the IOVA-counter check rejects an
 * allocation. Walks mm->buffer_list and compares the live in-list
 * buffers belonging to @asid (count + summed iova_size) against
 * asid_iova[asid].iova_used and mm->buffer_count. If counter > sum,
 * the leak is on the counter side (a free path decremented the list
 * but not the counter, or never ran at all). If counter ~= sum, the
 * buffers are genuinely still on the list because some fd never
 * closed (or its release path skipped cleanup).
 *
 * Tgid breakdown (up to 16 distinct PIDs) is printed so it is obvious
 * which user-space process owns the surviving buffers.
 */
static void __maybe_unused
aipu_mm_v3_dump_asid_state(struct aipu_memory_manager *mm, u32 asid)
{
	struct aipu_iova_buffer *buffer;
	struct { int tgid; int count; u64 total; } tg[16];
	int n_tg = 0, i;
	int list_total = 0, asid_count = 0;
	u64 asid_total = 0;

	if (!mm || !mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return;

	memset(tg, 0, sizeof(tg));

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		list_total++;
		if (buffer->asid != asid)
			continue;
		asid_count++;
		asid_total += buffer->iova_size;

		for (i = 0; i < n_tg; i++)
			if (tg[i].tgid == buffer->tgid)
				break;
		if (i == n_tg && n_tg < (int)ARRAY_SIZE(tg)) {
			tg[n_tg].tgid = buffer->tgid;
			n_tg++;
		}
		if (i < (int)ARRAY_SIZE(tg)) {
			tg[i].count++;
			tg[i].total += buffer->iova_size;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	dev_info(mm->dev,
		 "V3 ASID %u accounting dump: counter=0x%llx, list=%d buffers on this ASID totalling 0x%llx (mm->buffer_count=%d, list_walk=%d)\n",
		 asid, mm->asid_iova[asid].iova_used,
		 asid_count, asid_total, mm->buffer_count, list_total);

	for (i = 0; i < n_tg; i++)
		dev_info(mm->dev,
			 "  tgid=%d: %d buffers, total 0x%llx\n",
			 tg[i].tgid, tg[i].count, tg[i].total);
}

/**
 * aipu_mm_v3_alloc_iova - Allocate IOVA in specific ASID range
 * @mm: memory manager
 * @buf_req: buffer request (asid, bytes, align_in_page)
 * @filp: file pointer
 * 
 * Return: allocated buffer or NULL on failure
 */
struct aipu_iova_buffer *aipu_mm_v3_alloc_iova(struct aipu_memory_manager *mm,
						struct aipu_buf_request *buf_req,
						struct file *filp)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct aipu_iova_buffer *buffer = NULL;
	struct aipu_asid_iova_info *asid_info;
	struct iova *iova;
	u64 asid_iova_base, asid_iova_size;
	u64 iova_size;
	unsigned long pfn_lo, pfn_hi;
	u32 asid = buf_req->asid;

	if (asid >= ZHOUYI_ASID_COUNT) {
		dev_err(mm->dev, "V3: invalid ASID %d\n", asid);
		return NULL;
	}

	asid_info = &mm->asid_iova[asid];
	asid_iova_base = asid_info->iova_base;
	asid_iova_size = asid_info->iova_size;

	/*
	 * Atomically check and reserve the accounting quota. iova_used is a
	 * process-wide counter shared by every fd that maps this ASID; the
	 * check-then-increment must be a single critical section, otherwise
	 * concurrent allocers/freers race on a non-atomic read-modify-write,
	 * lost updates accumulate, and the counter drifts upward until this
	 * gate rejects allocations even though the IOVA domain still has room
	 * ("V3: ASID N out of IOVA space" under multi-process stress).
	 *
	 * Reserve up front, then roll back on any failure below. The actual
	 * (sleeping) alloc_iova/kzalloc run outside the spinlock.
	 */
	iova_size = ALIGN(buf_req->bytes, PAGE_SIZE);
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used + iova_size > asid_iova_size) {
		spin_unlock(&mm->asid_lock);
		dev_err(mm->dev, "V3: ASID %d out of IOVA space (used 0x%llx, need 0x%llx)\n",
			asid, asid_info->iova_used, iova_size);
		/*
		 * Diagnostic dump kept available but disabled by default.
		 * Re-enable if "out of IOVA space" reappears, to tell a
		 * counter-accounting drift apart from a real per-fd leak:
		 *   aipu_mm_v3_dump_asid_state(mm, asid);
		 */
		return NULL;
	}
	asid_info->iova_used += iova_size;
	spin_unlock(&mm->asid_lock);

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		goto err_unreserve;

	/*
	 * Calculate pfn range for this ASID's usable region.
	 * The never-map zone at [asid_base + 3GB, asid_base + 4GB) acts as
	 * a hard barrier so alloc_iova cannot leak into the next ASID.
	 */
	pfn_lo = asid_iova_base >> PAGE_SHIFT;
	pfn_hi = ((asid_iova_base + asid_iova_size) >> PAGE_SHIFT) - 1;
	
	dev_dbg(mm->dev, "V3: alloc_iova ASID %d, pfn_lo=0x%lx, pfn_hi=0x%lx, size=0x%llx\n",
		asid, pfn_lo, pfn_hi, (u64)(iova_size >> PAGE_SHIFT));

	/*
	 * Allocate IOVA from domain.
	 * size_aligned=false: large NPU buffers only need PAGE_SIZE alignment.
	 * With size_aligned=true, the start address would be rounded to the next
	 * power-of-2 >= size, wasting huge amounts of IOVA space (e.g. a 1.5GB
	 * alloc would require a 2GB-aligned start, failing in a 3GB range).
	 */
	iova = alloc_iova(iovad, iova_size >> PAGE_SHIFT, pfn_hi, false);
	if (!iova) {
		dev_err(mm->dev, "V3: alloc_iova failed for ASID %d, size 0x%llx\n",
			asid, iova_size);
		goto err_free_buffer;
	}

	/* Verify IOVA is within ASID range (pfn_hi is now inclusive) */
	if (iova->pfn_lo < pfn_lo || iova->pfn_hi > pfn_hi) {
		dev_err(mm->dev, "V3: IOVA 0x%lx-0x%lx outside ASID %d range [0x%lx, 0x%lx]\n",
			iova->pfn_lo, iova->pfn_hi, asid, pfn_lo, pfn_hi);
		__free_iova(iovad, iova);
		goto err_free_buffer;
	}

	/* Initialize buffer structure */
	buffer->iova_start = (u64)iova->pfn_lo << PAGE_SHIFT;
	buffer->iova_size = iova_size;
	buffer->allocated_size = 0;
	buffer->region = buf_req->region;
	buffer->asid = asid;
	buffer->data_type = buf_req->data_type;
	buffer->ref_count = 1;
	/*
	 * [audit F-01] filp==NULL is reserved for the hold-TCB allocation path
	 * (aipu_mm_hold_tcb_buf_alloc -> aipu_mm_alloc(..., NULL) with
	 * data_type == AIPU_MM_DATA_TYPE_TCB). Such a buffer is intentionally
	 * cross-fd: the exit-encode tail is shared by every job's TCB chain,
	 * so it must outlive any single fd. It is reclaimed at module deinit
	 * by aipu_mm_hold_tcb_buf_free, NOT by aipu_mm_v3_free_buffers_by_filp
	 * (which can never match a NULL filp).
	 *
	 * Any NEW caller passing filp==NULL with non-TCB data_type would
	 * silently leak iova_used for the module lifetime — guard against
	 * that here so the leak is observable in dmesg.
	 */
	WARN_ONCE(!filp && buf_req->data_type != AIPU_MM_DATA_TYPE_TCB,
		"aipu_mm: V3 IOVA alloc filp==NULL with data_type=%d (expected %d=TCB) "
		"asid=%d size=0x%llx — caller will leak iova_used until module deinit\n",
		buf_req->data_type, AIPU_MM_DATA_TYPE_TCB, asid, iova_size);
	buffer->filp = filp;
	buffer->tgid = task_tgid_nr(current);
	buffer->pid = task_pid_nr(current);
	buffer->exec_id = buf_req->exec_id;
	buffer->data_type = buf_req->data_type;
	INIT_LIST_HEAD(&buffer->phy_blocks);
	mutex_init(&buffer->phy_mutex);

	/* Return IOVA as pa (this will be adjusted later if needed) */
	buf_req->desc.pa = buffer->iova_start;
	buf_req->desc.dev_offset = buffer->iova_start;
	buf_req->desc.bytes = iova_size;
	buf_req->desc.asid = asid;
	buf_req->desc.region = AIPU_BUF_REGION_DEFAULT;

	/* Add to buffer list */
	mutex_lock(&mm->buffer_mutex);
	list_add_tail(&buffer->node, &mm->buffer_list);
	mm->buffer_count++;
	mutex_unlock(&mm->buffer_mutex);

	dev_dbg(mm->dev, "V3: allocated IOVA 0x%llx size 0x%llx for ASID %d\n",
		buffer->iova_start, iova_size, asid);

	v3dbg(mm, "ALLOC_IOVA  iova[0x%llx-0x%llx) size=0x%llx asid=%u dtype=%s(%u) exec=0x%llx tgid=%d pid=%d\n",
	      buffer->iova_start, buffer->iova_start + buffer->iova_size,
	      buffer->iova_size, asid, aipu_mm_v3_dtype_str(buffer->data_type),
	      buffer->data_type, buffer->exec_id, buffer->tgid, buffer->pid);

	return buffer;

err_free_buffer:
	kfree(buffer);
err_unreserve:
	/* Release the quota reserved above so a failed alloc never leaks it. */
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used >= iova_size)
		asid_info->iova_used -= iova_size;
	else
		asid_info->iova_used = 0;
	spin_unlock(&mm->asid_lock);
	return NULL;
}

/**
 * aipu_mm_v3_flush_block_pages_nb - clean+invalidate every page of a block
 * @mm: memory manager
 * @block: physical block whose backing pages are flushed
 *
 * Flushes on the cacheable linear alias (page_address) WITHOUT issuing a
 * completion barrier. The caller MUST issue exactly one barrier
 * (cache_ops.flush_barrier() or dsb(sy)) after the last block it flushes. This
 * replaces the old per-page flush_range(), which emitted a full-system dsb
 * per 4KB page -- thousands of barriers when flushing a large working set.
 *
 * Falls back to the barrier-carrying flush_range on arches that did not opt
 * in to flush_range_nb (behaviour then matches the original code).
 */
static void aipu_mm_v3_flush_block_pages_nb(struct aipu_memory_manager *mm,
					    struct aipu_phy_block *block)
{
	void (*flush)(void *, size_t) = mm->cache_ops.flush_range_nb ?
					mm->cache_ops.flush_range_nb :
					mm->cache_ops.flush_range;
	int i;

	if (!flush)
		return;

	for (i = 0; i < block->page_count; i++) {
		void *kaddr = page_address(block->pages[i]);

		if (kaddr)
			flush(kaddr, PAGE_SIZE);
	}
}

/**
 * aipu_mm_v3_alloc_phy_and_map - Allocate physical pages and map to IOVA
 * @mm: memory manager
 * @buffer: IOVA buffer
 * @buf_req: buffer request for size info
 *
 * Return: 0 on success, error code otherwise
 */
int aipu_mm_v3_alloc_phy_and_map(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer,
					struct aipu_buf_request *buf_req)
{
	struct aipu_phy_block *block;
	u32 page_count;
	u64 iova_addr;
	int ret;

	if (!mm || !buffer || !buf_req)
		return -EINVAL;

	iova_addr = buffer->iova_start + buffer->allocated_size;
	page_count = buf_req->bytes >> PAGE_SHIFT;
	if (buf_req->bytes & (PAGE_SIZE - 1))
		page_count++;

	dev_dbg(mm->dev, "V3: alloc_phy_and_map iova 0x%llx, allocated 0x%llx\n",
		iova_addr, buffer->allocated_size);

	/* Check space in buffer */
	if (buffer->allocated_size + (page_count << PAGE_SHIFT) > buffer->iova_size) {
		dev_err(mm->dev, "V3: IOVA buffer full (allocated 0x%llx, need %d pages)\n",
			buffer->allocated_size, page_count);
		return -ENOMEM;
	}

	/* Allocate physical pages */
	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return -ENOMEM;

	block->pages = aipu_dma_alloc_pages(mm->dev, page_count,
					    PAGE_SIZE >> PAGE_SHIFT,
					    GFP_KERNEL | __GFP_ZERO);
	if (!block->pages) {
		dev_err(mm->dev, "V3: failed to allocate %d physical pages\n", page_count);
		ret = -ENOMEM;
		goto err_free_block;
	}

	block->size = (u64)page_count << PAGE_SHIFT;
	block->page_count = page_count;
	block->dmabuf = NULL;
	block->bind_memory = false;
	INIT_LIST_HEAD(&block->list);

	/* Build scatter-gather table and map to IOMMU */
	ret = sg_alloc_table_from_pages(&block->sgt, block->pages,
					block->page_count, 0, block->size,
					GFP_KERNEL);
	if (ret) {
		dev_err(mm->dev, "V3: failed to allocate SG table\n");
		goto err_free_pages;
	}

	/* Check if already mapped (should not happen) */
	if (iommu_iova_to_phys(mm->iommu_domain, iova_addr)) {
		dev_err(mm->dev, "V3: IOVA 0x%llx already mapped!\n", iova_addr);
		/* Try to unmap first */
		iommu_unmap(mm->iommu_domain, iova_addr, block->size);
	}

	/* Map to IOMMU at specific IOVA */
	{
		ssize_t mapped = iommu_map_sg(mm->iommu_domain, iova_addr, block->sgt.sgl,
				      block->sgt.nents, IOMMU_READ | IOMMU_WRITE,
				      GFP_KERNEL);
		if (mapped <= 0) {
			dev_err(mm->dev, "V3: IOMMU map failed at IOVA 0x%llx (ret=%zd)\n",
				iova_addr, mapped);
			ret = -ENOMEM;
			goto err_free_sg;
		}
		v3dbg(mm, "MAP         iova[0x%llx-0x%llx) size=0x%llx mapped=0x%zx%s pages=%u tgid=%d\n",
		      iova_addr, iova_addr + block->size, block->size, (size_t)mapped,
		      (mapped != (ssize_t)block->size) ? " PARTIAL!" : "",
		      page_count, task_tgid_nr(current));
	}

	block->iova_start = iova_addr;
	block->dma_addr = iova_addr;

	/* Map kernel virtual address for cache operations */
	/* Use noncached mapping for DMA coherency with NPU hardware */
	block->va = vmap(block->pages, page_count, VM_MAP,
			 pgprot_noncached(PAGE_KERNEL));
	if (!block->va) {
		/* [audit F-13] vmap failure must propagate as -ENOMEM.
		 * If a hold-TCB-class buffer ends up with kva == NULL,
		 * a subsequent aipu_mm_link_tcb dereferences kva under
		 * mm->shlock and OOPSes the kernel with a spinlock held.
		 * Unwind the IOMMU map, SG table, pages, and block, then
		 * return -ENOMEM so the caller's existing rollback runs.
		 */
		dev_err(mm->dev,
			"V3: vmap failed for IOVA 0x%llx size 0x%llx\n",
			iova_addr, block->size);
		ret = -ENOMEM;
		goto err_iommu_unmap;
	}

	/* Add block to buffer */
	mutex_lock(&buffer->phy_mutex);
	buffer->ref_count++;
	buffer->allocated_size += block->size;
	list_add_tail(&block->list, &buffer->phy_blocks);
	mutex_unlock(&buffer->phy_mutex);

	/* Flush (clean+invalidate) cache for the new pages using linear mapping.
	 * __GFP_ZERO fills pages via Normal Cacheable mapping, leaving dirty cache
	 * lines.  On ARM64, DC operations on Device-mapped VA are CONSTRAINED
	 * UNPREDICTABLE, so page_address() (Normal Cacheable) must be used.
	 * Without this flush, dirty cache lines can be evicted after NPU DMA writes,
	 * overwriting NPU output data in DRAM ("cache stomp").
	 *
	 * Use the no-barrier per-page flush and a single trailing barrier rather
	 * than a full-system dsb per page.
	 */
	aipu_mm_v3_flush_block_pages_nb(mm, block);
	if (mm->cache_ops.flush_barrier)
		mm->cache_ops.flush_barrier();

	dev_dbg(mm->dev, "V3: mapped IOVA 0x%llx to %d phys pages\n",
		iova_addr, page_count);

	return 0;

err_iommu_unmap:
	iommu_unmap(mm->iommu_domain, iova_addr, block->size);
err_free_sg:
	sg_free_table(&block->sgt);
err_free_pages:
	aipu_dma_free_pages(block->pages, block->page_count);
err_free_block:
	kfree(block);
	return ret;
}


/**
 * aipu_mm_v3_find_block_by_iova - Find physical block by IOVA address
 * @mm: memory manager
 * @iova: IOVA address
 * @asid: ASID (for validation)
 * 
 * Return: block pointer or NULL
 */
struct aipu_phy_block *aipu_mm_v3_find_block_by_iova(struct aipu_memory_manager *mm,
							      u64 iova, u32 asid)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;

	if (!mm || !mm->use_v3_custom_iova)
		return NULL;

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (buffer->asid != asid)
			continue;
		
		if (iova < buffer->iova_start ||
		    iova >= buffer->iova_start + buffer->iova_size)
			continue;

		mutex_lock(&buffer->phy_mutex);
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			if (iova >= block->iova_start &&
			    iova < block->iova_start + block->size) {
				mutex_unlock(&buffer->phy_mutex);
				mutex_unlock(&mm->buffer_mutex);
				return block;
			}
		}
		mutex_unlock(&buffer->phy_mutex);
	}
	mutex_unlock(&mm->buffer_mutex);

	return NULL;
}

/**
 * aipu_mm_v3_buffer_npu_writes - true if NPU may have DMA-written this buffer
 * @data_type: the enum aipu_mm_data_type stored at allocation time
 *
 * Post-job we only need to "dc civac" buffers the NPU has actually written:
 * the goal is to drop any speculative Normal-Cacheable lines so the CPU
 * sees the NPU's DMA writes. Buffers the NPU only reads (program text,
 * read-only parameters, static weights) already saw a clean+invalidate at
 * allocation time (alloc_phy_and_map), are never overwritten by the CPU
 * through the cacheable alias afterwards, and the NPU does not modify
 * their contents -- so flushing them every inference is pure waste.
 *
 * Conservative side (return true):
 *   REUSE -- feature maps / activations / outputs: written by NPU.
 *   STACK -- scratch the NPU writes during execution.
 *   TCB   -- command descriptors; NPU writes job status fields.
 *   NONE  -- unclassified caller (e.g. legacy paths); flush to stay safe.
 * Skipped (return false):
 *   TEXT, RODATA, STATIC, WEIGHT -- NPU-read-only by construction.
 */
static bool aipu_mm_v3_buffer_npu_writes(u8 data_type)
{
	switch (data_type) {
	case AIPU_MM_DATA_TYPE_TEXT:
	case AIPU_MM_DATA_TYPE_RODATA:
	case AIPU_MM_DATA_TYPE_STATIC:
	case AIPU_MM_DATA_TYPE_WEIGHT:
		return false;
	default:
		return true;
	}
}

/**
 * aipu_mm_v3_flush_all_for_filp - Flush all V3 custom IOVA buffers for a filp
 * @mm: memory manager
 * @filp: file pointer identifying the process
 *
 * Ensures cache coherency for all DMA buffers belonging to this filp.
 * Called after NPU job completion to ensure CPU sees NPU DMA writes.
 * Uses page_address() (Normal Cacheable linear mapping) for cache maintenance,
 * since DC operations on Device-mapped VA are CONSTRAINED UNPREDICTABLE on ARM64.
 *
 * Skips NPU-read-only buffer classes (text/rodata/static/weight). For a
 * typical inference those dominate working-set size; flushing them post-job
 * is wasted work -- the NPU never writes them, so there are no fresh DMA
 * writes that the CPU could miss, and they were already clean+invalidated
 * at allocation time when __GFP_ZERO dirtied them.
 */
void aipu_mm_v3_flush_all_for_filp(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_iova_buffer *buffer;
	struct aipu_phy_block *block;

	if (!mm || !filp || !mm->use_v3_custom_iova)
		return;

	dsb(sy);

	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (buffer->filp != filp)
			continue;
		if (!aipu_mm_v3_buffer_npu_writes(buffer->data_type))
			continue;

		mutex_lock(&buffer->phy_mutex);
		list_for_each_entry(block, &buffer->phy_blocks, list) {
			/*
			 * No per-page barrier: the single trailing dsb(sy)
			 * below completes all the dc ops issued in this walk.
			 */
			aipu_mm_v3_flush_block_pages_nb(mm, block);
		}
		mutex_unlock(&buffer->phy_mutex);
	}
	mutex_unlock(&mm->buffer_mutex);

	dsb(sy);
}


/**
 * aipu_mm_v3_pa_to_iova - Convert device PA to IOVA
 * @mm: memory manager
 * @dev_pa: device physical address (asid_base + offset)
 * @asid: ASID
 * 
 * Return: IOVA address or 0 on error
 */
u64 aipu_mm_v3_pa_to_iova(struct aipu_memory_manager *mm, u64 dev_pa, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 asid_base;

	if (!mm || !mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	asid_info = &mm->asid_iova[asid];
	asid_base = asid_info->asid_base;

	/* Check if dev_pa is in valid range */
	if (dev_pa < asid_base ||
	    dev_pa >= asid_base + asid_info->iova_size) {
		return 0;
	}

	/* IOVA = offset = dev_pa - asid_base + iova_base */
	return (dev_pa - asid_base) + asid_info->iova_base;
}

/**
 * aipu_mm_v3_iova_to_pa - Convert IOVA to device PA
 * @mm: memory manager
 * @iova: IOVA address
 * @asid: ASID
 * 
 * Return: device PA or 0 on error
 */
u64 aipu_mm_v3_iova_to_pa(struct aipu_memory_manager *mm, u64 iova, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 offset;

	if (!mm || !mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	asid_info = &mm->asid_iova[asid];

	/* Check if IOVA is in valid range */
	if (iova < asid_info->iova_base ||
	    iova >= asid_info->iova_base + asid_info->iova_size) {
		return 0;
	}

	/* PA = asid_base + offset = asid_base + (iova - iova_base) */
	offset = iova - asid_info->iova_base;
	return asid_info->asid_base + offset;
}

/**
 * aipu_mm_v3_get_tcb_addr - Get TCB address (relative offset)
 * @mm: memory manager
 * @dev_pa: device physical address
 * @asid: ASID
 * 
 * Return: 32-bit relative offset for TCB
 */
u32 aipu_mm_v3_get_tcb_addr(struct aipu_memory_manager *mm, u64 dev_pa, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 offset;

	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	asid_info = &mm->asid_iova[asid];

	if (dev_pa < asid_info->asid_base) {
		dev_err(mm->dev, "V3: dev_pa 0x%llx < asid_base 0x%llx\n",
			dev_pa, asid_info->asid_base);
		return 0;
	}

	offset = dev_pa - asid_info->asid_base;

	/* Check offset is within 32-bit range */
	if (offset >= 0x100000000ULL) {
		dev_err(mm->dev, "V3: offset 0x%llx exceeds 32-bit\n", offset);
		return 0;
	}

	return lower_32_bits(offset);
}

/**
 * aipu_mm_v3_map_imported_sgt - Map an imported dma-buf into the V3 custom IOVA domain
 * @mm:       memory manager
 * @sgt:      scatter-gather table of the imported dma-buf (physical pages)
 * @asid:     target ASID (the default context uses ASID 0)
 * @out_pa:   [out] NPU-visible address (asid_base + offset) to hand back to UMD
 * @out_size: [out] page-aligned size that was mapped
 *
 * The standard DMA API (dma_buf_map_attachment/sg_dma_address) maps into the
 * kernel's default DMA-IOMMU cookie allocator, whose IOVAs do NOT lie inside a
 * V3 ASID's usable window once the custom IOVA domain is active. Route imported
 * dma-bufs through the same custom domain that normal NPU buffers use so the NPU
 * can translate them. Mirrors aipu_mm_v3_alloc_iova()/alloc_phy_and_map().
 *
 * Return: 0 on success, negative errno otherwise.
 */
int aipu_mm_v3_map_imported_sgt(struct aipu_memory_manager *mm, struct sg_table *sgt,
				u32 asid, u64 *out_pa, u64 *out_size)
{
	struct iova_domain *iovad = &mm->iova_domain;
	struct aipu_asid_iova_info *asid_info;
	struct scatterlist *sg;
	struct iova *iova;
	u64 total = 0, size, iova_addr;
	unsigned long pfn_lo, pfn_hi;
	ssize_t mapped;
	int i;

	if (!mm || !sgt || !out_pa || !out_size)
		return -EINVAL;

	if (!mm->use_v3_custom_iova || asid >= ZHOUYI_ASID_COUNT)
		return -EINVAL;

	asid_info = &mm->asid_iova[asid];

	/* Physical view: sum the original page-granular segment lengths. */
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i)
		total += sg->length;

	size = ALIGN(total, PAGE_SIZE);
	if (!size)
		return -EINVAL;

	/* Atomically check + reserve quota (see aipu_mm_v3_alloc_iova). */
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used + size > asid_info->iova_size) {
		spin_unlock(&mm->asid_lock);
		dev_err(mm->dev, "V3: ASID %d out of IOVA space for import (used 0x%llx, need 0x%llx)\n",
			asid, asid_info->iova_used, size);
		return -ENOMEM;
	}
	asid_info->iova_used += size;
	spin_unlock(&mm->asid_lock);

	pfn_lo = asid_info->iova_base >> PAGE_SHIFT;
	pfn_hi = ((asid_info->iova_base + asid_info->iova_size) >> PAGE_SHIFT) - 1;

	/* size_aligned=false: page alignment is enough (see aipu_mm_v3_alloc_iova). */
	iova = alloc_iova(iovad, size >> PAGE_SHIFT, pfn_hi, false);
	if (!iova) {
		dev_err(mm->dev, "V3: alloc_iova failed for import (ASID %d, size 0x%llx)\n",
			asid, size);
		goto err_unreserve;
	}

	if (iova->pfn_lo < pfn_lo || iova->pfn_hi > pfn_hi) {
		dev_err(mm->dev, "V3: import IOVA 0x%lx-0x%lx outside ASID %d range\n",
			iova->pfn_lo, iova->pfn_hi, asid);
		__free_iova(iovad, iova);
		goto err_unreserve;
	}

	iova_addr = (u64)iova->pfn_lo << PAGE_SHIFT;

	mapped = iommu_map_sg(mm->iommu_domain, iova_addr, sgt->sgl, sgt->orig_nents,
			      IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (mapped < (ssize_t)total) {
		dev_err(mm->dev, "V3: iommu_map_sg failed for import at IOVA 0x%llx (ret=%zd, need 0x%llx)\n",
			iova_addr, mapped, total);
		if (mapped > 0)
			iommu_unmap(mm->iommu_domain, iova_addr, mapped);
		__free_iova(iovad, iova);
		goto err_unreserve;
	}

	*out_pa = aipu_mm_v3_iova_to_pa(mm, iova_addr, asid);
	*out_size = size;

	dev_dbg(mm->dev, "V3: imported dma-buf mapped IOVA 0x%llx size 0x%llx ASID %d, pa 0x%llx\n",
		iova_addr, size, asid, *out_pa);

	return 0;

err_unreserve:
	/* Release the quota reserved above so a failed import never leaks it. */
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used >= size)
		asid_info->iova_used -= size;
	else
		asid_info->iova_used = 0;
	spin_unlock(&mm->asid_lock);
	return -ENOMEM;
}

/**
 * aipu_mm_v3_unmap_imported_sgt - Tear down a mapping made by aipu_mm_v3_map_imported_sgt
 * @mm:     memory manager
 * @dev_pa: the NPU pa (out_pa) returned by the map helper
 * @size:   the page-aligned size returned by the map helper
 * @asid:   ASID the import was mapped into
 *
 * Only unmaps/frees the custom-domain IOVA. The dma-buf attachment (and thus
 * the backing pages) is released by the caller via dma_buf_unmap_attachment().
 */
void aipu_mm_v3_unmap_imported_sgt(struct aipu_memory_manager *mm, u64 dev_pa,
				   u64 size, u32 asid)
{
	struct aipu_asid_iova_info *asid_info;
	u64 dev_iova;

	if (!mm || !mm->use_v3_custom_iova || !dev_pa || !size)
		return;
	if (asid >= ZHOUYI_ASID_COUNT)
		return;

	dev_iova = aipu_mm_v3_pa_to_iova(mm, dev_pa, asid);
	if (!dev_iova) {
		dev_warn(mm->dev, "V3: cannot unmap import, bad pa 0x%llx ASID %d\n",
			 dev_pa, asid);
		return;
	}

	iommu_unmap(mm->iommu_domain, dev_iova, size);
	free_iova(&mm->iova_domain, dev_iova >> PAGE_SHIFT);

	asid_info = &mm->asid_iova[asid];
	spin_lock(&mm->asid_lock);
	if (asid_info->iova_used >= size)
		asid_info->iova_used -= size;
	else
		asid_info->iova_used = 0;
	spin_unlock(&mm->asid_lock);

	dev_dbg(mm->dev, "V3: unmapped imported dma-buf pa 0x%llx (IOVA 0x%llx) size 0x%llx ASID %d\n",
		dev_pa, dev_iova, size, asid);
}

/**
 * aipu_mm_v3_destroy_buffer_now - Immediately tear down a V3 IOVA buffer.
 * @mm: memory manager
 * @buffer: buffer to destroy
 *
 * Unmaps from IOMMU, frees the IOVA span, frees backing pages, and removes the
 * buffer from buffer_list. Used by the alloc-failure error path (buffer never
 * became visible to the NPU) and by aipu_mm_v3_free_buffers_by_filp (fd close,
 * at which point no NPU work for this fd can still be in flight).
 */
void aipu_mm_v3_destroy_buffer_now(struct aipu_memory_manager *mm,
				   struct aipu_iova_buffer *buffer)
{
	struct aipu_phy_block *block, *tmp;
	struct aipu_asid_iova_info *asid_info;
	struct iova_domain *iovad = &mm->iova_domain;

	if (!mm || !buffer)
		return;

	/* Unmap and free all physical blocks */
	mutex_lock(&buffer->phy_mutex);
	list_for_each_entry_safe(block, tmp, &buffer->phy_blocks, list) {
		if (block->iova_start && block->size) {
			/* Unmap from IOMMU */
			size_t _un = iommu_unmap(mm->iommu_domain,
						 block->iova_start, block->size);
			(void)_un;

			v3dbg(mm, "UNMAP       iova[0x%llx-0x%llx) size=0x%llx unmapped=0x%zx%s asid=%u dtype=%s(%u) ref=%d exec=0x%llx tgid=%d\n",
			      block->iova_start, block->iova_start + block->size,
			      block->size, _un,
			      (_un != block->size) ? " SHORT!" : "",
			      buffer->asid, aipu_mm_v3_dtype_str(buffer->data_type),
			      buffer->data_type, buffer->ref_count, buffer->exec_id,
			      buffer->tgid);
		}

		if (block->va)
			vunmap(block->va);

		if (block->sgt.nents)
			sg_free_table(&block->sgt);

		if (block->pages) {
			if (!block->bind_memory)
				aipu_dma_free_pages(block->pages, block->page_count);
		}

		list_del(&block->list);
		kfree(block);
	}
	mutex_unlock(&buffer->phy_mutex);

	/* Update ASID used count */
	if (buffer->asid < ZHOUYI_ASID_COUNT) {
		asid_info = &mm->asid_iova[buffer->asid];
		spin_lock(&mm->asid_lock);
		if (asid_info->iova_used >= buffer->iova_size)
			asid_info->iova_used -= buffer->iova_size;
		else
			asid_info->iova_used = 0;
		spin_unlock(&mm->asid_lock);
	}

	/* Free IOVA from domain */
	free_iova(iovad, buffer->iova_start >> PAGE_SHIFT);

	/* Remove from buffer list */
	mutex_lock(&mm->buffer_mutex);
	list_del(&buffer->node);
	mm->buffer_count--;
	mutex_unlock(&mm->buffer_mutex);

	dev_dbg(mm->dev, "V3: freed buffer IOVA 0x%llx size 0x%llx ASID %d\n",
		buffer->iova_start, buffer->iova_size, buffer->asid);

	kfree(buffer);
}

/**
 * aipu_mm_v3_free_iova_buffer - Free a V3 IOVA buffer (immediate).
 * @mm: memory manager
 * @buffer: buffer to free
 *
 * Runtime free path (aipu_free_dma_iova_phy). Tears down immediately.
 *
 * The use-after-unmap (DPTSW-23453 SMMU event 0x10) is now fixed at the source
 * in the UMD: reuse/feature-map buffers are persisted at graph level (no
 * per-job free), so no NPU-walked state references a freed IOVA. The KMD no
 * longer needs to defer iommu_unmap. If the UMD persistence is ever disabled,
 * re-enable the size-gated defer (keep aipu_mm_v3_destroy_buffer_now) — see
 * git history for the deferred version.
 */
void aipu_mm_v3_free_iova_buffer(struct aipu_memory_manager *mm,
					struct aipu_iova_buffer *buffer)
{
	if (!mm || !buffer)
		return;

	aipu_mm_v3_destroy_buffer_now(mm, buffer);
}

/**
 * aipu_mm_v3_free_buffers_by_filp - Free all buffers belonging to a file
 * @mm: memory manager
 * @filp: file pointer
 *
 * Called when fd is closed
 *
 * [audit F-01] Buffers with buffer->filp == NULL (hold-TCB cross-fd
 * allocations from aipu_mm_hold_tcb_buf_alloc) are intentionally NOT
 * reclaimed here — they are module-lifetime and reclaimed in
 * aipu_mm_hold_tcb_buf_free at deinit. The filp != NULL guard at entry
 * and the buffer->filp == filp comparison both implicitly skip them;
 * the WARN_ONCE in aipu_mm_v3_alloc_iova catches any new caller that
 * accidentally enters this regime with a non-TCB data_type.
 */
void aipu_mm_v3_free_buffers_by_filp(struct aipu_memory_manager *mm,
					     struct file *filp)
{
	struct aipu_iova_buffer *buffer, *tmp;
	struct aipu_iova_buffer **buffers_to_free;
	int count = 0, i = 0;

	if (!mm || !mm->use_v3_custom_iova || !filp)
		return;

	/* First pass: count matching buffers */
	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry(buffer, &mm->buffer_list, node) {
		if (buffer->filp == filp)
			count++;
	}
	mutex_unlock(&mm->buffer_mutex);

	if (count == 0)
		return;

	/* Allocate temp array */
	buffers_to_free = kcalloc(count, sizeof(*buffers_to_free), GFP_KERNEL);
	if (!buffers_to_free)
		return;

	/* Second pass: collect buffers */
	mutex_lock(&mm->buffer_mutex);
	list_for_each_entry_safe(buffer, tmp, &mm->buffer_list, node) {
		if (buffer->filp == filp && i < count) {
			buffers_to_free[i] = buffer;
			i++;
		}
	}
	mutex_unlock(&mm->buffer_mutex);

	/* Free them.
	 * At fd close no NPU work for this fd can still be in flight, so it is
	 * safe to actually tear down the mappings now. Deferred buffers (freed
	 * earlier via aipu_free_dma_iova_phy) and any still-live buffers both
	 * get destroyed immediately here. */
	for (i = 0; i < count; i++) {
		if (buffers_to_free[i])
			aipu_mm_v3_destroy_buffer_now(mm, buffers_to_free[i]);
	}

	kfree(buffers_to_free);
	dev_dbg(mm->dev, "V3: freed %d buffers for filp\n", count);
}
