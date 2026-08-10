// SPDX-License-Identifier: GPL-2.0
/*
 * xaga boot-stage marker writer (XAGR ring), built into the kernel.
 *
 * xaga (Redmi Note 11T Pro / POCO X4 GT / Redmi K50i, MT6895) boot trace.
 * Arms a 64KB "XAGR" header + circular text ring in the aee_lk reserved
 * DRAM region (0x50740000) at the head of arm64 setup_arch - the earliest
 * point the arm64 MMU fixmap makes the region writable - and mirrors every
 * printk() (via vprintk_emit) into the ring. Markers survive an AP watchdog
 * reboot in DRAM; the xaga-marker reader built into the lineage_xaga kernel
 * prints them on the next boot, so a boot hang can be located even when the
 * kernel dies before the ramoops console is up.
 *
 * Layout matches the reader (lineage_xaga drivers/misc/xaga-marker.c):
 *   u32 magic @0x0000, u32 cursor @0x0004, u32 total @0x0008,
 *   u32 stage @0x1000, text ring @0x2000 (0xE000 bytes).
 *
 * The ring lives in aee_lk (0x50700000, 8MB, ring at +4MB = 0x50740000),
 * NOT log_store (0x7ffbf000): LK's PL_LOG_STORE rewrites the log_store
 * header on every boot (ram_header->sig 0xABCD1234), wiping the ring before
 * the reader can see it. minirdump (0x48170000) triggers mrdump and reboots,
 * and ramoops owns pstore (0x48090000) - device findings 2026-08-09/10.
 * aee_lk is a non-secure reserved area untouched on a normal boot; +4MB
 * clears any LK aee header writes.
 *
 * Built-in (it was a module until the vendor-ramdisk module never wrote -
 * never confirmed loaded): CONFIG_XAGA_MARKER_WRITER is set only by the xaga
 * defconfig fragment; other OPPO devices leave it off. The module-load
 * notifier still logs every later module load, so a hang in a vendor module
 * probe leaves that module's name as the last ring entry.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/printk.h>
#include <linux/xaga_marker.h>

/* aee_lk reserved region: non-secure, survives the WDT reboot in DRAM.
 * Ring at aee_lk + 4MB (0x50740000) to clear LK aee header writes. */
#define XAGA_MRDUMP_PA	0x50740000UL
#define XAGA_MRDUMP_SZ	0x10000UL
#define XAGA_RING_OFF	0x2000U
#define XAGA_RING_SZ	0xE000U
#define XAGA_MAGIC	0x52474158UL	/* "XAGR" */
#define XAGA_MAX_MSG	256

static void __iomem *xaga_mr_base;

static void xaga_marker_ring_write(const char *buf, int n)
{
	void __iomem *ring;
	u32 cursor;
	int i;

	if (!xaga_mr_base)
		return;
	/* Re-assert our magic on every write: MTK aee/mrdump_mini may rewrite
	 * the region header; the next write restores it. */
	writel(XAGA_MAGIC, xaga_mr_base + 0x0000);
	cursor = readl(xaga_mr_base + 0x0004);
	ring = xaga_mr_base + XAGA_RING_OFF;
	for (i = 0; i < n; i++)
		writeb(buf[i], ring + ((cursor + i) % XAGA_RING_SZ));
	writel(cursor + n, xaga_mr_base + 0x0004);
	writel(readl(xaga_mr_base + 0x0008) + n, xaga_mr_base + 0x0008);
}

void xaga_marker_put(const char *fmt, ...)
{
	va_list args;
	char buf[XAGA_MAX_MSG];
	int n;

	va_start(args, fmt);
	n = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (n <= 0)
		return;
	xaga_marker_ring_write(buf, n);
}
EXPORT_SYMBOL_GPL(xaga_marker_put);

void xaga_marker_stage(u32 stage)
{
	if (!xaga_mr_base)
		return;
	writel(stage, xaga_mr_base + 0x1000);
	xaga_marker_put("stage=%u\n", stage);
}
EXPORT_SYMBOL_GPL(xaga_marker_stage);

/* Mirrors every printk() into the ring while armed; called from
 * vprintk_emit. Must be safe in any printk context: no printk, no locks, no
 * allocation. The ring is lock-free: concurrent writers may occasionally
 * interleave, acceptable for a diagnostic ring. */
void xaga_marker_early_printk(const char *fmt, va_list args)
{
	va_list ap;
	char buf[XAGA_MAX_MSG];
	int n;

	if (!xaga_mr_base)
		return;
	va_copy(ap, args);
	n = vscnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n <= 0)
		return;
	xaga_marker_ring_write(buf, n);
}

/* Called from the head of arm64 setup_arch, right after
 * early_fixmap_init()/early_ioremap_init() - the earliest point the arm64
 * MMU maps the reserved region (it is not in the linear map before
 * paging_init). Everything printed from here on lands in the ring. */
void __init xaga_marker_early_init(void)
{
	xaga_mr_base = early_ioremap(XAGA_MRDUMP_PA, XAGA_MRDUMP_SZ);
	if (!xaga_mr_base) {
		pr_info("xaga-marker-writer: early_ioremap 0x%08lx failed\n",
			XAGA_MRDUMP_PA);
		return;
	}
	/* fresh ring per boot: only the last boot's markers survive */
	writel(XAGA_MAGIC, xaga_mr_base + 0x0000);
	writel(0, xaga_mr_base + 0x0004);
	writel(0, xaga_mr_base + 0x0008);
	writel(0, xaga_mr_base + 0x1000);
	pr_info("xaga-marker-writer: XAGR ring armed at 0x%08lx\n",
		XAGA_MRDUMP_PA);
	xaga_marker_stage(1);
}

static int xaga_marker_module_nb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct module *mod = data;

	switch (action) {
	case MODULE_STATE_COMING:
	case MODULE_STATE_LIVE:
		xaga_marker_put("module: %s\n", mod->name);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block xaga_marker_nb = {
	.notifier_call = xaga_marker_module_nb,
};

static int __init xaga_marker_w_late_init(void)
{
	xaga_marker_put("marker writer built-in init\n");
	register_module_notifier(&xaga_marker_nb);
	return 0;
}
core_initcall(xaga_marker_w_late_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("xaga boot-stage marker writer (XAGR ring), built-in");
