// SPDX-License-Identifier: GPL-2.0
/*
 * xaga-dumpregs - dump xaga display engine registers to the "oops"
 * partition (/dev/block/sdc81) on kernel oops/panic.
 *
 * xaga (Redmi Note 11T Pro / POCO X4 GT / Redmi K50i, MT6895) is stuck at
 * recovery display bring-up on the 6.12 port (crtc gets no panel timing ->
 * master bind incomplete -> atomic oops in mtk_dsi_connector_duplicate_state).
 * This built-in driver mirrors the userspace dumpregs.c register reads (OVL,
 * RDMA, PQ chain, DSC, DSI, MUTEX, MIPI TX PLL, MMSYS crossbar, RSZ0) into
 * the oops partition so the display state at the crash moment can be pulled
 * back via `dd if=/dev/block/sdc81 bs=64 skip=1` after the reboot.
 *
 * Built into the GKI kernel (CONFIG_XAGA_DUMPREGS, set by the xaga defconfig
 * fragment) so it works even when first-stage init never runs (UFS is
 * built-in, /dev/block nodes come from devtmpfs). The block device is opened
 * by delayed work after UFS probes; the dump itself uses panic-safe polling
 * bios (no sleeping) so it also works from the panic notifier.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>
#include <linux/kdebug.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/kmsg_dump.h>

#define XAGA_DR_MAGIC	0x44414758	/* "XGAD" */
#define XAGA_DR_VERSION	1
#define XAGA_DR_HEADER	64
#define XAGA_DR_MAX	(16 * 1024)	/* 16 KiB register text cap */

#define DISP_PA		0x14000000UL
#define DISP_SZ		0x100000UL	/* mutex..dsi region */
#define PLL_PA		0x11f70000UL
#define PLL_SZ		0x1000UL

static void __iomem *disp_base;
static void __iomem *pll_base;
static struct block_device *dr_bdev;
static char *dr_buf;			/* header + register text */
static atomic_t dr_dumped = ATOMIC_INIT(0);
static struct delayed_work dr_open_work;
static raw_spinlock_t dr_lock = __RAW_SPIN_LOCK_UNLOCKED(dr_lock);

static u32 dr_rd(u64 pa)
{
	return readl(disp_base + (pa - DISP_PA));
}

static u32 dr_rd_pll(u64 pa)
{
	return readl(pll_base + (pa - PLL_PA));
}

static void dr_fmt(char **p, char *end, const char *fmt, ...)
{
	va_list args;
	int n;

	if (*p >= end)
		return;
	va_start(args, fmt);
	n = vscnprintf(*p, end - *p, fmt, args);
	va_end(args);
	*p += n;
}

/* Register dump, byte-identical to the userspace dumpregs.c reads. */
static void xaga_dr_dump_regs(char *buf, size_t size)
{
	char *p = buf;
	char *end = buf + size;
	static const struct { u32 base; const char *name; } ovl[] = {
		{0x14002000, "OVL0"}, {0x14003000, "OVL0_2L"},
		{0x14004000, "OVL1_2L"},
	};
	static const struct { u32 base; const char *name; } pq[] = {
		{0x14007000, "TDSHP0"}, {0x14008000, "C3D0"},
		{0x14009000, "COLOR0"}, {0x1400a000, "CCORR0"},
		{0x1400b000, "CCORR1"}, {0x1400c000, "DMDP_AAL0"},
		{0x1400d000, "AAL0"}, {0x1400e000, "GAMMA0"},
		{0x1400f000, "POSTMASK0"}, {0x14010000, "DITHER0"},
		{0x14013000, "CM0"}, {0x14014000, "SPR0"},
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(ovl); i++) {
		dr_fmt(&p, end, "--- %s ---\n", ovl[i].name);
		dr_fmt(&p, end,
		       "EN=0x%08x INTSTA=0x%08x ROI=0x%08x DATAPATH=0x%08x\n",
		       dr_rd(ovl[i].base + 0x0c), dr_rd(ovl[i].base + 0x08),
		       dr_rd(ovl[i].base + 0x20), dr_rd(ovl[i].base + 0x24));
		dr_fmt(&p, end, "SRC_CON=0x%08x CON0=0x%08x SRC_SIZE0=0x%08x\n",
		       dr_rd(ovl[i].base + 0x2c), dr_rd(ovl[i].base + 0x30),
		       dr_rd(ovl[i].base + 0x38));
		dr_fmt(&p, end, "PITCH0=0x%08x RDMA_CTRL0=0x%08x ADDR0=0x%08x\n",
		       dr_rd(ovl[i].base + 0x44), dr_rd(ovl[i].base + 0xc0),
		       dr_rd(ovl[i].base + 0xf40));
	}

	dr_fmt(&p, end, "--- RDMA0 ---\n");
	dr_fmt(&p, end, "GLOBAL=0x%08x SIZE0=0x%08x SIZE1=0x%08x FIFO=0x%08x\n",
	       dr_rd(0x14006010), dr_rd(0x14006014), dr_rd(0x14006018),
	       dr_rd(0x14006040));
	dr_fmt(&p, end, "INT_EN=0x%08x INT_STA=0x%08x\n",
	       dr_rd(0x14006000), dr_rd(0x14006004));

	dr_fmt(&p, end, "--- PQ CHAIN ---\n");
	for (i = 0; i < ARRAY_SIZE(pq); i++)
		dr_fmt(&p, end, "%-10s EN=0x%08x sh=0x%08x INTSTA=0x%08x\n",
		       pq[i].name, dr_rd(pq[i].base + 0x0),
		       dr_rd(pq[i].base + 0x100), dr_rd(pq[i].base + 0x8));

	dr_fmt(&p, end, "--- DSC ---\n");
	dr_fmt(&p, end, "CON=0x%08x INTSTA=0x%08x MODE=0x%08x ENC_W=0x%08x\n",
	       dr_rd(0x14015000), dr_rd(0x14015008), dr_rd(0x14015030),
	       dr_rd(0x1401503c));
	dr_fmt(&p, end, "SLICE_W=0x%08x SLICE_H=0x%08x CHUNK=0x%08x BUF=0x%08x SHADOW=0x%08x\n",
	       dr_rd(0x14015020), dr_rd(0x14015024), dr_rd(0x14015028),
	       dr_rd(0x1401502c), dr_rd(0x14015228));
	dr_fmt(&p, end, "PPS0=0x%08x PPS1=0x%08x PPS2=0x%08x\n",
	       dr_rd(0x14015080), dr_rd(0x14015084), dr_rd(0x14015088));

	dr_fmt(&p, end, "--- DSI ---\n");
	dr_fmt(&p, end,
	       "START=0x%08x CON=0x%08x MODE=0x%08x TXRX=0x%08x PSCTRL=0x%08x\n",
	       dr_rd(0x14017000), dr_rd(0x14017010), dr_rd(0x14017014),
	       dr_rd(0x14017018), dr_rd(0x1401701c));
	dr_fmt(&p, end, "SIZE_CON=0x%08x VM_CMD=0x%08x INTSTA=0x%08x\n",
	       dr_rd(0x14017038), dr_rd(0x14017200), dr_rd(0x1401700c));
	dr_fmt(&p, end, "VSA=0x%x VBP=0x%x VFP=0x%x VACT=0x%x\n",
	       dr_rd(0x14017020), dr_rd(0x14017024), dr_rd(0x14017028),
	       dr_rd(0x1401702c));
	dr_fmt(&p, end, "HSA=0x%x HBP=0x%x HFP=0x%x HSTX_CKL=0x%x\n",
	       dr_rd(0x14017050), dr_rd(0x14017054), dr_rd(0x14017058),
	       dr_rd(0x14017064));
	dr_fmt(&p, end, "LCCON=0x%08x LD0CON=0x%08x BUF_CON1=0x%08x\n",
	       dr_rd(0x14017104), dr_rd(0x14017108), dr_rd(0x14017404));
	dr_fmt(&p, end, "RW_TIMES=0x%08x SODI_HI=0x%08x SODI_LO=0x%08x\n",
	       dr_rd(0x14017410), dr_rd(0x14017414), dr_rd(0x14017418));
	dr_fmt(&p, end, "PREULTRA_HI=0x%08x ULTRA_HI=0x%08x URGENT_HI=0x%08x\n",
	       dr_rd(0x14017424), dr_rd(0x1401742c), dr_rd(0x14017434));

	dr_fmt(&p, end, "--- MUTEX ---\n");
	dr_fmt(&p, end, "EN=0x%08x SOF=0x%08x MOD0=0x%08x MOD1=0x%08x\n",
	       dr_rd(0x14001020), dr_rd(0x1400102c), dr_rd(0x14001030),
	       dr_rd(0x14001034));

	dr_fmt(&p, end, "--- MIPI TX PLL ---\n");
	dr_fmt(&p, end, "PLL_CON0=0x%08x PLL_CON1=0x%08x PLL_CON4=0x%08x LANE_CON=0x%08x\n",
	       dr_rd_pll(0x11f7002c), dr_rd_pll(0x11f70030),
	       dr_rd_pll(0x11f7003c), dr_rd_pll(0x11f70004));

	dr_fmt(&p, end, "--- MMSYS CROSSBAR 0xF00-0xFFF ---\n");
	for (i = 0; i < 0x100; i += 16)
		dr_fmt(&p, end, "%04x: %08x %08x %08x %08x\n", 0xf00 + i,
		       dr_rd(0x14000000 + 0xf00 + i + 0),
		       dr_rd(0x14000000 + 0xf00 + i + 4),
		       dr_rd(0x14000000 + 0xf00 + i + 8),
		       dr_rd(0x14000000 + 0xf00 + i + 0xc));

	dr_fmt(&p, end, "--- OVL DATAPATH_EXT + SYSRAM ---\n");
	dr_fmt(&p, end, "OVL0 DP_EXT=0x%08x OVL1_2L DP_EXT=0x%08x\n",
	       dr_rd(0x14002324), dr_rd(0x14004324));

	dr_fmt(&p, end, "--- RSZ0 (OVL1_2L->OVL0 scaler) ---\n");
	for (i = 0; i < 0x80; i += 4)
		dr_fmt(&p, end, "  +0x%03x: 0x%08x\n", i, dr_rd(0x14005000 + i));

	dr_fmt(&p, end, "--- DSC WRAP inputs ---\n");
	dr_fmt(&p, end, "PQ0_SOUT_SEL(FAC)=0x%08x DLI0_SOUT_SEL(FB4)=0x%08x DSC_WRAP_L_SEL=0x%08x DSC_WRAP_R_SEL=0x%08x\n",
	       dr_rd(0x14000fac), dr_rd(0x14000fb4), dr_rd(0x14000c00),
	       dr_rd(0x14000c04));
}

/* Panic-safe polling bio write (from the former xaga_oops_log module). */
struct dr_bio_done {
	struct completion done;
	int err;
};

#define DR_POLL_MAX	(100000000)

static void dr_bio_endio(struct bio *bio)
{
	struct dr_bio_done *bd = bio->bi_private;

	bd->err = blk_status_to_errno(bio->bi_status);
	complete(&bd->done);
	bio_put(bio);
}

static int dr_poll_done(struct dr_bio_done *bd)
{
	unsigned int spins = 0;

	while (!completion_done(&bd->done)) {
		if (++spins >= DR_POLL_MAX)
			return -ETIMEDOUT;
		cpu_relax();
	}
	return 0;
}

static int dr_blk_write(loff_t pos, const void *buf, size_t len)
{
	struct dr_bio_done bd;
	struct bio *bio = NULL;
	size_t written = 0;
	int ret = 0;

	while (written < len) {
		struct page *page = virt_to_page(buf + written);
		unsigned int off = offset_in_page(buf + written);
		size_t chunk = min(len - written, (size_t)(PAGE_SIZE - off));

		if (!bio) {
			init_completion(&bd.done);
			bd.err = 0;
			bio = bio_alloc(dr_bdev, 16, REQ_OP_WRITE,
					GFP_ATOMIC | __GFP_NOWARN);
			if (!bio) {
				ret = -ENOMEM;
				break;
			}
			bio->bi_iter.bi_sector = (pos + written) >> SECTOR_SHIFT;
			bio->bi_private = &bd;
			bio->bi_end_io = dr_bio_endio;
		}
		if (bio_add_page(bio, page, chunk, off) != chunk) {
			submit_bio(bio);
			bio = NULL;
			ret = dr_poll_done(&bd);
			if (!ret && bd.err)
				ret = bd.err;
			if (ret)
				break;
			continue;
		}
		written += chunk;
	}
	if (bio) {
		submit_bio(bio);
		ret = dr_poll_done(&bd);
		if (!ret && bd.err)
			ret = bd.err;
	}
	return ret;
}

/* Print the register text into the kernel log ring, which the marker-writer
 * mirrors into the XAGR ring (log_store 0x7ffbf000) -> expdb on the next
 * boot. This path needs NO block device, so it works even when the oops
 * partition node is not ready yet. 6.12 printk is a lockless ring buffer,
 * safe from die/panic context; the mirror hook runs before console output.
 * Chunks of ~400B keep it readable in the 56KB ring. */
static void xaga_dr_print_regs(const char *text, size_t len)
{
	size_t off = 0;
	int n = 0;

	while (off < len) {
		size_t chunk = min(len - off, (size_t)400);

		pr_info("xaga-dr[%d]: %.*s", n++, (int)chunk, text + off);
		off += chunk;
	}
}

/* Dump the full kernel log (dmesg) to the oops partition on oops/panic.
 * Triggered by kmsg_dump(KMSG_DUMP_OOPS) from oops_exit() (and by the
 * panic path) - this is exactly the "last_kmsg" we otherwise cannot get.
 * The die notifier's register text (xaga-dr[...]) is already in the log
 * ring by then, so it ends up in the dmesg copy as well. */
#define DR_KMSG_MAX	(1024 * 1024)	/* 1 MiB cap: oops partition is 16MB */

struct xaga_dr_header {
	__le32 magic;
	__le32 version;
	__le32 reason;
	__le32 len;
	__le64 ts_nsec;
	__le32 seq;
	__le32 reserved[9];
} __packed;

static struct kmsg_dumper dr_kmsg_dumper;
static atomic_t dr_kmsg_done = ATOMIC_INIT(0);
static char dr_kmsg_buf[4096];

static void dr_dump_kmsg(struct kmsg_dumper *dumper,
			 struct kmsg_dump_detail *detail)
{
	struct kmsg_dump_iter iter;
	loff_t pos = XAGA_DR_HEADER;
	size_t len;
	u32 total = 0;
	int ret;

	if (!dr_bdev || atomic_xchg(&dr_kmsg_done, 1))
		return;

	kmsg_dump_rewind(&iter);
	while (kmsg_dump_get_buffer(&iter, true, dr_kmsg_buf,
				    sizeof(dr_kmsg_buf), &len)) {
		if (len > DR_KMSG_MAX - total)
			len = DR_KMSG_MAX - total;
		ret = dr_blk_write(pos, dr_kmsg_buf, len);
		if (ret) {
			pr_err("xaga-dumpregs: kmsg write failed: %d (pos=%lld total=%u)\n",
			       ret, (long long)pos, total);
			break;
		}
		pos += len;
		total += len;
		if (total >= DR_KMSG_MAX)
			break;
	}

	/* 64B XGAD header at offset 0 (overwritten last; magic + len + why) */
	if (total) {
		struct xaga_dr_header *hdr =
			(struct xaga_dr_header *)dr_kmsg_buf;

		memset(hdr, 0, sizeof(*hdr));
		hdr->magic = cpu_to_le32(XAGA_DR_MAGIC);
		hdr->version = cpu_to_le32(XAGA_DR_VERSION);
		hdr->reason = cpu_to_le32(detail->reason);
		hdr->len = cpu_to_le32(total);
		hdr->ts_nsec = cpu_to_le64(ktime_get_real_fast_ns());
		hdr->seq = cpu_to_le32(1);
		dr_blk_write(0, dr_kmsg_buf, sizeof(*hdr));
		pr_info("xaga-dumpregs: saved %u bytes dmesg to oops partition (%s)\n",
			total, kmsg_dump_reason_str(detail->reason));
	} else {
		/* Distinguish "dumper never ran" from "log buffer empty". */
		pr_info("xaga-dumpregs: kmsg dump empty (%s), bdev=%px\n",
			kmsg_dump_reason_str(detail->reason), dr_bdev);
	}
}

/* Print the DSI0 child-node inventory (which panels exist in the live DT
 * and are available) into the XAGR ring. This settles whether the mipi-dsi
 * devices were ever created by mipi_dsi_host_register()'s
 * for_each_available_child_of_node() walk. Only called from the die
 * notifier (process context); of_* here takes RCU, not panic-safe. */
static void xaga_dr_of_status(void)
{
	struct device_node *np, *child;
	const char *compat;
	u32 reg;

	np = of_find_node_by_name(NULL, "dsi");
	if (!np) {
		pr_info("xaga-dr: dsi node NOT FOUND in DT\n");
		return;
	}
	pr_info("xaga-dr: dsi node %s %s\n", np->full_name,
		of_device_is_available(np) ? "available" : "disabled");
	for_each_available_child_of_node(np, child) {
		compat = of_get_property(child, "compatible", NULL);
		reg = 0xffffffff;
		of_property_read_u32(child, "reg", &reg);
		pr_info("xaga-dr:   child %s compatible=%s reg=%u\n",
			child->full_name, compat ? compat : "(none)", reg);
	}
}

static void xaga_dr_dump_and_save(const char *why)
{
	unsigned long flags;
	size_t len;

	/* Always log the driver state first: the XAGR ring keeps the last
	 * ~64KB of printk, so this single line lands in expdb on the next
	 * boot even when the dump is skipped below (NULL bdev/base/buf). */
	pr_info("xaga-dumpregs: %s: bdev=%px disp_base=%px pll_base=%px buf=%px\n",
		why, dr_bdev, disp_base, pll_base, dr_buf);

	if (!dr_buf || !disp_base)
		return;
	if (!raw_spin_trylock_irqsave(&dr_lock, flags))
		return;

	xaga_dr_dump_regs(dr_buf + XAGA_DR_HEADER, XAGA_DR_MAX);
	len = strnlen(dr_buf + XAGA_DR_HEADER, XAGA_DR_MAX);

	/* Register text goes into the kernel log ring only (mirrored into
	 * the XAGR ring -> expdb, and picked up by dr_dump_kmsg() below as
	 * part of the full dmesg written to the oops partition). */
	xaga_dr_print_regs(dr_buf + XAGA_DR_HEADER, len);

	raw_spin_unlock_irqrestore(&dr_lock, flags);
}

static int xaga_dr_die(struct notifier_block *nb, unsigned long val, void *data)
{
	if (val == DIE_OOPS && !atomic_xchg(&dr_dumped, 1)) {
		xaga_dr_of_status();	/* process ctx: RCU-safe of_* walk */
		xaga_dr_dump_and_save("oops");
		/* Trigger the kmsg dumper here (early, before mrdump floods the
		 * XAGR ring and before oops_exit() may be skipped) so its
		 * "saved N bytes dmesg" line lands in the ring and the dmesg
		 * write to the oops partition happens right at the oops. */
		kmsg_dump_desc(KMSG_DUMP_OOPS, "xaga-dumpregs");
	}
	return NOTIFY_OK;
}

static int xaga_dr_panic(struct notifier_block *nb, unsigned long val, void *data)
{
	if (!atomic_xchg(&dr_dumped, 1))
		xaga_dr_dump_and_save("panic");
	return NOTIFY_OK;
}

static struct notifier_block dr_die_nb = {
	.notifier_call = xaga_dr_die,
};

static struct notifier_block dr_panic_nb = {
	.notifier_call = xaga_dr_panic,
};

/* Open the oops partition once UFS (built-in) has probed and devtmpfs
 * (CONFIG_DEVTMPFS=y, enabled by the xaga fragment) has created
 * /dev/block/sdc81 - both happen without any userspace. Returns the bdev
 * on success, NULL on failure (never schedules, never sleeps long). */
static struct block_device *xaga_dr_try_open(void)
{
	static const char * const paths[] = {
		"/dev/block/by-name/oops",	/* stable label, init-created link */
		"/dev/block/sdc81",		/* devtmpfs raw node, no init */
	};
	struct file *f = NULL;
	struct inode *inode;
	struct block_device *bdev = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(paths); i++) {
		f = filp_open(paths[i], O_RDWR | O_DSYNC | O_NOATIME, 0);
		if (!IS_ERR(f))
			break;
	}
	if (IS_ERR(f))
		return NULL;
	inode = f->f_mapping->host;
	if (!S_ISBLK(inode->i_mode)) {
		pr_err("xaga-dumpregs: %s not a block device\n", paths[i]);
		filp_close(f, NULL);
		return NULL;
	}
	bdev = I_BDEV(inode);
	/* keep the file open: I_BDEV stays valid while the struct file lives */
	pr_info("xaga-dumpregs: oops partition %s ready\n", paths[i]);
	return bdev;
}

static void xaga_dr_open_work(struct work_struct *work)
{
	int retry = 0;

	dr_bdev = xaga_dr_try_open();
	if (dr_bdev)
		return;
	/* Log every retry: XAGR keeps only the last ~64KB (~1-2s), so the
	 * newest retry lines land in expdb on the next boot. Fast 200ms
	 * polling opens the partition as soon as UFS/devtmpfs create the
	 * node (empirically ~2.5-3s, first 2s attempt was -ENOENT). */
	pr_info("xaga-dumpregs: open oops partition retry err %d\n", -ENOENT);
	if (++retry < 30) {
		schedule_delayed_work(&dr_open_work, HZ / 5);
		return;
	}
	pr_err("xaga-dumpregs: cannot open oops partition after %d retries\n",
	       retry);
}

static int __init xaga_dumpregs_init(void)
{
	disp_base = ioremap(DISP_PA, DISP_SZ);
	pll_base = ioremap(PLL_PA, PLL_SZ);
	if (!disp_base || !pll_base) {
		/* Do NOT bail out: register the notifiers anyway so the
		 * state line in xaga_dr_dump_and_save() lands in the XAGR
		 * ring on the next oops and pinpoints the failure. */
		pr_err("xaga-dumpregs: ioremap failed (disp=%px pll=%px), "
		       "continuing for diagnostics\n", disp_base, pll_base);
	}
	dr_buf = kzalloc(XAGA_DR_HEADER + XAGA_DR_MAX + SECTOR_SIZE - 1,
			 GFP_KERNEL);
	if (!dr_buf)
		return -ENOMEM;

	register_die_notifier(&dr_die_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &dr_panic_nb);
	dr_kmsg_dumper.dump = dr_dump_kmsg;
	dr_kmsg_dumper.max_reason = KMSG_DUMP_OOPS;	/* oops + panic both */
	kmsg_dump_register(&dr_kmsg_dumper);
	INIT_DELAYED_WORK(&dr_open_work, xaga_dr_open_work);
	schedule_delayed_work(&dr_open_work, 2 * HZ);

	pr_info("xaga-dumpregs: dmesg -> oops partition on oops/panic\n");
	return 0;
}
late_initcall(xaga_dumpregs_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("xaga display register dump to oops partition");
