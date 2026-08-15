// SPDX-License-Identifier: GPL-2.0
/*
 * xaga-kmsg2usb - gate the g_serial (ttyGS0 console) USB gadget on the
 * "kmsg2usb" marker stored at the head of the oops partition.
 *
 * xaga (Redmi Note 11T Pro / POCO X4 GT / Redmi K50i, MT6895) 6.12 recovery
 * bring-up: the g_serial console (CONFIG_USB_G_SERIAL + CONFIG_U_SERIAL_CONSOLE)
 * mirrors every printk() to ttyGS0 so a boot can be watched from the host via
 * screen /dev/ttyACM0 115200. Always-on it also claims the USB gadget
 * (mtu3 UDC) which the phone otherwise needs for charging/adb signalling.
 *
 * This driver gates the gadget on a marker in the oops partition
 * (/dev/block/sdc81, /dev/block/by-name/oops):
 *   - partition head == "kmsg2usb"  -> keep g_serial up (log capture mode)
 *   - otherwise                     -> usb_composite_unregister() the g_serial
 *                                      gadget, freeing the UDC for normal use
 *   - /proc/kmsg2usb (write 1/0)    -> toggle the marker (and apply it live)
 *   - /proc/kmsg2usb (read)         -> '1' (g_serial on) or '0' (off)
 *
 * The oops partition is opened exactly like the former xaga-dumpregs driver:
 * filp_open("/dev/block/by-name/oops") / "/dev/block/sdc81" + I_BDEV, then
 * small polling bios (one page each) so the write also works from a panic
 * notifier. The gadget switch goes through the exported
 * switch_gserial_enable() from drivers/usb/gadget/legacy/serial.c.
 *
 * Built into the GKI kernel (CONFIG_XAGA_KMSG2USB, xaga defconfig fragment).
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/string.h>

#define XAGA_K2U_MAGIC		"kmsg2usb"
#define XAGA_K2U_MAGIC_LEN	8	/* strlen("kmsg2usb") */
#define XAGA_K2U_CLEAR		"endfield-industry"
#define XAGA_K2U_CLEAR_LEN	16	/* strlen("endfield-industry") */
#define XAGA_K2U_SECTOR		SECTOR_SIZE	/* 512, partition head sector */

extern int switch_gserial_enable(bool do_enable);

static struct block_device *k2u_bdev;
static struct file *k2u_file;
static bool k2u_enabled;	/* current g_serial gadget state */
static bool k2u_marker;		/* current partition-head marker state */
static struct delayed_work k2u_open_work;

static int k2u_blk_write(loff_t pos, const void *buf, size_t len)
{
	loff_t p = pos;
	ssize_t n;

	if (!k2u_file)
		return -ENODEV;
	n = kernel_write(k2u_file, buf, len, &p);
	if (n < 0)
		pr_err("xaga-kmsg2usb: kernel_write fail: %zd\n", n);
	return n < 0 ? n : 0;
}

/* ---- read the marker from the partition head ---- */

static bool k2u_partition_has_marker(void)
{
	char *buf;
	bool hit = false;
	loff_t pos = 0;
	ssize_t n;

	if (!k2u_file)
		return false;
	/* kernel_read() on the already-opened filp: the open (filp_open) went
	 * through SELinux once and succeeded, so the standard block-device
	 * read path here is not re-audited; raw bios from kworker got avc
	 * denied on this device (denied { read write } for sdc81). */
	buf = kzalloc(XAGA_K2U_SECTOR, GFP_KERNEL);
	if (!buf)
		return false;
	n = kernel_read(k2u_file, buf, XAGA_K2U_SECTOR, &pos);
	if (n < 0) {
		pr_err("xaga-kmsg2usb: kernel_read failed: %zd\n", n);
		goto out;
	}
	if (n < XAGA_K2U_MAGIC_LEN) {
		pr_err("xaga-kmsg2usb: short read: %zd bytes (want %d)\n",
		       n, XAGA_K2U_MAGIC_LEN);
		goto out;
	}

	hit = memcmp(buf, XAGA_K2U_MAGIC, XAGA_K2U_MAGIC_LEN) == 0;
out:
	kfree(buf);
	return hit;
}

/* ---- marker write: "kmsg2usb" or "endfield-industry" at sector 0 ---- */

static int k2u_partition_write_marker(const char *tag, size_t len)
{
	char *buf;
	int ret;

	if (!k2u_bdev)
		return -ENODEV;
	/* kzalloc so virt_to_page() works in k2u_blk_write (linear map). */
	buf = kzalloc(XAGA_K2U_SECTOR, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memcpy(buf, tag, min(len, (size_t)XAGA_K2U_SECTOR));
	ret = k2u_blk_write(0, buf, XAGA_K2U_SECTOR);
	kfree(buf);
	return ret;
}

/* ---- apply the marker to the gadget state ---- */

static void k2u_apply(bool on)
{
	int ret;

	if (on == k2u_enabled)
		return;
	ret = switch_gserial_enable(on);
	if (ret)
		pr_err("xaga-kmsg2usb: switch_gserial_enable(%d) failed: %d\n",
		       on, ret);
	else
		k2u_enabled = on;
	pr_info("xaga-kmsg2usb: g_serial %s (marker=%s)\n",
		on ? "enabled" : "disabled", k2u_marker ? "kmsg2usb" : "off");
}

/* ---- proc handler ---- */

static ssize_t k2u_proc_write(struct file *file, const char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	char buf[16];
	ssize_t n;
	long val;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	n = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, ubuf, count);
	if (n < 0)
		return n;
	buf[n] = 0;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;

	if (val == 1) {
		ret = k2u_partition_write_marker(XAGA_K2U_MAGIC,
						XAGA_K2U_MAGIC_LEN);
		if (ret)
			return ret;
		k2u_marker = true;
		k2u_apply(true);
	} else if (val == 0) {
		ret = k2u_partition_write_marker(XAGA_K2U_CLEAR,
						XAGA_K2U_CLEAR_LEN);
		if (ret)
			return ret;
		k2u_marker = false;
		k2u_apply(false);
	} else {
		return -EINVAL;
	}
	return n;
}

static int k2u_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", k2u_enabled ? 1 : 0);
	return 0;
}

static int k2u_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, k2u_proc_show, NULL);
}

static const struct proc_ops k2u_proc_fops = {
	.proc_open	= k2u_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= k2u_proc_write,
};

/* ---- open the oops partition after UFS/devtmpfs are up ---- */

static struct block_device *k2u_try_open(void)
{
	static const char * const paths[] = {
		"/dev/block/by-name/oops",	/* init-created stable link */
		"/dev/block/sdc81",		/* devtmpfs raw node */
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
		pr_err("xaga-kmsg2usb: %s not a block device\n", paths[i]);
		filp_close(f, NULL);
		return NULL;
	}
	bdev = I_BDEV(inode);
	k2u_file = f;	/* keep open: I_BDEV stays valid */
	pr_info("xaga-kmsg2usb: oops partition %s ready\n", paths[i]);
	return bdev;
}

static void k2u_open_work_fn(struct work_struct *work)
{
	int retry = 0;

	k2u_bdev = k2u_try_open();
	if (!k2u_bdev) {
		if (++retry < 30) {
			schedule_delayed_work(&k2u_open_work, HZ / 5);
			return;
		}
		pr_err("xaga-kmsg2usb: cannot open oops partition after %d retries\n",
		       retry);
		return;
	}

	k2u_marker = k2u_partition_has_marker();
	pr_info("xaga-kmsg2usb: partition marker = %s\n",
		k2u_marker ? "kmsg2usb" : "off");

	/* Default: keep g_serial as-is only when the marker is present.
	 * When the marker is absent we tear the gadget down to free the
	 * UDC for charging/other uses. */
	if (k2u_marker) {
		k2u_enabled = true;
		k2u_apply(true);	/* no-op if already up */
	} else {
		k2u_enabled = true;	/* assume up until proven off */
		k2u_apply(false);
	}
}

static int __init xaga_kmsg2usb_init(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("kmsg2usb", 0644, NULL, &k2u_proc_fops);
	if (!pe) {
		pr_err("xaga-kmsg2usb: proc_create failed\n");
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&k2u_open_work, k2u_open_work_fn);
	schedule_delayed_work(&k2u_open_work, 2 * HZ);

	pr_info("xaga-kmsg2usb: g_serial gate ready (/proc/kmsg2usb)\n");
	return 0;
}
late_initcall(xaga_kmsg2usb_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("xaga g_serial gate on oops-partition kmsg2usb marker");
