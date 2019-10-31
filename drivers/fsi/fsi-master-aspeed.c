// SPDX-License-Identifier: GPL-2.0+
// Copyright (C) IBM Corporation 2018
// FSI master driver for AST2600

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fsi.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/iopoll.h>

#include "fsi-master.h"

#define FSI_LINK_ENABLE_SETUP_TIME	10	/* in mS */

#define FSI_NUM_DEBUGFS_ENTRIES		17

#define DEFAULT_DIVISOR			14

struct fsi_master_aspeed;

struct fsi_master_aspeed_debugfs_entry {
	struct fsi_master_aspeed *aspeed;
	uint32_t addr;
};

struct fsi_master_aspeed {
	struct fsi_master	master;
	struct device		*dev;
	void __iomem		*base;
	struct clk		*clk;

	struct dentry		*debugfs_dir;
	struct fsi_master_aspeed_debugfs_entry debugfs[FSI_NUM_DEBUGFS_ENTRIES];
};

#define to_fsi_master_aspeed(m) \
	container_of(m, struct fsi_master_aspeed, master)

/* Control register (size 0x400) */
static const u32 ctrl_base = 0x80000000;

static const u32 fsi_base = 0xa0000000;

#define OPB_FSI_VER	0x00
#define OPB_TRIGGER	0x04
#define OPB_CTRL_BASE	0x08
#define OPB_FSI_BASE	0x0c
#define OPB_CLK_SYNC	0x3c
#define OPB_IRQ_CLEAR	0x40
#define OPB_IRQ_MASK	0x44
#define OPB_IRQ_STATUS	0x48

#define OPB0_SELECT	0x10
#define OPB0_RW		0x14
#define OPB0_XFER_SIZE	0x18
#define OPB0_FSI_ADDR	0x1c
#define OPB0_FSI_DATA_W	0x20
#define OPB0_STATUS	0x80
/* half world */
#define  STATUS_HW_ACK	BIT(0)
/* full word */
#define  STATUS_FW_ACK	BIT(1)
#define  STATUS_ERR_ACK	BIT(2)
#define OPB0_FSI_DATA_R	0x84

#define OPB0_W_ENDIAN	0x4c
#define OPB0_R_ENDIAN	0x5c

/* OPB_IRQ_MASK */
#define OPB1_XFER_ACK_EN BIT(17)
#define OPB0_XFER_ACK_EN BIT(16)

#define OPB_IRQ_CH0_DMA_EOT		BIT(0)
#define OPB_IRQ_CH1_DMA_EOT		BIT(1)
#define OPB_IRQ_CH2_DMA_EOT		BIT(2)
#define OPB_IRQ_CH3_DMA_EOT		BIT(3)
#define OPB_IRQ_CH0_DMA_FIFO_FULL	BIT(4)
#define OPB_IRQ_CH1_DMA_FIFO_FULL	BIT(5)
#define OPB_IRQ_CH2_DMA_FIFO_FULL	BIT(6)
#define OPB_IRQ_CH3_DMA_FIFO_FULL	BIT(7)
#define OPB_IRQ_CH0_DMA_FIFO_EMPTY	BIT(8)
#define OPB_IRQ_CH1_DMA_FIFO_EMPTY	BIT(9)
#define OPB_IRQ_CH2_DMA_FIFO_EMPTY	BIT(10)
#define OPB_IRQ_CH3_DMA_FIFO_EMPTY	BIT(11)
#define OPB_IRQ_CH0_DMA_TCONT_DONE	BIT(12)
#define OPB_IRQ_CH1_DMA_TCONT_DONE	BIT(13)
#define OPB_IRQ_CH2_DMA_TCONT_DONE	BIT(14)
#define OPB_IRQ_CH3_DMA_TCONT_DONE	BIT(15)
#define OPB_IRQ_OPB1_XFER_ACK		BIT(16)
#define OPB_IRQ_OPB0_XFER_ACK		BIT(17)
#define OPB_IRQ_SLAVE0			BIT(18)
#define OPB_IRQ_SLAVE1			BIT(19)
#define OPB_IRQ_SLAVE2			BIT(20)
#define OPB_IRQ_SLAVE3			BIT(21)
#define OPB_IRQ_SLAVE4			BIT(22)
#define OPB_IRQ_SLAVE5			BIT(23)
#define OPB_IRQ_SLAVE6			BIT(24)
#define OPB_IRQ_SLAVE7			BIT(25)
#define OPB_IRQ_ANY_HOTPLUG		BIT(26)
#define OPB_IRQ_ANY_PORT_ERROR		BIT(27)
#define OPB_IRQ_ANY_MST_ERROR		BIT(28)

/* OPB_RW */
#define CMD_READ	BIT(0)
#define CMD_WRITE	0

/* OPBx_XFER_SIZE */
#define XFER_WORD	(BIT(1) | BIT(0))
#define XFER_HALFWORD	(BIT(0))
#define XFER_BYTE	(0)

#define CREATE_TRACE_POINTS
#include <trace/events/fsi_master_aspeed.h>

static int __opb_write(struct fsi_master_aspeed *aspeed, u32 addr,
		       u32 val, u32 transfer_size)
{
	void __iomem *base = aspeed->base;
	u32 reg, status;
	int ret;

	writel(CMD_WRITE, base + OPB0_RW);
	writel(transfer_size, base + OPB0_XFER_SIZE);
	writel(addr, base + OPB0_FSI_ADDR);
	writel(val, base + OPB0_FSI_DATA_W);
	writel(0x1, base + OPB_IRQ_CLEAR);
	writel(0x1, base + OPB_TRIGGER);

	ret = readl_poll_timeout(base + OPB_IRQ_STATUS, reg,
				(reg & OPB0_XFER_ACK_EN) != 0,
				0, 10000);

	status = readl(base + OPB0_STATUS);

	trace_fsi_master_aspeed_opb_write(addr, val, transfer_size, status, reg);

	/* Return error when poll timed out */
	if (ret)
		return ret;

	/* Command failed, master will reset */
	if (status & STATUS_ERR_ACK)
		return -EIO;

	return 0;
}

static int opb_writeb(struct fsi_master_aspeed *aspeed, u32 addr, u8 val)
{
	return __opb_write(aspeed, addr, val, XFER_BYTE);
}

static int opb_writew(struct fsi_master_aspeed *aspeed, u32 addr, __be16 val)
{
	return __opb_write(aspeed, addr, (__force u16)val, XFER_HALFWORD);
}

static int opb_writel(struct fsi_master_aspeed *aspeed, u32 addr, __be32 val)
{
	return __opb_write(aspeed, addr, (__force u32)val, XFER_WORD);
}

static int __opb_read(struct fsi_master_aspeed *aspeed, uint32_t addr,
		      u32 transfer_size, void *out)
{
	void __iomem *base = aspeed->base;
	u32 result, reg;
	int status, ret;

	writel(CMD_READ, base + OPB0_RW);
	writel(transfer_size, base + OPB0_XFER_SIZE);
	writel(addr, base + OPB0_FSI_ADDR);
	writel(0x1, base + OPB_IRQ_CLEAR);
	writel(0x1, base + OPB_TRIGGER);

	ret = readl_poll_timeout(base + OPB_IRQ_STATUS, reg,
			   (reg & OPB0_XFER_ACK_EN) != 0,
			   0, 10000);

	status = readl(base + OPB0_STATUS);

	result = readl(base + OPB0_FSI_DATA_R);

	trace_fsi_master_aspeed_opb_read(addr, transfer_size, result,
			readl(base + OPB0_STATUS),
			reg);

	/* Return error when poll timed out */
	if (ret)
		return ret;

	/* Command failed, master will reset */
	if (status & STATUS_ERR_ACK)
		return -EIO;

	if (out) {
		switch (transfer_size) {
		case XFER_BYTE:
			*(u8 *)out = result;
			break;
		case XFER_HALFWORD:
			*(u16 *)out = result;
			break;
		case XFER_WORD:
			*(u32 *)out = result;
			break;
		default:
			return -EINVAL;
		}

	}

	return 0;
}

static int opb_readl(struct fsi_master_aspeed *aspeed, uint32_t addr, __be32 *out)
{
	return __opb_read(aspeed, addr, XFER_WORD, out);
}

static int opb_readw(struct fsi_master_aspeed *aspeed, uint32_t addr, __be16 *out)
{
	return __opb_read(aspeed, addr, XFER_HALFWORD, (void *)out);
}

static int opb_readb(struct fsi_master_aspeed *aspeed, uint32_t addr, u8 *out)
{
	return __opb_read(aspeed, addr, XFER_BYTE, (void *)out);
}

static int check_errors(struct fsi_master_aspeed *aspeed, int err)
{
	int ret;

	 if (trace_fsi_master_aspeed_opb_error_enabled()) {
		 __be32 mresp0, mstap0, mesrb0;

		 opb_readl(aspeed, ctrl_base + FSI_MRESP0, &mresp0);
		 opb_readl(aspeed, ctrl_base + FSI_MSTAP0, &mstap0);
		 opb_readl(aspeed, ctrl_base + FSI_MESRB0, &mesrb0);

		 trace_fsi_master_aspeed_opb_error(
				 be32_to_cpu(mresp0),
				 be32_to_cpu(mstap0),
				 be32_to_cpu(mesrb0));
	 };

	if (err == -EIO) {
		/* Check MAEB (0x70) ? */

		/* Then clear errors in master */
		ret = opb_writel(aspeed, ctrl_base + 0xd0,
				cpu_to_be32(0x20000000));
		if (ret) {
			/* TODO: log? return different code? */
			return ret;
		}
		/* TODO: confirm that 0x70 was okay */
	}

	/* This will pass through timeout errors */
	return err;
}

static int aspeed_master_read(struct fsi_master *master, int link,
			uint8_t id, uint32_t addr, void *val, size_t size)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int ret;

	if (id != 0)
		return -EINVAL;

	addr += link * FSI_HUB_LINK_SIZE;

	switch (size) {
	case 1:
		ret = opb_readb(aspeed, fsi_base + addr, val);
		break;
	case 2:
		ret = opb_readw(aspeed, fsi_base + addr, val);
		break;
	case 4:
		ret = opb_readl(aspeed, fsi_base + addr, val);
		break;
	default:
		return -EINVAL;
	}

	ret = check_errors(aspeed, ret);
	if (ret)
		return ret;

	return 0;
}

static int aspeed_master_write(struct fsi_master *master, int link,
			uint8_t id, uint32_t addr, const void *val, size_t size)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int ret;

	if (id != 0)
		return -EINVAL;

	addr += link * FSI_HUB_LINK_SIZE;

	switch (size) {
	case 1:
		ret = opb_writeb(aspeed, fsi_base + addr, *(u8 *)val);
		break;
	case 2:
		ret = opb_writew(aspeed, fsi_base + addr, *(__be16 *)val);
		break;
	case 4:
		ret = opb_writel(aspeed, fsi_base + addr, *(__be32 *)val);
		break;
	default:
		return -EINVAL;
	}

	ret = check_errors(aspeed, ret);
	if (ret)
		return ret;

	return 0;
}

static int aspeed_master_link_enable(struct fsi_master *master, int link)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int idx, bit, ret;
	__be32 reg, result;

	idx = link / 32;
	bit = link % 32;

	reg = cpu_to_be32(0x80000000 >> bit);

	result = opb_writel(aspeed, ctrl_base + FSI_MSENP0 + (4 * idx), reg);

	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	ret = opb_readl(aspeed, ctrl_base + FSI_MENP0 + (4 * idx), &result);
	if (ret)
		return ret;

	if (result != reg) {
		dev_err(aspeed->dev, "%s failed: %08x\n", __func__, result);
		return -EIO;
	}

	return 0;
}

static int aspeed_master_term(struct fsi_master *master, int link, uint8_t id)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	uint32_t addr;
	__be32 cmd;
	int rc;

	addr = 0x4;
	cmd = cpu_to_be32(0xecc00000);

	dev_dbg(aspeed->dev, "sending term to link %d slave %d\n", link, id);

	rc = aspeed_master_write(master, link, id, addr, &cmd, 4);

	dev_dbg(aspeed->dev, "term done (%d)\n", rc);

	return rc;
}

static int aspeed_master_break(struct fsi_master *master, int link)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	uint32_t addr;
	__be32 cmd;
	int rc;

	addr = 0x0;
	cmd = cpu_to_be32(0xc0de0000);

	dev_dbg(aspeed->dev, "sending break to link %d\n", link);

	rc = aspeed_master_write(master, link, 0, addr, &cmd, 4);

	dev_dbg(aspeed->dev, "break done (%d)\n", rc);

	return rc;
}

static void aspeed_master_release(struct device *dev)
{
	struct fsi_master_aspeed *aspeed =
		to_fsi_master_aspeed(dev_to_fsi_master(dev));

	kfree(aspeed);
}

/* mmode encoders */
static inline u32 fsi_mmode_crs0(u32 x)
{
	return (x & FSI_MMODE_CRS0MASK) << FSI_MMODE_CRS0SHFT;
}

static inline u32 fsi_mmode_crs1(u32 x)
{
	return (x & FSI_MMODE_CRS1MASK) << FSI_MMODE_CRS1SHFT;
}

static int aspeed_master_init(struct fsi_master_aspeed *aspeed)
{
	__be32 reg;

	reg = cpu_to_be32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK
			| FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	opb_writel(aspeed, ctrl_base + FSI_MRESP0, reg);

	/* Initialize the MFSI (hub master) engine */
	reg = cpu_to_be32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK
			| FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	opb_writel(aspeed, ctrl_base + FSI_MRESP0, reg);

	reg = cpu_to_be32(FSI_MECTRL_EOAE | FSI_MECTRL_P8_AUTO_TERM);
	opb_writel(aspeed, ctrl_base + FSI_MECTRL, reg);

	reg = cpu_to_be32(FSI_MMODE_ECRC | FSI_MMODE_EPC | FSI_MMODE_RELA
			| fsi_mmode_crs0(DEFAULT_DIVISOR)
			| fsi_mmode_crs1(DEFAULT_DIVISOR)
			| FSI_MMODE_P8_TO_LSB);
	opb_writel(aspeed, ctrl_base + FSI_MMODE, reg);

	reg = cpu_to_be32(0xffff0000);
	opb_writel(aspeed, ctrl_base + FSI_MDLYR, reg);

	reg = cpu_to_be32(~0);
	opb_writel(aspeed, ctrl_base + FSI_MSENP0, reg);

	/* Leave enabled long enough for master logic to set up */
	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	opb_writel(aspeed, ctrl_base + FSI_MCENP0, reg);

	opb_readl(aspeed, ctrl_base + FSI_MAEB, NULL);

	reg = cpu_to_be32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK);
	opb_writel(aspeed, ctrl_base + FSI_MRESP0, reg);

	opb_readl(aspeed, ctrl_base + FSI_MLEVP0, NULL);

	/* Reset the master bridge */
	reg = cpu_to_be32(FSI_MRESB_RST_GEN);
	opb_writel(aspeed, ctrl_base + FSI_MRESB0, reg);

	reg = cpu_to_be32(FSI_MRESB_RST_ERR);
	opb_writel(aspeed, ctrl_base + FSI_MRESB0, reg);

	return 0;
}

static int fsi_master_aspeed_debugfs_get(void *data, u64 *val)
{
	int rc;
	u32 out;
	struct fsi_master_aspeed_debugfs_entry *entry = data;

	rc = opb_readl(entry->aspeed, ctrl_base + entry->addr, &out);
	if (rc)
		return rc;

	*val = (u64)be32_to_cpu(out);
	return 0;
}
static int fsi_master_aspeed_debugfs_set(void *data, u64 val)
{
	u32 rc;
	u32 in = cpu_to_be32((u32)(val & 0xFFFFFFFFULL));
	struct fsi_master_aspeed_debugfs_entry *entry = data;

	rc = opb_writel(entry->aspeed, ctrl_base + entry->addr, in);
	if (rc)
		return rc;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fsi_master_aspeed_debugfs_ops,
			 fsi_master_aspeed_debugfs_get,
			 fsi_master_aspeed_debugfs_set, "0x%08llx\n");

static int fsi_master_aspeed_clock_debugfs_get(void *data, u64 *val)
{
	struct fsi_master_aspeed *aspeed = data;
	u32 out;
	int rc;

	rc = opb_readl(aspeed, ctrl_base, &out);
	if (rc)
		return rc;

	*val = (u64)((be32_to_cpu(out) >> 18) & 0x3ff);

	return 0;
}

static int fsi_master_aspeed_clock_debugfs_set(void *data, u64 val)
{
	struct fsi_master_aspeed *aspeed = data;
	u32 reg, rc;
	__be32 raw;

	if (val > 0x3ff)
		return -EINVAL;

	rc = opb_readl(aspeed, ctrl_base, &raw);
	if (rc)
		return rc;

	reg = be32_to_cpu(raw);

	reg &= ~(0x3ff << 18);
	reg |= (val & 0x3ff) << 18;

	rc = opb_writel(aspeed, ctrl_base, cpu_to_be32(reg));
	if (rc)
		return rc;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fsi_master_aspeed_clock_debugfs_ops,
			 fsi_master_aspeed_clock_debugfs_get,
			 fsi_master_aspeed_clock_debugfs_set, "0x%llx\n");


static int fsi_master_aspeed_probe(struct platform_device *pdev)
{
	struct fsi_master_aspeed *aspeed;
	struct resource *res;
	int rc, links, reg;
	__be32 raw;

	aspeed = devm_kzalloc(&pdev->dev, sizeof(*aspeed), GFP_KERNEL);
	if (!aspeed)
		return -ENOMEM;

	aspeed->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	aspeed->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(aspeed->base))
		return PTR_ERR(aspeed->base);

	aspeed->clk = devm_clk_get(aspeed->dev, NULL);
	if (IS_ERR(aspeed->clk)) {
		dev_err(aspeed->dev, "couldn't get clock\n");
		return PTR_ERR(aspeed->clk);
	}
	rc = clk_prepare_enable(aspeed->clk);
	if (rc) {
		dev_err(aspeed->dev, "couldn't enable clock\n");
		return rc;
	}

	writel(0x1, aspeed->base + OPB_CLK_SYNC);
	writel(OPB1_XFER_ACK_EN | OPB0_XFER_ACK_EN,
			aspeed->base + OPB_IRQ_MASK);
	/* TODO: Try without this */
	writel(0x10, aspeed->base + 0x64); // Retry counter number ???
	writel(0x0f, aspeed->base + 0xe4); // DMA Enable

	writel(ctrl_base, aspeed->base + OPB_CTRL_BASE);
	writel(fsi_base, aspeed->base + OPB_FSI_BASE);

	/* Set read data order */
	writel(0x0011bb1b, aspeed->base + OPB0_R_ENDIAN);

	/* Set write data order */
	writel(0x0011bb1b, aspeed->base + OPB0_W_ENDIAN);
	writel(0xffaa5500, aspeed->base + 0x50);

	/*
	 * Select OPB0 for all operations.
	 * Will need to be reworked when enabling DMA or anything that uses
	 * OPB1.
	 */
	writel(0x1, aspeed->base + OPB0_SELECT);

	rc = opb_readl(aspeed, ctrl_base + FSI_MVER, &raw);
	if (rc) {
		dev_err(&pdev->dev, "failed to read hub version\n");
		return rc;
	}

	reg = be32_to_cpu(raw);
	links = (reg >> 8) & 0xff;
	dev_info(&pdev->dev, "hub version %08x (%d links)\n", reg, links);

	aspeed->master.dev.parent = &pdev->dev;
	aspeed->master.dev.release = aspeed_master_release;
	aspeed->master.dev.of_node = of_node_get(dev_of_node(&pdev->dev));

	aspeed->master.n_links = links;
	aspeed->master.read = aspeed_master_read;
	aspeed->master.write = aspeed_master_write;
	aspeed->master.send_break = aspeed_master_break;
	aspeed->master.term = aspeed_master_term;
	aspeed->master.link_enable = aspeed_master_link_enable;

	dev_set_drvdata(&pdev->dev, aspeed);

	aspeed_master_init(aspeed);

	aspeed->debugfs_dir = debugfs_create_dir("fsi-master-aspeed", NULL);
	if (aspeed->debugfs_dir) {
		int idx = 0;
		struct fsi_master_aspeed_debugfs_entry *etrs = aspeed->debugfs;

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MMODE;
		debugfs_create_file("mmode", 0644, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MDLYR;
		debugfs_create_file("mdlyr", 0644, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MCRSP;
		debugfs_create_file("mcrsp0", 0644, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MENP0;
		debugfs_create_file("menp0", 0644, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MLEVP0;
		debugfs_create_file("mlevp0", 0444, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MSENP0;
		debugfs_create_file("msenp0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MCENP0;
		debugfs_create_file("mcenp0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MAEB;
		debugfs_create_file("maeb", 0444, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MVER;
		debugfs_create_file("mver", 0444, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MSTAP0;
		debugfs_create_file("mstap0", 0444, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MRESP0;
		debugfs_create_file("mresp0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MESRB0;
		debugfs_create_file("mesrb0", 0444, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MRESB0;
		debugfs_create_file("mresb0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MSCSB0;
		debugfs_create_file("mscsb0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MATRB0;
		debugfs_create_file("matrb0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MDTRB0;
		debugfs_create_file("mdtrb0", 0200, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		etrs[idx].aspeed = aspeed;
		etrs[idx].addr = FSI_MECTRL;
		debugfs_create_file("mectrl", 0644, aspeed->debugfs_dir,
				    &etrs[idx++],
				    &fsi_master_aspeed_debugfs_ops);

		debugfs_create_file("clock_div", 0644, aspeed->debugfs_dir,
				aspeed, &fsi_master_aspeed_clock_debugfs_ops);
	}

	rc = fsi_master_register(&aspeed->master);
	if (rc)
		goto err_release;

	/* At this point, fsi_master_register performs the device_initialize(),
	 * and holds the sole reference on master.dev. This means the device
	 * will be freed (via ->release) during any subsequent call to
	 * fsi_master_unregister.  We add our own reference to it here, so we
	 * can perform cleanup (in _remove()) without it being freed before
	 * we're ready.
	 */
	get_device(&aspeed->master.dev);
	return 0;

err_release:
	clk_disable_unprepare(aspeed->clk);
	return rc;
}

static int fsi_master_aspeed_remove(struct platform_device *pdev)
{
	struct fsi_master_aspeed *aspeed = platform_get_drvdata(pdev);

	debugfs_remove_recursive(aspeed->debugfs_dir);

	fsi_master_unregister(&aspeed->master);
	clk_disable_unprepare(aspeed->clk);

	return 0;
}

static const struct of_device_id fsi_master_aspeed_match[] = {
	{ .compatible = "aspeed,ast2600-fsi-master" },
	{ },
};

static struct platform_driver fsi_master_aspeed_driver = {
	.driver = {
		.name		= "fsi-master-aspeed",
		.of_match_table	= fsi_master_aspeed_match,
	},
	.probe	= fsi_master_aspeed_probe,
	.remove = fsi_master_aspeed_remove,
};

module_platform_driver(fsi_master_aspeed_driver);
MODULE_LICENSE("GPL");
