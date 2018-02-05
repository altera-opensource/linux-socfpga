/*
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/pm_runtime.h>
#include "vc4_drv.h"
#include "vc4_regs.h"

#ifdef CONFIG_DEBUG_FS
#define REGDEF(reg) { reg, #reg }
static const struct {
	uint32_t reg;
	const char *name;
} vc4_reg_defs[] = {
	REGDEF(V3D_IDENT0),
	REGDEF(V3D_IDENT1),
	REGDEF(V3D_IDENT2),
	REGDEF(V3D_SCRATCH),
	REGDEF(V3D_L2CACTL),
	REGDEF(V3D_SLCACTL),
	REGDEF(V3D_INTCTL),
	REGDEF(V3D_INTENA),
	REGDEF(V3D_INTDIS),
	REGDEF(V3D_CT0CS),
	REGDEF(V3D_CT1CS),
	REGDEF(V3D_CT0EA),
	REGDEF(V3D_CT1EA),
	REGDEF(V3D_CT0CA),
	REGDEF(V3D_CT1CA),
	REGDEF(V3D_CT00RA0),
	REGDEF(V3D_CT01RA0),
	REGDEF(V3D_CT0LC),
	REGDEF(V3D_CT1LC),
	REGDEF(V3D_CT0PC),
	REGDEF(V3D_CT1PC),
	REGDEF(V3D_PCS),
	REGDEF(V3D_BFC),
	REGDEF(V3D_RFC),
	REGDEF(V3D_BPCA),
	REGDEF(V3D_BPCS),
	REGDEF(V3D_BPOA),
	REGDEF(V3D_BPOS),
	REGDEF(V3D_BXCF),
	REGDEF(V3D_SQRSV0),
	REGDEF(V3D_SQRSV1),
	REGDEF(V3D_SQCNTL),
	REGDEF(V3D_SRQPC),
	REGDEF(V3D_SRQUA),
	REGDEF(V3D_SRQUL),
	REGDEF(V3D_SRQCS),
	REGDEF(V3D_VPACNTL),
	REGDEF(V3D_VPMBASE),
	REGDEF(V3D_PCTRC),
	REGDEF(V3D_PCTRE),
	REGDEF(V3D_PCTR0),
	REGDEF(V3D_PCTRS0),
	REGDEF(V3D_PCTR1),
	REGDEF(V3D_PCTRS1),
	REGDEF(V3D_PCTR2),
	REGDEF(V3D_PCTRS2),
	REGDEF(V3D_PCTR3),
	REGDEF(V3D_PCTRS3),
	REGDEF(V3D_PCTR4),
	REGDEF(V3D_PCTRS4),
	REGDEF(V3D_PCTR5),
	REGDEF(V3D_PCTRS5),
	REGDEF(V3D_PCTR6),
	REGDEF(V3D_PCTRS6),
	REGDEF(V3D_PCTR7),
	REGDEF(V3D_PCTRS7),
	REGDEF(V3D_PCTR8),
	REGDEF(V3D_PCTRS8),
	REGDEF(V3D_PCTR9),
	REGDEF(V3D_PCTRS9),
	REGDEF(V3D_PCTR10),
	REGDEF(V3D_PCTRS10),
	REGDEF(V3D_PCTR11),
	REGDEF(V3D_PCTRS11),
	REGDEF(V3D_PCTR12),
	REGDEF(V3D_PCTRS12),
	REGDEF(V3D_PCTR13),
	REGDEF(V3D_PCTRS13),
	REGDEF(V3D_PCTR14),
	REGDEF(V3D_PCTRS14),
	REGDEF(V3D_PCTR15),
	REGDEF(V3D_PCTRS15),
	REGDEF(V3D_DBGE),
	REGDEF(V3D_FDBGO),
	REGDEF(V3D_FDBGB),
	REGDEF(V3D_FDBGR),
	REGDEF(V3D_FDBGS),
	REGDEF(V3D_ERRSTAT),
};

int vc4_v3d_debugfs_regs(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(vc4_reg_defs); i++) {
		seq_printf(m, "%s (0x%04x): 0x%08x\n",
			   vc4_reg_defs[i].name, vc4_reg_defs[i].reg,
			   V3D_READ(vc4_reg_defs[i].reg));
	}

	return 0;
}

int vc4_v3d_debugfs_ident(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	uint32_t ident1 = V3D_READ(V3D_IDENT1);
	uint32_t nslc = VC4_GET_FIELD(ident1, V3D_IDENT1_NSLC);
	uint32_t tups = VC4_GET_FIELD(ident1, V3D_IDENT1_TUPS);
	uint32_t qups = VC4_GET_FIELD(ident1, V3D_IDENT1_QUPS);

	seq_printf(m, "Revision:   %d\n",
		   VC4_GET_FIELD(ident1, V3D_IDENT1_REV));
	seq_printf(m, "Slices:     %d\n", nslc);
	seq_printf(m, "TMUs:       %d\n", nslc * tups);
	seq_printf(m, "QPUs:       %d\n", nslc * qups);
	seq_printf(m, "Semaphores: %d\n",
		   VC4_GET_FIELD(ident1, V3D_IDENT1_NSEM));

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static void vc4_v3d_init_hw(struct drm_device *dev)
{
	struct vc4_dev *vc4 = to_vc4_dev(dev);

	/* Take all the memory that would have been reserved for user
	 * QPU programs, since we don't have an interface for running
	 * them, anyway.
	 */
	V3D_WRITE(V3D_VPMBASE, 0);
}

int vc4_v3d_get_bin_slot(struct vc4_dev *vc4)
{
	struct drm_device *dev = vc4->dev;
	unsigned long irqflags;
	int slot;
	uint64_t seqno = 0;
	struct vc4_exec_info *exec;

try_again:
	spin_lock_irqsave(&vc4->job_lock, irqflags);
	slot = ffs(~vc4->bin_alloc_used);
	if (slot != 0) {
		/* Switch from ffs() bit index to a 0-based index. */
		slot--;
		vc4->bin_alloc_used |= BIT(slot);
		spin_unlock_irqrestore(&vc4->job_lock, irqflags);
		return slot;
	}

	/* Couldn't find an open slot.  Wait for render to complete
	 * and try again.
	 */
	exec = vc4_last_render_job(vc4);
	if (exec)
		seqno = exec->seqno;
	spin_unlock_irqrestore(&vc4->job_lock, irqflags);

	if (seqno) {
		int ret = vc4_wait_for_seqno(dev, seqno, ~0ull, true);

		if (ret == 0)
			goto try_again;

		return ret;
	}

	return -ENOMEM;
}

/**
 * vc4_allocate_bin_bo() - allocates the memory that will be used for
 * tile binning.
 *
 * The binner has a limitation that the addresses in the tile state
 * buffer that point into the tile alloc buffer or binner overflow
 * memory only have 28 bits (256MB), and the top 4 on the bus for
 * tile alloc references end up coming from the tile state buffer's
 * address.
 *
 * To work around this, we allocate a single large buffer while V3D is
 * in use, make sure that it has the top 4 bits constant across its
 * entire extent, and then put the tile state, tile alloc, and binner
 * overflow memory inside that buffer.
 *
 * This creates a limitation where we may not be able to execute a job
 * if it doesn't fit within the buffer that we allocated up front.
 * However, it turns out that 16MB is "enough for anybody", and
 * real-world applications run into allocation failures from the
 * overall CMA pool before they make scenes complicated enough to run
 * out of bin space.
 */
int
vc4_allocate_bin_bo(struct drm_device *drm)
{
	struct vc4_dev *vc4 = to_vc4_dev(drm);
	struct vc4_v3d *v3d = vc4->v3d;
	uint32_t size = 16 * 1024 * 1024;
	int ret = 0;
	struct list_head list;

	/* We may need to try allocating more than once to get a BO
	 * that doesn't cross 256MB.  Track the ones we've allocated
	 * that failed so far, so that we can free them when we've got
	 * one that succeeded (if we freed them right away, our next
	 * allocation would probably be the same chunk of memory).
	 */
	INIT_LIST_HEAD(&list);

	while (true) {
		struct vc4_bo *bo = vc4_bo_create(drm, size, true,
						  VC4_BO_TYPE_BIN);

		if (IS_ERR(bo)) {
			ret = PTR_ERR(bo);

			dev_err(&v3d->pdev->dev,
				"Failed to allocate memory for tile binning: "
				"%d. You may need to enable CMA or give it "
				"more memory.",
				ret);
			break;
		}

		/* Check if this BO won't trigger the addressing bug. */
		if ((bo->base.paddr & 0xf0000000) ==
		    ((bo->base.paddr + bo->base.base.size - 1) & 0xf0000000)) {
			vc4->bin_bo = bo;

			/* Set up for allocating 512KB chunks of
			 * binner memory.  The biggest allocation we
			 * need to do is for the initial tile alloc +
			 * tile state buffer.  We can render to a
			 * maximum of ((2048*2048) / (32*32) = 4096
			 * tiles in a frame (until we do floating
			 * point rendering, at which point it would be
			 * 8192).  Tile state is 48b/tile (rounded to
			 * a page), and tile alloc is 32b/tile
			 * (rounded to a page), plus a page of extra,
			 * for a total of 320kb for our worst-case.
			 * We choose 512kb so that it divides evenly
			 * into our 16MB, and the rest of the 512kb
			 * will be used as storage for the overflow
			 * from the initial 32b CL per bin.
			 */
			vc4->bin_alloc_size = 512 * 1024;
			vc4->bin_alloc_used = 0;
			vc4->bin_alloc_overflow = 0;
			WARN_ON_ONCE(sizeof(vc4->bin_alloc_used) * 8 !=
				     bo->base.base.size / vc4->bin_alloc_size);

			break;
		}

		/* Put it on the list to free later, and try again. */
		list_add(&bo->unref_head, &list);
	}

	/* Free all the BOs we allocated but didn't choose. */
	while (!list_empty(&list)) {
		struct vc4_bo *bo = list_last_entry(&list,
						    struct vc4_bo, unref_head);

		list_del(&bo->unref_head);
		drm_gem_object_put_unlocked(&bo->base.base);
	}

	return ret;
}

#ifdef CONFIG_PM
static int vc4_v3d_runtime_suspend(struct device *dev)
{
	struct vc4_v3d *v3d = dev_get_drvdata(dev);
	struct vc4_dev *vc4 = v3d->vc4;

	vc4_irq_uninstall(vc4->dev);

	drm_gem_object_put_unlocked(&vc4->bin_bo->base.base);
	vc4->bin_bo = NULL;

	clk_disable_unprepare(v3d->clk);

	return 0;
}

static int vc4_v3d_runtime_resume(struct device *dev)
{
	struct vc4_v3d *v3d = dev_get_drvdata(dev);
	struct vc4_dev *vc4 = v3d->vc4;
	int ret;

	ret = vc4_allocate_bin_bo(vc4->dev);
	if (ret)
		return ret;

	ret = clk_prepare_enable(v3d->clk);
	if (ret != 0)
		return ret;

	vc4_v3d_init_hw(vc4->dev);

	/* We disabled the IRQ as part of vc4_irq_uninstall in suspend. */
	enable_irq(vc4->dev->irq);
	vc4_irq_postinstall(vc4->dev);

	return 0;
}
#endif

static int vc4_v3d_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = dev_get_drvdata(master);
	struct vc4_dev *vc4 = to_vc4_dev(drm);
	struct vc4_v3d *v3d = NULL;
	int ret;

	v3d = devm_kzalloc(&pdev->dev, sizeof(*v3d), GFP_KERNEL);
	if (!v3d)
		return -ENOMEM;

	dev_set_drvdata(dev, v3d);

	v3d->pdev = pdev;

	v3d->regs = vc4_ioremap_regs(pdev, 0);
	if (IS_ERR(v3d->regs))
		return PTR_ERR(v3d->regs);

	vc4->v3d = v3d;
	v3d->vc4 = vc4;

	v3d->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(v3d->clk)) {
		int ret = PTR_ERR(v3d->clk);

		if (ret == -ENOENT) {
			/* bcm2835 didn't have a clock reference in the DT. */
			ret = 0;
			v3d->clk = NULL;
		} else {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get V3D clock: %d\n",
					ret);
			return ret;
		}
	}

	if (V3D_READ(V3D_IDENT0) != V3D_EXPECTED_IDENT0) {
		DRM_ERROR("V3D_IDENT0 read 0x%08x instead of 0x%08x\n",
			  V3D_READ(V3D_IDENT0), V3D_EXPECTED_IDENT0);
		return -EINVAL;
	}

	ret = clk_prepare_enable(v3d->clk);
	if (ret != 0)
		return ret;

	ret = vc4_allocate_bin_bo(drm);
	if (ret) {
		clk_disable_unprepare(v3d->clk);
		return ret;
	}

	/* Reset the binner overflow address/size at setup, to be sure
	 * we don't reuse an old one.
	 */
	V3D_WRITE(V3D_BPOA, 0);
	V3D_WRITE(V3D_BPOS, 0);

	vc4_v3d_init_hw(drm);

	ret = drm_irq_install(drm, platform_get_irq(pdev, 0));
	if (ret) {
		DRM_ERROR("Failed to install IRQ handler\n");
		return ret;
	}

	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 40); /* a little over 2 frames. */
	pm_runtime_enable(dev);

	return 0;
}

static void vc4_v3d_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct drm_device *drm = dev_get_drvdata(master);
	struct vc4_dev *vc4 = to_vc4_dev(drm);

	pm_runtime_disable(dev);

	drm_irq_uninstall(drm);

	/* Disable the binner's overflow memory address, so the next
	 * driver probe (if any) doesn't try to reuse our old
	 * allocation.
	 */
	V3D_WRITE(V3D_BPOA, 0);
	V3D_WRITE(V3D_BPOS, 0);

	vc4->v3d = NULL;
}

static const struct dev_pm_ops vc4_v3d_pm_ops = {
	SET_RUNTIME_PM_OPS(vc4_v3d_runtime_suspend, vc4_v3d_runtime_resume, NULL)
};

static const struct component_ops vc4_v3d_ops = {
	.bind   = vc4_v3d_bind,
	.unbind = vc4_v3d_unbind,
};

static int vc4_v3d_dev_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &vc4_v3d_ops);
}

static int vc4_v3d_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vc4_v3d_ops);
	return 0;
}

static const struct of_device_id vc4_v3d_dt_match[] = {
	{ .compatible = "brcm,bcm2835-v3d" },
	{ .compatible = "brcm,cygnus-v3d" },
	{ .compatible = "brcm,vc4-v3d" },
	{}
};

struct platform_driver vc4_v3d_driver = {
	.probe = vc4_v3d_dev_probe,
	.remove = vc4_v3d_dev_remove,
	.driver = {
		.name = "vc4_v3d",
		.of_match_table = vc4_v3d_dt_match,
		.pm = &vc4_v3d_pm_ops,
	},
};
