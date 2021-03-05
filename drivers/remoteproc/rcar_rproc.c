// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) IoT.bzh 2021
 */

#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/soc/renesas/rcar-rst.h>

#include "remoteproc_internal.h"

struct rcar_rproc {
	struct device			*dev;
	struct rproc			*rproc;
	struct reset_control            *rst;
};

static int rcar_rproc_mem_alloc(struct rproc *rproc,
				 struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %p+%zx\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %pa+%lx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int rcar_rproc_mem_release(struct rproc *rproc,
				   struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static int rcar_rproc_prepare(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	u64 da;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		/* No need to translate pa to da, R-Car use same map */
		da = rmem->base;

		mem = rproc_mem_entry_init(dev, NULL,
					   (dma_addr_t)rmem->base,
					   rmem->size, da,
					   rcar_rproc_mem_alloc,
					   rcar_rproc_mem_release,
					   it.node->name);

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
	}

	return 0;
}

static int rcar_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_info(&rproc->dev, "No resource table in elf\n");

	return 0;
}

static int rcar_rproc_start(struct rproc *rproc)
{
	struct rcar_rproc *priv = rproc->priv;
	int err;

	err = reset_control_deassert(priv->rst);
	if (err)
		dev_err(&rproc->dev, "failed to deassert reset\n");

	return err;
}

static int rcar_rproc_stop(struct rproc *rproc)
{
	struct rcar_rproc *priv = rproc->priv;
	int err;

	err = reset_control_assert(priv->rst);
	if (err)
		dev_err(&rproc->dev, "failed to assert reset\n");

	return err;
}

static struct rproc_ops rcar_rproc_ops = {
	.prepare	= rcar_rproc_prepare,
	.start		= rcar_rproc_start,
	.stop		= rcar_rproc_stop,
	.load		= rproc_elf_load_segments,
	.parse_fw	= rcar_rproc_parse_fw,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= rproc_elf_get_boot_addr,

};

static int rcar_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rcar_rproc *priv;
	struct rproc *rproc;
	int ret;

	rproc = rproc_alloc(dev, np->name, &rcar_rproc_ops,
			    NULL, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->dev = dev;

	priv->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		ret = PTR_ERR(priv->rst);
		dev_err(dev, "failed to get rproc reset\n");
		goto free_rproc;
	}

	pm_runtime_enable(priv->dev);
	ret = pm_runtime_get_sync(priv->dev);
	if (ret) {
		dev_err(&rproc->dev, "failed to power up\n");
		goto free_rproc;
	}

	dev_set_drvdata(dev, rproc);

	/* Manually start the rproc */
	rproc->auto_boot = false;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto pm_disable;
	}

	return 0;

pm_disable:
	pm_runtime_disable(priv->dev);
free_rproc:
	rproc_free(rproc);

	return ret;
}

static int rcar_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct rcar_rproc *priv = rproc->priv;

	rproc_del(rproc);
	pm_runtime_disable(priv->dev);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id rcar_rproc_of_match[] = {
	{ .compatible = "renesas,rcar-cr7" },
	{},
};

MODULE_DEVICE_TABLE(of, rcar_rproc_of_match);

static struct platform_driver rcar_rproc_driver = {
	.probe = rcar_rproc_probe,
	.remove = rcar_rproc_remove,
	.driver = {
		.name = "rcar-rproc",
		.of_match_table = rcar_rproc_of_match,
	},
};

module_platform_driver(rcar_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas Gen3 RCAR remote processor control driver");
MODULE_AUTHOR("Julien Massot <julien.massot@iot.bzh>");
