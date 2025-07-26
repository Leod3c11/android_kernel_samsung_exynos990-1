#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>

#include <soc/samsung/cal-if.h>  // cal_dfs_set_rate

#define GPU_DOMAIN_ID	MARGIN_G3D  // confirme se esse é o correto!

struct gpu_ctrl_data {
	struct device *dev;
	struct class *class;
	struct device *sys_dev;
	u32 current_max_freq;
};

static struct gpu_ctrl_data *ctrl;

static ssize_t max_freq_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ctrl->current_max_freq);
}

static ssize_t max_freq_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long freq;
	int ret;

	ret = kstrtoul(buf, 10, &freq);
	if (ret)
		return ret;

	ret = cal_dfs_set_rate(GPU_DOMAIN_ID, freq);
	if (ret) {
		dev_err(ctrl->dev, "Falha ao aplicar frequência: %lu\n", freq);
		return ret;
	}

	ctrl->current_max_freq = freq;
	dev_info(ctrl->dev, "Nova frequência aplicada: %lu\n", freq);
	return count;
}

static DEVICE_ATTR_RW(max_freq);

static int exynos_gpu_ctrl_probe(struct platform_device *pdev)
{
	int ret;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = &pdev->dev;

	// Criar classe
	ctrl->class = class_create(THIS_MODULE, "gpu_ctrl");
	if (IS_ERR(ctrl->class))
		return PTR_ERR(ctrl->class);

	// Criar dispositivo virtual em /sys/class/gpu_ctrl/
	ctrl->sys_dev = device_create(ctrl->class, NULL, 0, NULL, "freq");
	if (IS_ERR(ctrl->sys_dev)) {
		class_destroy(ctrl->class);
		return PTR_ERR(ctrl->sys_dev);
	}

	ret = device_create_file(ctrl->sys_dev, &dev_attr_max_freq);
	if (ret) {
		device_destroy(ctrl->class, 0);
		class_destroy(ctrl->class);
		return ret;
	}

	dev_info(&pdev->dev, "Controlador de frequência da GPU iniciado.\n");
	return 0;
}

static int exynos_gpu_ctrl_remove(struct platform_device *pdev)
{
	device_remove_file(ctrl->sys_dev, &dev_attr_max_freq);
	device_destroy(ctrl->class, 0);
	class_destroy(ctrl->class);
	dev_info(&pdev->dev, "Controlador de frequência da GPU descarregado.\n");
	return 0;
}

static const struct of_device_id exynos_gpu_ctrl_of_match[] = {
	{ .compatible = "samsung,exynos-gpu-freq-ctrl", },
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_gpu_ctrl_of_match);

static struct platform_driver exynos_gpu_ctrl_driver = {
	.probe	= exynos_gpu_ctrl_probe,
	.remove	= exynos_gpu_ctrl_remove,
	.driver	= {
		.name	= "exynos_gpu_freq_ctrl",
		.of_match_table = exynos_gpu_ctrl_of_match,
	},
};

module_platform_driver(exynos_gpu_ctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leonardo");
MODULE_DESCRIPTION("Controlador manual de frequência da GPU via sysfs");
