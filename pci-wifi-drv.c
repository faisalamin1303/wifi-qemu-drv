#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>

#define VID 0x1234
#define DID 0xbeef

static struct pci_device_id wifi_ids[] = {
	{PCI_DEVICE(VID, DID)},
	{},
};
MODULE_DEVICE_TABLE(pci, wifi_ids);

static int wifi_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
	int status;
	void __iomem *ptr_bar0, __iomem *ptr_bar1;
	status = pcim_enable_device(pdev);
	if(status != 0){
		printk("pci-wifi-drv - Error enabling device\n");
		return status;
	}
	ptr_bar0 = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if(!ptr_bar0) {
		printk("pci-wifi-drv - Error mapping BAR0\n");
		return -ENODEV;
	}

	ptr_bar1 = pcim_iomap(pdev, 1, pci_resource_len(pdev, 1));
	if(!ptr_bar0) {
		printk("pci-wifi-drv - Error mapping BAR1\n");
		return -ENODEV;
	}
	printk("pci-wifi-drv - ID: 0x%x\n", ioread32(ptr_bar0));
	printk("pci-wifi-drv - Random Value: 0x%x\n", ioread32(ptr_bar0 + 0xc));
	
	iowrite32(0x11223344, ptr_bar0 + 4);
	mdelay(1);

	printk("pci-wifi-drv - Inverse Pattern: 0x%x\n", ioread32(ptr_bar0 + 0x4));
	
	iowrite32(0x44332211, ptr_bar1);	
	printk("pci-wifi-drv - BAR1 offset 0: 0x%x\n", ioread8(ptr_bar1));
	printk("pci-wifi-drv - BAR1 offset 0: 0x%x\n", ioread16(ptr_bar1));
	printk("pci-wifi-drv - BAR1 offset 0: 0x%x\n", ioread32(ptr_bar1));
	
	return 0;
}

static void wifi_remove(struct pci_dev *pdev) {
	printk("pci-wifi -Removing the device\n");
}

static struct pci_driver wifi_driver = {
	.name = "pci-wifi-driver",
	.probe = wifi_probe,
	.remove = wifi_remove,
	.id_table = wifi_ids,
};

module_pci_driver(wifi_driver);

MODULE_LICENSE("GPL");