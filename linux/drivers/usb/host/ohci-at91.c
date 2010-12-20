/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 *  Copyright (C) 2004 SAN People (Pty) Ltd.
 *  Copyright (C) 2005 Thibaut VARENE <varenet@parisc-linux.org>
 * 
 * AT91RM9200 Bus Glue
 *
 * Based on fragments of 2,4 driver by Rick Bronson.
 * Based on ohci-omap.c
 *
 * This file is licenced under the GPL.
 */

#include <linux/device.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/AT91RM9200_UHP.h>
#include <asm/hardware/clock.h>

#ifndef CONFIG_ARCH_AT91RM9200
#error "This file is AT91RM9200 bus glue.  CONFIG_ARCH_AT91RM9200 must be defined."
#endif

/* interface and function clocks */
static struct clk *iclk, *fclk;

extern int usb_disabled(void);

/*-------------------------------------------------------------------------*/

static void at91_start_hc(struct platform_device *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);
	AT91PS_UHP __iomem ohci_regs = hcd->regs;

	dev_dbg(&pdev->dev, "starting AT91RM9200 OHCI USB Controller\n");

	clk_use(iclk);
	clk_use(fclk);

	/*
	 * Configure the power sense and control lines.  Place the USB
	 * host controller in reset.
	 */
	ohci_regs->UHP_HcControl = 0;
}

static void at91_stop_hc(struct platform_device *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);
	AT91PS_UHP __iomem ohci_regs = hcd->regs;

	dev_dbg(&pdev->dev, "stopping AT91RM9200 OHCI USB Controller\n");

	/*
	 * Put the USB host controller into reset.
	 */
	ohci_regs->UHP_HcControl = 0;

	/*
	 * Stop the USB clocks.
	 */
	clk_unuse(fclk);
	clk_unuse(iclk);
}


/*-------------------------------------------------------------------------*/

static int usb_hcd_at91_remove (struct usb_hcd *, struct platform_device *);

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */


/**
 * usb_hcd_at91_probe - initialize AT91RM9200-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
int usb_hcd_at91_probe (const struct hc_driver *driver,
			struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd = NULL;

	if (pdev->num_resources != 2) {
		pr_debug("hcd probe: invalid num_resources");
		return -ENODEV;
	}

	if ((pdev->resource[0].flags != IORESOURCE_MEM) || (pdev->resource[1].flags != IORESOURCE_IRQ)) {
		pr_debug("hcd probe: invalid resource type\n");
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, "at91rm9200");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed\n");
		retval = -EIO;
		goto err2;
	}

	iclk = clk_get(&pdev->dev, "ohci_clk");
	fclk = clk_get(&pdev->dev, "uhpck");

	at91_start_hc(pdev);
	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, pdev->resource[1].start, SA_INTERRUPT);
	if (retval == 0)
		return retval;


	/* Error handling */
	at91_stop_hc(pdev);
	iounmap(hcd->regs);

 err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

 err1:
	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_at91_remove - shutdown processing for AT91RM9200-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_at91_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static int usb_hcd_at91_remove (struct usb_hcd *hcd, struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	at91_stop_hc(pdev);
	iounmap(hcd->regs);
 	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

	clk_put(fclk);
	clk_put(iclk);

	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

/*-------------------------------------------------------------------------*/

static int __devinit
ohci_at91_start (struct usb_hcd *hcd)
{
//	struct at91_ohci_data	*board = hcd->self.controller->platform_data;
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);
	int			ret;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run(ohci)) < 0) {
		err("can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}
//	hcd->self.root_hub->maxchild = board->ports;
	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_at91_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"AT91RM9200 OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_at91_start,
	.stop =			ohci_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,

#ifdef CONFIG_PM
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_at91_drv_probe(struct device *dev)
{
	return usb_hcd_at91_probe(&ohci_at91_hc_driver, to_platform_device(dev));
}

static int ohci_hcd_at91_drv_remove(struct device *dev)
{
	return usb_hcd_at91_remove(dev_get_drvdata(dev), to_platform_device(dev));
}

#ifdef CONFIG_PM

static int ohci_hcd_at91_drv_suspend(struct device *dev, u32 state, u32 level)
{
  //	struct platform_device *pdev = to_platform_device(dev);
  //	struct usb_hcd *hcd = dev_get_drvdata(dev);
	printk("%s(%s:%d): not implemented yet\n",
		__func__, __FILE__, __LINE__);

	clk_unuse(fclk);
	return 0;
}

static int ohci_hcd_at91_drv_resume(struct device *dev, u32 state)
{
//	struct platform_device *pdev = to_platform_device(dev);
//	struct usb_hcd *hcd = dev_get_drvdata(dev);
	printk("%s(%s:%d): not implemented yet\n",
		__func__, __FILE__, __LINE__);

	clk_use(fclk);
	return 0;
}

#endif

static struct device_driver ohci_hcd_at91_driver = {
	.name		= "at91rm9200-ohci",
	.bus		= &platform_bus_type,
	.probe		= ohci_hcd_at91_drv_probe,
	.remove		= ohci_hcd_at91_drv_remove,
#ifdef CONFIG_PM
	.suspend	= ohci_hcd_at91_drv_suspend,
	.resume		= ohci_hcd_at91_drv_resume,
#endif
};

static int __init ohci_hcd_at91_init (void)
{
	if (usb_disabled())
		return -ENODEV;

	return driver_register(&ohci_hcd_at91_driver);
}

static void __exit ohci_hcd_at91_cleanup (void)
{
	driver_unregister(&ohci_hcd_at91_driver);
}

module_init (ohci_hcd_at91_init);
module_exit (ohci_hcd_at91_cleanup);
