// https://github.com/sarfata/pi-blaster/blob/master/pi-blaster.c
// https://github.com/richardghirst/PiBits/blob/master/ServoBlaster/kernel/servoblaster.c
// http://www.valvers.com/wp-content/uploads/2013/01/arm-c-virtual-addresses.jpg
// https://www.kernel.org/doc/gorman/html/understand/understand009.html

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gpio.h>

#include "common.h"

// TODO: use device tree : xxd /proc/device-tree/soc/ranges (2nd word)
#if   defined (MYLIFE_ARCH_RPI1)
#define IO_PHYS_BASE 0x20000000
#elif defined (MYLIFE_ARCH_RPI2)
#define IO_PHYS_BASE 0x3f000000
#else
#error "Unknown arch"
#endif

#define CYCLE_TIME_US 10000
#define SAMPLE_US     10
#define NUM_SAMPLES   (CYCLE_TIME_US / SAMPLE_US)
#define NUM_CBS       (NUM_SAMPLES * 2)

struct dma_cb {
  uint32_t info;
  uint32_t src;
  uint32_t dst;
  uint32_t length;
  uint32_t stride;
  uint32_t next;
  uint32_t pad[2];
};

struct ctl {
  uint32_t sample[NUM_SAMPLES];
  struct dma_cb cb[NUM_CBS];
};

#define NUM_PAGES     ((sizeof(struct ctl) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define PWM_BASE      (IO_PHYS_BASE + 0x20C000)
#define PWM_LEN       0x28
#define CLK_BASE      (IO_PHYS_BASE + 0x101000)
#define CLK_LEN       0xA8
#define DMA_BASE      (IO_PHYS_BASE + 0x00007000)
#define DMA_CHAN_NUM  14    // the DMA Channel we are using, NOTE: DMA Ch 0 seems to be used by X... better not use it ;)
#define DMA_CHAN_SIZE 0x100 // size of register space for a single DMA channel
#define DMA_CHAN_MAX  14    // number of DMA Channels we have... actually, there are 15... but channel fifteen is mapped at a different DMA_BASE, so we leave that one alone
#define DMA_CHAN_BASE (DMA_BASE + DMA_CHAN_NUM * DMA_CHAN_SIZE)

static volatile uint32_t *dma_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *pwm_reg;

int hw_init(void) {

  printk(KERN_INFO "DMA Channel:                 %d\n", DMA_CHAN_NUM);
  printk(KERN_INFO "PWM frequency:               %5d Hz\n", 1000000/CYCLE_TIME_US);
  printk(KERN_INFO "PWM steps:                   %5d\n", NUM_SAMPLES);
  printk(KERN_INFO "Maximum period (100  %%):   %5dus\n", CYCLE_TIME_US);
  printk(KERN_INFO "Minimum period (%1.3f%%): %5dus\n", 100.0 * SAMPLE_US / CYCLE_TIME_US, SAMPLE_US);

  dma_reg = memremap(DMA_CHAN_BASE, DMA_CHAN_SIZE, MEMREMAP_WT);
  pwm_reg = memremap(PWM_BASE, PWM_LEN, MEMREMAP_WT);
  clk_reg = memremap(CLK_BASE, CLK_LEN, MEMREMAP_WT);

  printk(KERN_INFO "NUM_PAGES:                 %d\n", NUM_PAGES);
/*
  unsigned long __get_free_pages(unsigned int gfp_mask, unsigned int order)
  GFP_KERNEL

  void __free_pages(struct page *page, unsigned int order)
 Free an order number of pages from the given page

void __free_page(struct page *page)
 Free a single page

void free_page(void *addr)
 Free a page from the given virtual address



  // Use the mailbox interface to the VC to ask for physical memory
  mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * PAGE_SIZE, PAGE_SIZE, mem_flag);
  dprintf("mem_ref %u\n", mbox.mem_ref);
  mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
  dprintf("bus_addr = %#x\n", mbox.bus_addr);
  mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * PAGE_SIZE);
  dprintf("virt_addr %p\n", mbox.virt_addr);

  if ((unsigned long)mbox.virt_addr & (PAGE_SIZE-1))
    fatal("pi-blaster: Virtual address is not page aligned\n");

  // we are done with the mbox
  mbox_close(mbox.handle);
  mbox.handle = -1;

  init_ctrl_data();
  init_hardware();
  init_channel_pwm();
  // Init pin2gpio array with 0/false values to avoid locking all of them as PWM.
  init_pin2gpio();
  // Only calls update_pwm after ctrl_data calculates the pin mask to unlock all pins on start.
*/
  hw_update();

  return 0;
}

void hw_exit(void) {
/*
  int i;

  dprintf("Resetting DMA...\n");
  if (dma_reg && mbox.virt_addr) {
    for (i = 0; i < num_channels; i++)
      channel_pwm[i] = 0;
    update_pwm();
    udelay(CYCLE_TIME_US);
    dma_reg[DMA_CS] = DMA_RESET;
    udelay(10);
  }

  dprintf("Freeing mbox memory...\n");
  if (mbox.virt_addr != NULL) {
    unmapmem(mbox.virt_addr, NUM_PAGES * PAGE_SIZE);
    if (mbox.handle <= 2) {
      mbox.handle = mbox_open();
    }
    mem_unlock(mbox.handle, mbox.mem_ref);
    mem_free(mbox.handle, mbox.mem_ref);
    mbox_close(mbox.handle);
  }
*/
  memunmap(dma_reg);
  memunmap(pwm_reg);
  memunmap(clk_reg);
}

void hw_update(void) {
  // TODO
}
