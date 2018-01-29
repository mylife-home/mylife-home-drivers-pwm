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

static unsigned long ctl_addr;
static void *dma_reg_addr;
static void *clk_reg_addr;
static void *pwm_reg_addr;
static volatile uint32_t *dma_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *pwm_reg;

static unsigned int get_page_order(unsigned int page_count);
static void memory_cleanup(void);

inline unsigned int get_page_order(unsigned int page_count) {
  unsigned int order = 0;
  while(1 << order < page_count) {
    ++order;
  }
  return order;
}

void memory_cleanup(void) {
  if(ctl_addr) {
    free_pages(ctl_addr, get_page_order(NUM_PAGES));
    ctl_addr = 0;
  }

  if(dma_reg_addr) {
    memunmap(dma_reg_addr);
    dma_reg_addr = NULL;
  }

  if(pwm_reg_addr) {
    memunmap(pwm_reg_addr);
    pwm_reg_addr = NULL;
  }

  if(clk_reg_addr) {
    memunmap(clk_reg_addr);
    clk_reg_addr = NULL;
  }
}

int hw_init(void) {

  ctl_addr = 0;
  dma_reg_addr = NULL;
  pwm_reg_addr = NULL;
  clk_reg_addr = NULL;

  printk(KERN_INFO "DMA Channel:   %5d\n", DMA_CHAN_NUM);
  printk(KERN_INFO "PWM frequency: %5d Hz\n", 1000000/CYCLE_TIME_US);

#define CHECK_MEM(x) if(!(x)) { memory_cleanup(); return -ENOMEM; }

  CHECK_MEM(dma_reg_addr = memremap(DMA_CHAN_BASE, DMA_CHAN_SIZE, MEMREMAP_WT));
  CHECK_MEM(pwm_reg_addr = memremap(PWM_BASE, PWM_LEN, MEMREMAP_WT));
  CHECK_MEM(clk_reg_addr = memremap(CLK_BASE, CLK_LEN, MEMREMAP_WT));
  CHECK_MEM(ctl_addr = __get_free_pages(GFP_KERNEL, get_page_order(NUM_PAGES)));

#undef CHECK_MEM

  dma_reg = dma_reg_addr;
  pwm_reg = pwm_reg_addr;
  clk_reg = clk_reg_addr;

/*
  unsigned long __get_free_pages(gfp_t gfp_mask, unsigned int order)
  GFP_KERNEL

  void free_pages(unsigned long addr, unsigned int order)

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
  memory_cleanup();
}

void hw_update(void) {
  // TODO
}
