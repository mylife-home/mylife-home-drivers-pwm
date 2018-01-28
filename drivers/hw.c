// https://github.com/sarfata/pi-blaster/blob/master/pi-blaster.c

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gpio.h>

#include "common.h"

#define PERI_BASE     BCM2708_PERI_BASE

#define CYCLE_TIME_US 10000
#define SAMPLE_US     10
#define NUM_SAMPLES   (CYCLE_TIME_US / SAMPLE_US)
#define NUM_CBS       (NUM_SAMPLES * 2)
#define NUM_PAGES     ((NUM_CBS * sizeof(dma_cb_t) + NUM_SAMPLES * 4 + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define DMA_CHAN_NUM  14    // the DMA Channel we are using, NOTE: DMA Ch 0 seems to be used by X... better not use it ;)
#define DMA_CHAN_SIZE 0x100 // size of register space for a single DMA channel
#define DMA_CHAN_MAX  14    // number of DMA Channels we have... actually, there are 15... but channel fifteen is mapped at a different DMA_BASE, so we leave that one alone
#define DMA_CHAN_BASE (DMA_BASE + DMA_CHAN_NUM * DMA_CHAN_SIZE)

#define PWM_BASE      (PERI_BASE + 0x20C000)
#define PWM_LEN       0x28
#define CLK_BASE      (PERI_BASE + 0x101000)
#define CLK_LEN       0xA8

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

  /* Use the mailbox interface to the VC to ask for physical memory */
  mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * PAGE_SIZE, PAGE_SIZE, mem_flag);
  /* TODO: How do we know that succeeded? */
  dprintf("mem_ref %u\n", mbox.mem_ref);
  mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
  dprintf("bus_addr = %#x\n", mbox.bus_addr);
  mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * PAGE_SIZE);
  dprintf("virt_addr %p\n", mbox.virt_addr);

  if ((unsigned long)mbox.virt_addr & (PAGE_SIZE-1))
    fatal("pi-blaster: Virtual address is not page aligned\n");

  /* we are done with the mbox */
  mbox_close(mbox.handle);
  mbox.handle = -1;

  init_ctrl_data();
  init_hardware();
  init_channel_pwm();
  // Init pin2gpio array with 0/false values to avoid locking all of them as PWM.
  init_pin2gpio();
  // Only calls update_pwm after ctrl_data calculates the pin mask to unlock all pins on start.

  hw_update();

  return 0;
}

void hw_exit(void) {
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
      /* we need to reopen mbox file */
      mbox.handle = mbox_open();
    }
    mem_unlock(mbox.handle, mbox.mem_ref);
    mem_free(mbox.handle, mbox.mem_ref);
    mbox_close(mbox.handle);
  }

  unmemremap(dma_reg);
  unmemremap(pwm_reg);
  unmemremap(clk_reg);
}

void hw_update(void) {
  // TODO
}
