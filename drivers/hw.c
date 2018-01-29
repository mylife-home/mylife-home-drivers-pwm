// https://github.com/sarfata/pi-blaster/blob/master/pi-blaster.c
// https://github.com/richardghirst/PiBits/blob/master/ServoBlaster/kernel/servoblaster.c
// http://www.valvers.com/wp-content/uploads/2013/01/arm-c-virtual-addresses.jpg
// https://www.kernel.org/doc/gorman/html/understand/understand009.html

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "common.h"

// TODO: use device tree : xxd /proc/device-tree/soc/ranges (2nd word)
#if   defined (MYLIFE_ARCH_RPI1)
#define IO_PHYS_BASE 0x20000000
#elif defined (MYLIFE_ARCH_RPI2)
#define IO_PHYS_BASE 0x3f000000
#else
#error "Unknown arch"
#endif

#define IO_BUS_BASE   0x7e000000

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

#define NUM_PAGES ((sizeof(struct ctl) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define PWM_OFFSET         0x0020C000
#define PWM_LEN            0x28
#define PWM_PHYS_BASE      (IO_PHYS_BASE + PWM_OFFSET)
#define PWM_BUS_BASE       (IO_BUS_BASE  + PWM_OFFSET)

#define PWM_CTL            0x00
#define PWM_STA            0x04
#define PWM_DMAC           0x08
#define PWM_RNG1           0x10
#define PWM_FIFO           0x18

#define CLK_OFFSET         0x00101000
#define CLK_LEN            0xA8
#define CLK_PHYS_BASE      (IO_PHYS_BASE + CLK_OFFSET)

#define CLK_CNTL           40
#define CLK_DIV            41

#define GPIO_OFFSET        0x00200000
#define GPIO_LEN           0x100
#define GPIO_BUS_BASE      (IO_BUS_BASE + GPIO_OFFSET)
#define GPCLR0             0x28
#define GPSET0             0x1c

#define DMA_PHYS_BASE      (IO_PHYS_BASE + 0x00007000)
#define DMA_CHAN_NUM       14    // the DMA Channel we are using, NOTE: DMA Ch 0 seems to be used by X... better not use it ;)
#define DMA_CHAN_SIZE      0x100 // size of register space for a single DMA channel
#define DMA_CHAN_MAX       14    // number of DMA Channels we have... actually, there are 15... but channel fifteen is mapped at a different DMA_PHYS_BASE, so we leave that one alone
#define DMA_PHYS_CHAN_BASE (DMA_PHYS_BASE + DMA_CHAN_NUM * DMA_CHAN_SIZE)

#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP       (1<<3)
#define DMA_D_DREQ          (1<<6)
#define DMA_PER_MAP(x)      ((x)<<16)
#define DMA_END             (1<<1)
#define DMA_RESET           (1<<31)
#define DMA_INT             (1<<2)

static unsigned long ctl_addr;
static void *dma_reg;
static void *clk_reg;
static void *pwm_reg;

static unsigned int get_page_order(unsigned int page_count);
static void memory_cleanup(void);
static void write_reg(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value);
static void write_reg_and_wait(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value, unsigned long usecs);
static void init_ctrl_data(void);
static void init_hardware(void);

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

  if(dma_reg) {
    memunmap(dma_reg);
    dma_reg = NULL;
  }

  if(pwm_reg) {
    memunmap(pwm_reg);
    pwm_reg = NULL;
  }

  if(clk_reg) {
    memunmap(clk_reg);
    clk_reg = NULL;
  }
}

inline void write_reg(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value) {
  volatile char *addr = reg_base_addr;
  addr += reg_offset;
  * ((volatile uint32_t *)addr) = value;
}

void write_reg_and_wait(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value, unsigned long usecs) {
  write_reg(reg_base_addr, reg_offset, value);
  udelay(usecs);
}

int hw_init(void) {

  ctl_addr = 0;
  dma_reg = NULL;
  pwm_reg = NULL;
  clk_reg = NULL;

  printk(KERN_INFO "DMA Channel:   %5d\n", DMA_CHAN_NUM);
  printk(KERN_INFO "PWM frequency: %5d Hz\n", 1000000 / CYCLE_TIME_US);

#define CHECK_MEM(x) if(!(x)) { memory_cleanup(); return -ENOMEM; }

  CHECK_MEM(dma_reg = memremap(DMA_PHYS_CHAN_BASE, DMA_CHAN_SIZE, MEMREMAP_WT));
  CHECK_MEM(pwm_reg = memremap(PWM_PHYS_BASE, PWM_LEN, MEMREMAP_WT));
  CHECK_MEM(clk_reg = memremap(CLK_PHYS_BASE, CLK_LEN, MEMREMAP_WT));
  CHECK_MEM(ctl_addr = __get_free_pages(GFP_KERNEL, get_page_order(NUM_PAGES)));

#undef CHECK_MEM

/*
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
*/
  memory_cleanup();
}

void hw_update(void) {
  // TODO
}

void init_ctrl_data(void) {

  struct ctl *ctl = (struct ctl *)ctl_addr;
  struct dma_cb *cbp = ctl->cb;
  int i;

  memset(ctl->sample, 0, sizeof(ctl->sample));

  for (i = 0; i < NUM_SAMPLES; i++) {
    ctl->sample[i] = 0;
  }

  /* Initialize all the DMA commands. They come in pairs.
   *  - 1st command copies a value from the sample memory to a destination
   *    address whichis gpclr0 register
   *  - 2nd command waits for a trigger from an external source (PWM)
   */
  for (i = 0; i < NUM_SAMPLES; i++) {

    // First DMA command
    cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
    cbp->src = virt_to_phys(ctl->sample + i);
    cbp->dst = GPIO_BUS_BASE + GPCLR0;
    cbp->length = sizeof(uint32_t);
    cbp->stride = 0;
    cbp->next = virt_to_phys(cbp + 1);
    cbp++;

    // Second DMA command
    cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
    cbp->src = virt_to_phys(ctl); // Any data will do
    cbp->dst = PWM_BUS_BASE + PWM_FIFO;
    cbp->length = sizeof(uint32_t);
    cbp->stride = 0;
    cbp->next = virt_to_phys(cbp + 1);
    ++cbp;
  }

  // point to the first
  --cbp;
  cbp->next = virt_to_phys(ctl->cb);
}

void init_hardware(void) {

  struct ctl *ctl = (struct ctl *)ctl_addr;

  // Initialize PWM
  write_reg_and_wait(pwm_reg, PWM_CTL, 0, 10);
  write_reg_and_wait(clk_reg, CLK_CNTL, 0x5A000006, 100); // Source=PLLD (500MHz)
  write_reg_and_wait(clk_reg, CLK_DIV, 0x5A000000 | (500<<12), 100); // set pwm div to 500, giving 1MHz
  write_reg_and_wait(clk_reg, CLK_CNTL, 0x5A000016, 100); // Source=PLLD and enable
  write_reg_and_wait(pwm_reg, PWM_RNG1, SAMPLE_US, 10);
  write_reg_and_wait(pwm_reg, PWM_DMAC, PWMDMAC_ENAB | PWMDMAC_THRSHLD, 10);
  write_reg_and_wait(pwm_reg, PWM_CTL, PWMCTL_CLRF, 10);
  write_reg_and_wait(pwm_reg, PWM_CTL, PWMCTL_USEF1 | PWMCTL_PWEN1, 10);

  // Initialize the DMA
  write_reg_and_wait(dma_reg, DMA_CS, DMA_RESET, 10);
  write_reg(dma_reg, DMA_CS, DMA_INT | DMA_END);
  write_reg(dma_reg, DMA_CONBLK_AD, virt_to_phys(ctl->cb));
  write_reg(dma_reg, DMA_DEBUG, 7); // clear debug error flags
  write_reg(dma_reg, DMA_CS, 0x10880001); // go, mid priority, wait for outstanding writes
}
