#ifndef __MYLIFE_DMA_PWM_HW_H__
#define __MYLIFE_DMA_PWM_HW_H__

extern int hw_init(void);
extern void hw_exit(void);
extern void hw_update(int wait);
extern void hw_dump(void);

#endif // __MYLIFE_DMA_PWM_HW_H__
