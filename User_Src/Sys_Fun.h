#ifndef _SYS_FUN_H_
#define _SYS_FUN_H_

void NVIC_INIT(void);
void SYSTICK_INIT(void);
uint32_t GET_NOWTIME(uint32_t * lasttime);
void EE_INIT(void);
void EE_SAVE_DATA(void);
void EE_READ_DATA(void);

#endif

