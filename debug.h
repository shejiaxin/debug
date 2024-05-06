#ifndef __FISH_H
#define __FISH_H
#include "includes.h"

#define FISH_STK_SIZE            300
#define FISH_TASK_PRIO           16

#define MAX_BUF_LEN 50
typedef struct
{
    char  FishRxBuf[MAX_BUF_LEN];
//    char  FishTxBuf[MAX_BUF_LEN];
    uint16_t rx_size;
}FISH_st;

typedef enum
{
    SYS_MODE_PASS = 0,
    SYS_MODE_FISH
}sys_mode_t;

extern FISH_st fishcom;

extern OS_SEM   fish_recv_sem;

void fish_hw_init(void);
void fish_create(void);
void fish_print(const char *fmt, ...);
#ifndef  USE_SERIAL_TASK_PRINTF  
void debug_print(const char *fmt, ...);
#endif
void debug_send(char *buf, uint16_t len);
void DispTaskInfo(void);
void FishCommRecvProc(uint8_t rx_data);
#endif
