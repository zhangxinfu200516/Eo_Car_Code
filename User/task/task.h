#ifndef TASK_H
#define TASK_H
#include <stdint.h>
#ifdef __cplusplus 
extern "C" {
#endif
extern void Task_Init(void);
extern void Task_Loop(void);
extern void Task_UART1_Callback(uint8_t *Buffer, uint16_t Length);
extern void Task_UART2_Callback(uint8_t *Buffer, uint16_t Length);
#ifdef __cplusplus
}
#endif
#endif // TASK_H