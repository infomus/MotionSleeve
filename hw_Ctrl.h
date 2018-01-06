
#ifndef HW_CTRL_H
#define HW_CTRL_H

// Initilizations
void hw_Init(void);
void SysClock_Init(void);
void I2C_Init(void);
void DMA_Init(void);
void UART_Init(void);
void EXTI_Init(void);
void PWM_Init(void);

// Checks
int hw_Check_Error(void);
err_bits Check_SysClock(void);
err_bits Check_I2C(void);
err_bits Check_DMA(void);
err_bits Check_UART(void);
err_bits Check_EXTI(void);
err_bits Check_PWM(void);

// Handlers
void EXTI2_IRQHandler(void);
void EXTI4_IRQHandler(void);

#endif
