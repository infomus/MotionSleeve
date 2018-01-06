
#include "hw_Ctrl.h"

static err_bits hw_error_flags;

void hw_Init()
{
	I2C_Init();
	DMA_Init();
	UART_Init();
	EXTI_Init();
}

int hw_Check_Err()
{
	Check_SysClock();
	Check_I2C();
	Check_DMA();
	Check_UART();

	return hw_error_flags ? 0 : 1;
}


void SysClock_Init(void)
{
	//HSI drives PLL which drives sysclk
	volatile uint32_t *reg_ptr = (uint32_t *) PWR_LOC;
	*(reg_ptr + PWR_CTL_OS) |= (0x3<<14);		//power in normal mode
	
	reg_ptr = (uint32_t *) RCC_LOC;
	*(reg_ptr + RCC_CTL_OS) &= ~BIT(24);	//disable main PLL before configuring it (sets voltage regulator scale 3)
	*(reg_ptr + RCC_PLL_CFG_OS) &= ~BIT(22);	//select HSI as PLL source
	*(reg_ptr + RCC_CTL_OS) |= BIT(1);				//enable HSI
	
	//Need clock freq of 84Mhz and VCO freq of 336Mhz
	*(reg_ptr + RCC_PLL_CFG_OS) &= ((0x10) | (0x2<<2) | (0x150<<6) | ~(1<<22) | (0x22<<24)); 	//set P, N, M and Q prescalar values
	
	*(reg_ptr + RCC_CFG_OS) &= 	((0x2<<2) | (0x0<<4) | (0x8<<10) | (0x0<<13)); 		//set PLL as sysclk source and set clock divisions for normal, low and high speed
	
	*(reg_ptr + RCC_CTL_OS) |= BIT(24);	//enable main PLL
	
}

err_bits Check_SysClock()
{
	//volatile uint32_t *reg_ptr = (uint32_t *) RCC_LOC;
	//TODO
}

void I2C_Init(void)
{
	volatile uint32_t *reg_ptr = (uint32_t *) RCC_LOC;
	
	*(reg_ptr + RCC_AHB1_OS) |= (BIT(1) | BIT(2));				//Enable GPIO(B,C) Clocks
	*(reg_ptr + RCC_APB1_OS) |= BIT(21);		//Enable I2C1 Clock
	
	/* GPIOB CONFIG */
	reg_ptr = (uint32_t *) GPIOB_LOC;				
	
	*(reg_ptr + GPIO_MODE_OS) &= ~((0xFU<<12));	//clear before updating
	*(reg_ptr + GPIO_MODE_OS) |= ((0xAU<<12));		//set pins 6, 7 to alt func
	*(reg_ptr + GPIO_OTYPE_OS) |= (BIT(6) | BIT(7));			//set as open-drain
	*(reg_ptr + GPIO_OSPEED_OS) &= ~((0xFU<<12));	//clear before updating
	*(reg_ptr + GPIO_OSPEED_OS) |= ((0xFU<<12));		//set to high speed (will get overriden by alt func anyway)
	*(reg_ptr + GPIO_PUPD_OS) &= ~((0xFU<<12));		//clear before updating
	*(reg_ptr + GPIO_PUPD_OS) |= ~((0xAU<<12));		//set as pull-ups
	*(reg_ptr + GPIO_AFL_OS) &= ~(0xFFU<<24);		//set pins 6, 7 (SCL and SDA respectively)
	*(reg_ptr + GPIO_AFL_OS) |= (0x44U<<24);
	
	/* I2C1 CONFIG */
	//NOTE: I2C1 driven by APB1 (half speed of main PLL)
	reg_ptr = (uint32_t *) I2C1_LOC;
	
	*(reg_ptr + I2C_CFG_OS) &= ~BIT(0);		//disable I2C before configuring
	*(reg_ptr + I2C_CFG2_OS) &= ~0x3F;		//clear frequency config
	*(reg_ptr + I2C_CFG2_OS) |= 0x2;
	*(reg_ptr + I2C_CTL_OS) &= ~0xFFF;	//standard mode
	*(reg_ptr + I2C_CTL_OS) |= 0x50;	//clock control
	*(reg_ptr + I2C_CTL_OS) &= ~BIT(15);	//standard mode
	*(reg_ptr + I2C_TRISE_OS) &= ~0x3F;		//clear rise time
	*(reg_ptr + I2C_TRISE_OS) |= 0x3;		//set rise time
	
	*(reg_ptr + I2C_CFG_OS) |= (BIT(10) & BIT(0));	//enable I2C and ACK
	
	//*(reg_ptr + I2C_CFG2_OS) 
}

err_bits Check_I2C(void)
{
	//volatile uint32_t *reg_ptr = (uint32_t *) I2C1_LOC;
	//TODO
}

void DMA_Init(void)
{
	volatile uint32_t *reg_ptr = (uint32_t *) RCC_LOC;
	*(reg_ptr + RCC_AHB1_OS) |= BIT(21);				//Enable DMA Clock
	
	reg_ptr = (uint32_t *) DMA_LOC;
	*(reg_ptr + DMA_STREAM4_CFG_OS) &= ~(BIT(0));			//disable target stream before configuring
	while ((*(reg_ptr + DMA_STREAM4_CFG_OS)) & BIT(0))	//wait until disabled
	*(reg_ptr + DMA_STREAM4_PAR_OS) = UART_DR_LOC;		//set address to send to
	*(reg_ptr + DMA_STREAM4_MAR_OS) = ANGLE_DATA_LOC;		//set address to send from
	*(reg_ptr + DMA_STREAM4_NDT_OS) |= 0xFF;				//number of bytes to send
	*(reg_ptr + DMA_STREAM4_CFG_OS) &= ~(0x7U<<25);	//clear
	*(reg_ptr + DMA_STREAM4_CFG_OS) |= (0x4U<<25);	//channel 4
	*(reg_ptr + DMA_STREAM4_CFG_OS) |= (0x3U<<16);	//high priority
	*(reg_ptr + DMA_STREAM4_CFG_OS) &= ~(0xFFFF);		//clear lower nibble
	//transfer error only, transfer complete int, memory to peripheral, 
	//no circular, memory inc only, periph and memory byte sized
	*(reg_ptr + DMA_STREAM4_CFG_OS) |= (0x454);	
}

err_bits Check_DMA(void)
{
	//volatile uint32_t *reg_ptr = (uint32_t *) DMA_LOC;
	//TODO
}

void UART_Init(void)
{
	volatile uint32_t *reg_ptr = (uint32_t *) RCC_LOC;
	*(reg_ptr + RCC_AHB1_OS) |= BIT(19);		//Enable UART4
	
	reg_ptr = (uint32_t *) GPIOC_LOC;
	*(reg_ptr + GPIO_MODE_OS) &= ~((0x3U<<20));	//clear before updating
	*(reg_ptr + GPIO_MODE_OS) |= ((0x2<<20));		//set pin 10 to alt func
	*(reg_ptr + GPIO_OTYPE_OS) |= (BIT(10));			//set as open-drain
	*(reg_ptr + GPIO_OSPEED_OS) &= ~((0x3U<<20));	//clear before updating
	*(reg_ptr + GPIO_OSPEED_OS) |= ((0x2U<<20));		//set to high speed (will get overriden by alt func anyway)
	*(reg_ptr + GPIO_PUPD_OS) &= ~((0x3U<<20));		//clear before updating
	*(reg_ptr + GPIO_PUPD_OS) |= ((0x1U<<20));		//set as pull-up
	*(reg_ptr + GPIO_AFH_OS) &= ~(0xFU<<8);		//set pin 10 to AF8 (UART4)
	*(reg_ptr + GPIO_AFH_OS) |= (0x8U<<8);
	
	reg_ptr = (uint32_t *) UART4_LOC;
	*(reg_ptr + UART_C1_OS) |= BIT(13);		//enable UART
	*(reg_ptr + UART_C1_OS) &= ~BIT(12);		//1 start, 8 data
	*(reg_ptr + UART_C2_OS) &= ~ (BIT(12) | BIT(13));	// 1 stop
	*(reg_ptr + UART_C3_OS) &= ~ (BIT(9) | BIT(8));		//no ctl flow
	//*(reg_ptr + UART_C3_OS) |= BIT(7);	//enable DMA transmitter
	*(reg_ptr + UART_C3_OS) |= BIT(3);		//Half duplex
	/** 115200 BAUD RATE (calc depends on pclk, oversample (default = 0) and two dividers)**/
	*(reg_ptr + UART_BR_OS) &= ~ (BIT(9) | BIT(8));
	
}

err_bits Check_UART(void)
{
	
}

void EXTI_Init(void)
{
	volatile uint32_t *reg_ptr = (uint32_t *) GPIOC_LOC;
	*(reg_ptr + GPIO_MODE_OS) &= (0xC00U);	//set as input
	*(reg_ptr + GPIO_PUPD_OS) &= (0xC00U);	//no pull up or pull down
	
	reg_ptr = (uint32_t *) GPIOB_LOC;
	*(reg_ptr + GPIO_MODE_OS) &= (0xC0U);	//set as input
	*(reg_ptr + GPIO_PUPD_OS) &= (0xC0U);	//no pull up or pull down
	
	reg_ptr = (uint32_t *) SYSCFG_LOC;
	*(reg_ptr + SYSCFG_EXTICR1_OS) &= ~0xFFFF;		//clear
	*(reg_ptr + SYSCFG_EXTICR1_OS) |= 0x0100;			//interrupt signal from PB2
	*(reg_ptr + SYSCFG_EXTICR2_OS) &= ~0xFFFF;		//clear
	*(reg_ptr + SYSCFG_EXTICR2_OS) |= 0x0200;			//interrupt signal from PC4
	
	reg_ptr = (uint32_t *) EXTI_LOC;
	*(reg_ptr + EXTI_RTS_OS) |= BIT(2);		//enable exti rising edge on line 2
	*(reg_ptr + EXTI_RTS_OS) |= BIT(4);		//enable exti rising edge on line 4
	
	reg_ptr = (uint32_t *) NVIC_LOC;
	*(reg_ptr + NVIC_IP_OS) &= (((0x11)<<16) & (0xF));	//set EXTI2 and EXTI4 prioties
	*(reg_ptr + NVIC_ISE0_OS) |= (BIT(8) & BIT(10));		//enable interrupts
}

err_bits Check_EXTI(void)
{
}

void PWM_Init(void)
{
		/* GPIOC CONFIG */
	volatile uint32_t *reg_ptr = (uint32_t *) GPIOC_LOC;				
	
	*(reg_ptr + GPIO_MODE_OS) &= ~((0xFU<<16) | (0xFU<<20));	//clear before updating
	*(reg_ptr + GPIO_MODE_OS) |= (0xAU<<12);		//set pins 6 to alt func
	*(reg_ptr + GPIO_OTYPE_OS) &= ~(0xFU<<6);									//no types
	*(reg_ptr + GPIO_OSPEED_OS) &= ~(0xFU<<12);	//clear before updating
	*(reg_ptr + GPIO_OSPEED_OS) |= (0x5U<<12);		//set to normal speed (will get overriden by alt func)
	*(reg_ptr + GPIO_PUPD_OS) &= ~(0xFU<<12);	//no pull up or pull down
	*(reg_ptr + GPIO_AFL_OS) &= (~0xFU<<24);		//clear before updating
	*(reg_ptr + GPIO_AFL_OS) |= (0x4U<<24);		//set pin 6 to alt func 4
	
	/* PWM CONFIG (HELL) */
	volatile uint16_t *halfw_reg_ptr = (uint16_t *) TIMER3_LOC;
	*(halfw_reg_ptr + TIMER_CTL_OS) &= ~BIT(0);	//disable tim
	*(halfw_reg_ptr + TIMER_CCM_OS) &= 0xFCFF;	//pwm output mode
	*(halfw_reg_ptr + TIMER_CCM_OS) &= ~BIT(3);	//preload disable (constant duty cycle)
	*(halfw_reg_ptr + TIMER_CCM_OS) |= 0xF0;		//pwm mode 1
	*(halfw_reg_ptr + TIMER_CCM_OS) |= BIT(7);	//auto clear on high level
	*(halfw_reg_ptr + TIMER_PSC_OS) = 0x347;		//sync to us (84Mz/8399)
	*(halfw_reg_ptr + TIMER_CCR_OS) = (TIMER3_PRELOAD>>1);		//50% duty cycle
	*(halfw_reg_ptr + TIMER_AR_OS) |= TIMER3_PRELOAD;		//1kHz arbitrary freq
	*(halfw_reg_ptr + TIMER_EG_OS) |= BIT(0);		//update generation
	*(halfw_reg_ptr + TIMER_CTL_OS) &= 0xFDF;	//center-aligned; triggered only on up counting
}

err_bits Check_PWM(void)
{
	// TODO
}

void TIMER4_Init(void)
{
	volatile uint16_t *reg_ptr = (uint16_t *) TIMER4_LOC;
	*(reg_ptr + TIMER_CTL_OS) &= ~BIT(0);	//disable
	*(reg_ptr + TIMER_CTL_OS) |= ~BIT(4);		//count up mode
	*(reg_ptr + TIMER_PSC_OS) = 0x347; 		//sync to us (84Mz / 8399)
	*(reg_ptr + TIMER_AR_OS) = TIMER4_PRELOAD;		//50ms
	*(reg_ptr + TIMER_CTL_OS) |= BIT(0);	//enable
}

void EXTI2_IRQHandler(void)
{
	if (!taps) //first double tap
	{
		TIMER4_ACCESS->CNT = 0x00;
	}
	else //second double tap
	{
		if (TIMER4_ACCESS->CNT < TIMER4_PRELOAD) ///valid quad tap
		{
			if (TIMER4_ACCESS->SR & BIT(5)) error_flags.exti_err = 2;
			
			if (CURR_STATE != CALIBRATING) 
			{
				for (int i=0; i< ((sizeof(accs)/sizeof(acc_unit))); i++)
				{
					accs[i].meta_data.is_calibrated = FALSE;
				}
				
				CURR_STATE = CALIBRATING;
			}
			
			taps = 0;
		}
		else	//50ms has elapsed
		{
			TIMER4_ACCESS->CNT = 0x00;		//reset timer
		}
	}
	
	//clear pending
	EXTI_ACCESS->PR |= BIT(4);
	ICP_ACCESS->ICP0 |= BIT(8);
}

void EXTI4_IRQHandler(void)
{
	if (CURR_STATE != STANDBY)
	{
		xTaskResumeFromISR(alert_task);
		
		vTaskSuspend(send_main_data_task);
		vTaskSuspend(send_upper_data_task);
		
		CURR_STATE = STANDBY;
	}
	
	//clear pending
	EXTI_ACCESS->PR |= BIT(6);
	ICP_ACCESS->ICP0 |= BIT(10);
}