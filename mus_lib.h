
/* SELF NOTES */
//Check if the 16-bit registers were altered by the 32-bit pointer
#ifndef __mus_lib_h
#define __mus_lib_h

#include "stdlib.h"
#include "stdint.h"

/* USEFUL MACROS */

#define TRUE		1
#define FALSE		0

#define BUFFER_SIZE 6
#define UPPER_DATA_FLAG				240		//upper nibble sets set
#define MAIN_DATA_FLAG				7			//lower nibble bits set

//bit masks
#define BIT(n)                  	( 1<<(n) )
#define BIT_SET(x, mask)        	( x |=  (mask) )
#define BIT_CLEAR(x, mask)      	( x &= ~(mask) )
#define BIT_TOGGLE(x, mask)       ( x ^=  (mask) )

//register OSting
#define REG_ADDR(x)		(x/4)

#define ACC1_ADDR 				0xA6
#define ACC2_ADDR					0x3A
#define ACC_MAX_DATA			0xFF

#define ACTIVE_THRESH			0xAUL

/* USEFUL REGISTER DEFINITIONS */

//GPIOB REGS
#define GPIOA_LOC		0x40020000UL	//A base
#define GPIOB_LOC		0x40020400UL	//B base
#define GPIOC_LOC		0x40020800UL	//C base

#define GPIO_MODE_OS (REG_ADDR(0x00))
#define GPIO_OTYPE_OS (REG_ADDR(0x04))
#define GPIO_OSPEED_OS (REG_ADDR(0x08))
#define GPIO_PUPD_OS (REG_ADDR(0x0C))
#define GPIO_AFL_OS (REG_ADDR(0x20))
#define GPIO_AFH_OS (REG_ADDR(0x24))

#define GPIOA_ACCESS   ((GPIO_REG *) GPIOA_LOC)
#define GPIOB_ACCESS   ((GPIO_REG *) GPIOB_LOC)
#define GPIOC_ACCESS   ((GPIO_REG *) GPIOC_LOC)

//POWER REGS
#define PWR_LOC		0x40007000UL	//base
#define PWR_CTL_OS (REG_ADDR(0x00))

#define PWR_ACCESS			((PWR_REG*) PWR_LOC)

//CLOCK REGS
#define RCC_LOC		0x40023800UL	//base
#define RCC_CTL_OS (REG_ADDR(0x00))
#define RCC_PLL_CFG_OS (REG_ADDR(0x04))
#define RCC_CFG_OS (REG_ADDR(0x08))
#define RCC_AHB1_OS (REG_ADDR(0x30))
#define RCC_APB1_OS (REG_ADDR(0x40))

#define RCC_ACCESS		((RCC_REG*) RCC_LOC)

//I2C REGS
#define I2C1_LOC			0x40005400UL	//base

#define I2C_CFG_OS		(REG_ADDR(0x0))
#define I2C_CFG2_OS		(REG_ADDR(0x04))
#define I2C_CTL_OS		(REG_ADDR(0x1C))
#define I2C_TRISE_OS	(REG_ADDR(0x20))

//UART REGS
#define UART4_LOC			0x40004C00	//base
#define UART_DR_LOC			0x40004C04
#define UART_C1_OS		(REG_ADDR(0x0C))
#define UART_C2_OS		(REG_ADDR(0x10))
#define UART_C3_OS		(REG_ADDR(0x14))
#define UART_S_OS			(REG_ADDR(0x00))
#define UART_BR_OS		(REG_ADDR(0x08))

#define UART_ACCESS   ((UART_REG *) UART4_LOC)

//DMA REGS
#define DMA_LOC				0x40026000UL		//base
#define DMA_STREAM4		0x60UL			//0x18 * 4
#define DMA_LIS_OS		(REG_ADDR(0x00))
#define DMA_LISC_OS		(REG_ADDR(0x08))
#define DMA_STREAM4_CFG_OS		(REG_ADDR(0x10 + DMA_STREAM4))
#define DMA_STREAM4_PAR_OS		(REG_ADDR(0x18 + DMA_STREAM4))
#define DMA_STREAM4_MAR_OS		(REG_ADDR(0x1C + DMA_STREAM4))
#define DMA_STREAM4_NDT_OS		(REG_ADDR(0x14 + DMA_STREAM4))

//EXTI REGS
#define EXTI_LOC 			0x40013C00UL
#define EXTI_RTS_OS		(REG_ADDR(0x08))

#define EXTI_ACCESS   ((EXTI_REG *) EXTI_LOC)

//SYSCFG REGS
#define SYSCFG_LOC						0x40013800UL
#define SYSCFG_EXTICR1_OS			(REG_ADDR(0x08))
#define SYSCFG_EXTICR2_OS			(REG_ADDR(0x0C))

//NVIC REGS
#define NVIC_LOC 				0xE000E100UL
#define NVIC_ISE0_OS  	(REG_ADDR(0x00))
#define NVIC_ISE1_OS  	(REG_ADDR(0x06))
#define NVIC_ISE2_OS  	(REG_ADDR(0x0B))
#define NVIC_IP_OS			(REG_ADDR(0x302))	
#define NVIC_ICP0_OS		(REG_ADDR(0x180))
#define NVIC_ICP1_OS		(REG_ADDR(0x184))
#define NVIC_ICP2_OS		(REG_ADDR(0x188))

#define ISE_ACCESS   ((NVIC_ISE *) NVIC_LOC + NVIC_ISE0_OS)
#define ICP_ACCESS   ((NVIC_ICP *) NVIC_LOC + NVIC_ICP0_OS)

//TIMER REGS
#define TIMER4_LOC			0x40000800UL
#define TIMER3_LOC			0x40000400UL
#define TIMER_CTL_OS		(REG_ADDR(0x00))
#define TIMER_CNT_OS		(REG_ADDR(0x24))
#define TIMER_PSC_OS		(REG_ADDR(0x28))
#define TIMER_AR_OS			(REG_ADDR(0x2C))
#define TIMER_CCM_OS		(REG_ADDR(0x18))
#define TIMER_CCR_OS		(REG_ADDR(0x34))
#define TIMER_EG_OS			(REG_ADDR(0x14))
#define TIMER4_PRELOAD	0xC350
#define TIMER3_PRELOAD	0x2A2

#define TIMER3_ACCESS   ((TIMER_REG *) TIMER3_LOC)
#define TIMER4_ACCESS   ((TIMER_REG *) TIMER4_LOC)

//FLASH MEM REGS
#define ANGLE_DATA_LOC			0x08010000UL

#define SRAM_LOC						0x20000000UL
#define sram_index(x)				(*((volatile float*) (SRAM_LOC + (4*x))))

/* GLOBAL TYPES  */

typedef enum
{
	ACTIVE,
	STANDBY,
	CALIBRATING,
	ERR
} program_state;

typedef struct
{
	unsigned char sysclk_err	: 3;
	unsigned char pwr_err 		: 3;
	unsigned char uart_err		: 3;
	unsigned char dma_err 		: 3;
	unsigned char exti_err		: 3;
	unsigned char i2c_err			: 3;
	unsigned char user_err 		: 3;
	unsigned char nvic_err		: 3;
} err_bits;

typedef struct
{
  volatile uint32_t CR1; 
  volatile uint32_t CR2;    
  volatile uint32_t SMCR;  
  volatile uint32_t DIER;   
  volatile uint32_t SR;     
  volatile uint32_t EGR;    
  volatile uint32_t CCMR1;      
  volatile uint32_t CCMR2;  
  volatile uint32_t CCER;    
  volatile uint32_t CNT;  
  volatile uint32_t PSC;
  volatile uint32_t ARR;
} TIMER_REG;

typedef struct
{
  volatile uint32_t MODE; 
  volatile uint32_t OTYPE;    
  volatile uint32_t SPEED;  
  volatile uint32_t PUPD;   
  volatile uint32_t ID;     
  volatile uint32_t OD;    
  volatile uint32_t BSR;      
} GPIO_REG;

typedef struct
{
  volatile uint32_t SR; 
  volatile uint32_t DR;    
  volatile uint32_t BRR;  
  volatile uint32_t CR1;       
} UART_REG;

typedef struct
{
	volatile uint32_t ICP0;
	volatile uint32_t ICP1;
	volatile uint32_t ICP2;
} NVIC_ICP;

typedef struct
{
	volatile uint32_t ISE0;
	volatile uint32_t ISE1;
	volatile uint32_t ISE2;
} NVIC_ISE;

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_REG;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLL;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RS;
	volatile uint32_t AHB2RS;
	volatile uint32_t AHB3RS;
	volatile uint32_t RES0;
	volatile uint32_t APB1RS;
	volatile uint32_t APB2RS;
	volatile uint32_t RES1;
	volatile uint32_t RES2;
	volatile uint32_t AHB1E;
	volatile uint32_t AHB2E;
	volatile uint32_t AHB3E;
	volatile uint32_t RES3;
	volatile uint32_t APB1E;
	volatile uint32_t APB2E;
} RCC_REG;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CSR;
} PWR_REG;

/* FUNC PROTOTYPES */
void state_update_task(void*);
void compute_main_theta_task(void*);
void compute_upper_theta_task(void*);
void send_main_data_task(void*);
void send_upper_data_task(void*);
void alert_task(void*);

float get_mag(int8_t, int8_t);

//power control
void enter_standby(void);
void exit_standby(void);

//ISRs
void EXTI2_IRQHandler(void);
void EXTI4_IRQHandler(void);

//inits
void SysClock_Init(void);
void I2C_Init(void);
void DMA_Init(void);
void UART_Init(void);
void EXTI_Init(void);
void mag_lut_init(void);
err_bits Check_SysClock(void);
err_bits Check_I2C(void);
err_bits Check_DMA(void);
err_bits Check_UART(void);
err_bits Check_EXTI(void);
void Err_Handler(void);

static inline int8_t get_sign(uint16_t x)
{
	return (x > 0) - (x < 0);
}

static inline uint16_t swp_nibb_16(uint16_t x)
{
	return ((x & 0xFF) << 8 | (x & 0xFF) >> 8);
}

#endif		//__mus_lib_h