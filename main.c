
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "mus_lib.h"
#include "acc_mod.h"
#include "hw_Ctrl.h"

static program_state CURR_STATE;
static err_bits error_flags;

extern inline int8_t get_sign(uint16_t);
extern inline uint16_t swp_nibb_16(uint16_t);

static acc_unit accs[2];

SemaphoreHandle_t pwr_mutex;
SemaphoreHandle_t uart_semaphore;
QueueHandle_t main_angle_q;
QueueHandle_t upper_angle_q;

static uint8_t taps;
	
int main(void)
{
	CURR_STATE = ACTIVE;
	taps = 0;
	
	SysClock_Init();
	
	hw_Init();

	if (hw_Check_Error())
	{
		Err_Handler();
	}


	if (!create_acc(accs, ACC1_ADDR, TRUE) || 
			!create_acc(&accs[1], ACC2_ADDR, FALSE)) 
	{
		error_flags.user_err =2;
		Err_Handler();
	}
	
	acc_exec(accs, accel_init);
	acc_exec(&accs[1], accel_init);
	
	acc_exec(accs, accel_ints_init);
	
	pwr_mutex = xSemaphoreCreateMutex();
	uart_semaphore = xSemaphoreCreateBinary();
	
	if (pwr_mutex == NULL || uart_semaphore == NULL) 
	{
		error_flags.user_err = 3;
		Err_Handler();
	}
	
	main_angle_q = xQueueCreate(1, sizeof(int8_t));
	upper_angle_q = xQueueCreate(3, sizeof(int8_t));
	
	if (main_angle_q == NULL || upper_angle_q == NULL)
	{
		error_flags.user_err = 4;
		Err_Handler();
	}
	
	mag_lut_init();
	
	xTaskCreate(state_update_task, (const char*) "state_update", 1024, NULL, 5, NULL);
	xTaskCreate(compute_main_theta_task, (const char*) "main_theta", 1024, NULL, 2, NULL);
	xTaskCreate(compute_upper_theta_task, (const char*) "upper_theta", 1024, NULL, 2, NULL);
	xTaskCreate(send_main_data_task, (const char*) "send_main", 1024, NULL, 3, NULL);
	xTaskCreate(send_upper_data_task, (const char*) "send_upper", 1024, NULL, 3, NULL);
	xTaskCreate(alert_task, (const char*) "alert", 1024, NULL, 1, NULL);
	
	vTaskSuspend(alert_task);
	
	vTaskStartScheduler();
	
  while (1)
  {
  }
}

void state_update_task(void* pV)
{
	if (CURR_STATE != ACTIVE)
	{
		//inactivity that has not yet been serviced
		if (CURR_STATE == STANDBY && RCC_ACCESS->CFGR & BIT(24))
		{
			if (xSemaphoreTake(pwr_mutex, 100))
			{
				enter_standby();
				xSemaphoreGive(pwr_mutex);
			}
		}
		
		if (CURR_STATE == CALIBRATING)
		{
			for (int i=0; i< ((sizeof(accs)/sizeof(acc_unit))); i++)
			{
				if (!accs[i].meta_data.is_calibrated) break;
				
				CURR_STATE = ACTIVE;
			}
		}
		
		if (CURR_STATE == ERR) Err_Handler();
	}
	else if (!(RCC_ACCESS->CFGR & BIT(24)))
	{
		if (xSemaphoreTake(pwr_mutex, 100))
		{
			exit_standby();
			xSemaphoreGive(pwr_mutex);
		}
	}
	
	vTaskDelay(1500);
}

void compute_main_theta_task(void* pV)
{
	static uint8_t operating_acc = 0;
	static uint8_t comp_acc = 1;
	//inactivity that has not yet been serviced
	
	if (!accs[operating_acc].meta_data.is_upper_link)
	{
		if (CURR_STATE == STANDBY && RCC_ACCESS->CFGR & BIT(24))
		{
			if (xSemaphoreTake(pwr_mutex, 100))
			{
				enter_standby();
				xSemaphoreGive(pwr_mutex);
			}
		}		
		if (acc_exec(&accs[operating_acc], accel_read_multi, ACC_DATA, (uint8_t*) &accs[operating_acc].curr_data.x_data, 2) &&
				acc_exec(&accs[operating_acc], accel_read_multi, ACC_DATA, (uint8_t*) &accs[operating_acc].curr_data.z_data, 2))
		{
			angle_data angles;
			do {
				data_signs axes_signs;
				
				//inconsistent endianness
				accs[operating_acc].curr_data.x_data = swp_nibb_16(accs[operating_acc].curr_data.x_data);
				accs[operating_acc].curr_data.z_data = swp_nibb_16(accs[operating_acc].curr_data.z_data);
				
				//signs
				axes_signs.zsign = get_sign(accs[operating_acc].curr_data.z_data);
				
				//avoid divide by zero
				int8_t x_acc = (!accs[operating_acc].curr_data.x_data) ? 
					abs(accs[operating_acc].curr_data.x_data) : (float) EPSILON;
				int8_t z_acc = (!accs[operating_acc].curr_data.z_data) ? 
					abs(accs[operating_acc].curr_data.z_data) : (float) EPSILON;
				int8_t z_comp = (!accs[comp_acc].curr_data.z_data) ?
					abs(accs[comp_acc].curr_data.z_data) : (float) EPSILON;
				
				//calculate angle
				angles.yaw = 0;
				angles.pitch = (int8_t) (CONVERSION_FACTOR * atanf(axes_signs.zsign * (z_acc/get_mag(x_acc, z_comp))));
				angles.roll = 0;
				
				if (CURR_STATE == CALIBRATING && !accs[operating_acc].meta_data.is_calibrated)
				{
					//Calibrate the accelerometers
					accs[operating_acc].base_angles = angles;
					accs[operating_acc].meta_data.is_calibrated = TRUE;
				}
				
				//offset val
				angles.pitch = angles.pitch - accs[operating_acc].base_angles.pitch;
				
			} while (!xQueueSend(main_angle_q, &angles, 50));
		}
		else
		{
			//Handle error
		}
	}
	else
	{
		comp_acc = operating_acc;
		if (++operating_acc > ((sizeof(accs)/sizeof(acc_unit)) -1)) operating_acc = 0;
	}
	
	vTaskDelay(100);
}

void compute_upper_theta_task(void* pV)
{
	static uint8_t operating_acc = 0;
	//inactivity that has not yet been serviced
	
	if (accs[operating_acc].meta_data.is_upper_link)
	{
		if (CURR_STATE == STANDBY && RCC_ACCESS->CFGR & BIT(24))
		{
			if (xSemaphoreTake(pwr_mutex, 100))
			{
				enter_standby();
				xSemaphoreGive(pwr_mutex);
			}
		}
		if (acc_exec(&accs[operating_acc], accel_read_multi, ACC_DATA, (uint8_t*) &accs[operating_acc].curr_data, 6))
		{
			//yaw => anterior/posterior
			//pitch => lateral/saggital
			//roll => coronal/frontal
			angle_data angles;
			do {
				data_signs axes_signs;
				
				//inconsistent endianness
				accs[operating_acc].curr_data.x_data = swp_nibb_16(accs[operating_acc].curr_data.x_data);
				accs[operating_acc].curr_data.y_data = swp_nibb_16(accs[operating_acc].curr_data.y_data);
				accs[operating_acc].curr_data.z_data = swp_nibb_16(accs[operating_acc].curr_data.z_data);
				
				//signs
				axes_signs.xsign = get_sign(accs[operating_acc].curr_data.x_data);
				axes_signs.ysign = get_sign(accs[operating_acc].curr_data.y_data);
				axes_signs.zsign = get_sign(accs[operating_acc].curr_data.z_data);
				
				//avoid divide by zero
				int8_t x_acc = (!accs[operating_acc].curr_data.x_data) ? 
					abs(accs[operating_acc].curr_data.x_data) : (float) EPSILON;
				int8_t y_acc = (!accs[operating_acc].curr_data.y_data) ? 
					abs(accs[operating_acc].curr_data.y_data) : (float) EPSILON;
				int8_t z_acc = (!accs[operating_acc].curr_data.z_data) ? 
					abs(accs[operating_acc].curr_data.z_data) : (float) EPSILON;
				
				//calculate angles
				angles.yaw = (int8_t) (CONVERSION_FACTOR * atanf(axes_signs.ysign * (y_acc/get_mag(x_acc, z_acc))));
				angles.pitch = (int8_t) (CONVERSION_FACTOR * atanf(axes_signs.zsign * (z_acc/get_mag(x_acc, z_acc))));
				angles.roll = (int8_t) (CONVERSION_FACTOR * atanf(axes_signs.xsign * (x_acc/get_mag(y_acc, z_acc))));
				
				if (CURR_STATE == CALIBRATING && !accs[operating_acc].meta_data.is_calibrated)
				{
					//Calibrate the accelerometers
					accs[operating_acc].base_angles = angles;
					accs[operating_acc].meta_data.is_calibrated = TRUE;
				}
				
				//offset vals
				angles.yaw = angles.yaw - accs[operating_acc].base_angles.yaw;
				angles.pitch = angles.pitch - accs[operating_acc].base_angles.pitch;
				angles.roll = angles.roll - accs[operating_acc].base_angles.roll;
				
			} while (!xQueueSend(upper_angle_q, &angles, 50));
		}
		else
		{
			//Handle error
		}
	}
	else
	{
		if (++operating_acc > ((sizeof(accs)/sizeof(acc_unit)) -1)) operating_acc = 0;
	}
	
	//approx. 2/3 slower than main data acquisition
	vTaskDelay(66);
}

void send_main_data_task(void* pV)
{
	static uint8_t smooth_angle = 0;
	static int8_t buffer[BUFFER_SIZE];
	static uint8_t buffer_index = 0;
	
	uint8_t received_angle;
	
	if (xSemaphoreTake(uart_semaphore, 100) && xQueueReceive(main_angle_q, &received_angle, 1000))
	{
		if (buffer[BUFFER_SIZE-1] == NULL)	//if buffer is not full
		{
			for (uint8_t i=0; i<BUFFER_SIZE; i++)
			{
				if (buffer[i] == NULL) buffer[i] = received_angle;
				else smooth_angle += buffer[i];
				
				i%=BUFFER_SIZE;
			}
		}
		else 
		{
			smooth_angle += received_angle - buffer[0];	//shift
			
			for (uint8_t i=0; i<BUFFER_SIZE; i++)
			{
				if (i == BUFFER_SIZE -1) buffer[i] = received_angle;
				buffer[i] = buffer[i+1];
			}
			
		}
		
		smooth_angle = smooth_angle/BUFFER_SIZE;
		
		UART_ACCESS->CR1 |= BIT(3);
		while (!(UART_ACCESS->SR & BIT(7)));	//wait for idle frame
		UART_ACCESS->DR = smooth_angle;
		while (!(UART_ACCESS->SR & BIT(6)));	//transmission complete
		
		xSemaphoreGive(uart_semaphore);
		
		smooth_angle = smooth_angle*BUFFER_SIZE;		//reset to be used as running total
	}
	
	taskYIELD();
}

void send_upper_data_task(void* pV)
{
	static angle_data buffer[BUFFER_SIZE];
	static angle_data smooth_angle;
	
	angle_data received_angle;
	
	if (xSemaphoreTake(uart_semaphore, 100) && xQueueReceive(upper_angle_q, &received_angle, 1000))
	{
		if (!buffer[BUFFER_SIZE-1].yaw && !buffer[BUFFER_SIZE-1].pitch && !buffer[BUFFER_SIZE-1].roll)	//if buffer is not full
		{
			for (uint8_t i=0; i<BUFFER_SIZE; i++)
			{
				//if not yet populated
				if (!buffer[i].yaw && !buffer[i].pitch && !buffer[i].roll) buffer[i] = received_angle;
				else //add to total
				{
					smooth_angle.yaw += buffer[i].yaw;
					smooth_angle.pitch += buffer[i].pitch;
					smooth_angle.roll += buffer[i].roll;
				}
				
				i%=BUFFER_SIZE;
			}
		}
		else  //if buffer is full
		{
			//shift
			smooth_angle.yaw += received_angle.yaw - buffer[0].yaw;
			smooth_angle.pitch += received_angle.pitch - buffer[0].pitch;
			smooth_angle.roll += received_angle.roll - buffer[0].roll;
			
			for (uint8_t i=0; i<BUFFER_SIZE; i++)
			{
				if (i == BUFFER_SIZE -1) buffer[i] = received_angle;
				buffer[i] = buffer[i+1];
			}
		}
		
		smooth_angle.yaw = smooth_angle.yaw/BUFFER_SIZE;
		smooth_angle.pitch = smooth_angle.pitch/BUFFER_SIZE;
		smooth_angle.roll = smooth_angle.roll/BUFFER_SIZE;
		
		UART_ACCESS->CR1 |= BIT(3);
		while (!(UART_ACCESS->SR & BIT(7)));	//wait for idle frame
		UART_ACCESS->DR = UPPER_DATA_FLAG;
		UART_ACCESS->DR = smooth_angle.yaw;
		UART_ACCESS->DR = smooth_angle.pitch;
		UART_ACCESS->DR = smooth_angle.roll;
		while (!(UART_ACCESS->SR & BIT(6)));	//transmission complete
		
		xSemaphoreGive(uart_semaphore);
		
		//reset so it can be reused as running total
		smooth_angle.yaw = smooth_angle.yaw*BUFFER_SIZE;
		smooth_angle.pitch = smooth_angle.pitch*BUFFER_SIZE;
		smooth_angle.roll = smooth_angle.roll*BUFFER_SIZE;
	}
	
	taskYIELD();
}

void alert_task(void* pV)
{
	if (CURR_STATE == STANDBY)
	{
		GPIOC_ACCESS->BSR ^= BIT(6);		//led
		TIMER3_ACCESS->CR1 ^= BIT(0);		//buzzer
		
		vTaskDelay(3000);
	}
	else vTaskSuspend(alert_task);
	
}

float get_mag(int8_t d1, int8_t d2)
{	
	//symmetrical matrix indexing
	if (d1<=d2)
	{
		return sram_index((((ACC_MAX_DATA+1)*d1)+d2-((d1*(d1+1))/2)));
	}
	else return sram_index((((ACC_MAX_DATA+1)*d1)+d2-((d1*(d1+1))/2)));
}


void enter_standby(void)
{
	/* Make-Shift Low-Power Mode Toggle*/
	//disable maskable interrupts
	ISE_ACCESS->ISE0 |= BIT(8) | BIT(10);
	//disable PLL
	RCC_ACCESS->CR &= ~BIT(24);
	//change voltage reg scale
	PWR_ACCESS->CR &= 0x5FFF;
	RCC_ACCESS->CFGR &= ~0x5;				//new M
	RCC_ACCESS->CFGR &= 0xAFF;			//new N
	RCC_ACCESS->CFGR &= 0x6FFFFFF;	//new Q
	RCC_ACCESS->AHB1E &= ~BIT(21);		//disable DMA clock
	RCC_ACCESS->AHB3E &= ~(BIT(1) | BIT(2) | BIT(19));	//disable timers and UART clocks
	RCC_ACCESS->CR |= BIT(24);		//enable PLL
	//NOTE: Without proper circuitry, these power-saving efforts are in vain
}

void exit_standby(void)
{
	ISE_ACCESS->ISE0 |= BIT(8) | BIT(10);
	//disable PLL
	RCC_ACCESS->CR &= ~BIT(24);
	//change to normal pwr
	PWR_ACCESS->CR &= ~0xFFFFF;
	PWR_ACCESS->CR |= (0x3<<14);
	//change to normal prescaler vals
	RCC_ACCESS->CFGR &= ((0x10) | (0x2<<2) | (0x150<<6) | ~(1<<22) | (0x22<<24));
	RCC_ACCESS->AHB1E |= BIT(21);		//enable DMA clock
	RCC_ACCESS->AHB3E |= (BIT(1) | BIT(2) | BIT(19));	//enable timers and UART clocks
	RCC_ACCESS->CR |= BIT(24);		//enable PLL
	
	if (PWR_ACCESS->CSR & BIT(1)) 
	{
		//clear standby flag
		PWR_ACCESS->CR |= BIT(3);
		//reinitilize sram which has, by now, been wiped out
		mag_lut_init();
	}
	else
	{
		error_flags.pwr_err = 1;
		CURR_STATE = ERR;
	}
}

void mag_lut_init(void)
{
	for (int i = 0; i<ACC_MAX_DATA; i++)
	{
		for (int j=i; j<ACC_MAX_DATA; j++)
		{
			//symmetrical matrix indexing writes ~6sec
			sram_index((((ACC_MAX_DATA+1)*i)+j-((i*(i+1))/2))) = sqrtf((i*i) + (j*j)); 
		}
	}
}

void Err_Handler(void)
{
	CURR_STATE = ERR;
	
	while (1)
	{
		//TODO
	}
}
