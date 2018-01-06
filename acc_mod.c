
#include "acc_mod.h"

acc_unit acc_accessor;

static uint8_t acc_init(void);
static uint8_t acc_ints_init(void);
static uint8_t acc_i2c_read(uint8_t, uint8_t*);
static uint8_t acc_i2c_read_multi(uint8_t, uint8_t *, uint8_t);
static uint8_t acc_i2c_write(uint8_t, uint8_t);
static uint8_t acc_i2c_write_multi(uint8_t, uint8_t*, uint32_t);


uint8_t create_acc(acc_unit* acc, uint8_t addr, uint8_t is_upper_link)
{
	static uint8_t first_acc = FALSE;
	
	if (addr == NULL) return NULL;
	
	(*acc).acc_addr = addr;
	if(is_upper_link) (*acc).meta_data.is_upper_link = TRUE;
	if(!first_acc) (*acc).meta_data.is_ints = TRUE;
	
	++first_acc;	
	return 1;
}

uint8_t acc_exec(acc_unit* acc, uint8_t func, ...)
{
	uint8_t succ = TRUE;			//innocent until proven guilty
	
	acc_accessor = *acc;
	
	va_list args;
	
	va_start(args, func);
	
	switch(func)
	{
		case accel_init:
			acc_init();
			break;
		case accel_ints_init:
			acc_ints_init();
			break;
		case accel_read:
			acc_i2c_read((uint8_t) *va_arg(args, uint8_t*), va_arg(args, uint8_t*));
			break;
		case accel_read_multi:
			acc_i2c_read_multi((uint8_t) *va_arg(args, uint8_t*), va_arg(args, uint8_t*), (uint8_t) *va_arg(args, uint8_t*));
			break;
		case accel_write:
			acc_i2c_write((uint8_t) *va_arg(args, uint8_t*), (uint8_t) *va_arg(args, uint8_t*));
			break;
		case accel_write_multi:
			acc_i2c_write_multi((uint8_t) *va_arg(args, uint8_t*), va_arg(args, uint8_t*), (uint8_t) *va_arg(args, uint8_t*));
			break;
		default:
			succ = FALSE;
			break;
	}
	
	va_end(args);
	
	//record mutations
	*acc = acc_accessor;
	
	return succ;
}

static uint8_t acc_init()
{
	uint8_t succ = 1;
	uint8_t check_id;
	
	acc_i2c_read(0x00, &check_id);
	if (check_id != ACC_ID) succ = FALSE;
	
	acc_i2c_write(ACC_PWR, 0x00);		//clear pwr ctl
	
	uint8_t cmd = 0x00;
	cmd |= BIT(2);		//wake up
	cmd |= BIT(4);		//measure mode enable
	
	acc_i2c_write(ACC_PWR, cmd);
	acc_i2c_write(ACC_FORMAT, 0x00);		//clear data format
	
	cmd = 0x00;		//clear
	cmd |= BIT(8);	//full res mode
	
	acc_i2c_write(ACC_FORMAT, cmd);	//configure data format
	
	//first piece of data
	if (!acc_i2c_read_multi(ACC_DATA, (uint8_t*) &acc_accessor.curr_data, 6)) succ = FALSE;
	
	return succ;
}

static uint8_t acc_ints_init(void)
{
	acc_i2c_write(ACC_INT_EN, 0x00);		//disable interrupts
	
	uint8_t cmd = 0x00;
	cmd |= BIT(3);		//inactivity interrupt map -> line 2 (EXTI Line 4)
	cmd &= ~BIT(5);		//double tap interrupt map -> line 1 (EXTI Line 2)
	
	acc_i2c_write(ACC_INT_MAP, cmd);
	cmd = 0xFF;			//max effort --> 62.5 mg/LSB = + 16g
	acc_i2c_write(ACC_TAP_THRESH, cmd);
	cmd = 0x10;			//contact over 10ms is not a tap (625 us*LSB = 10ms)
	acc_i2c_write(ACC_TAP_DUR, cmd);
	cmd = 0x2;			//second valid tap can begin after half ms
	acc_i2c_write(ACC_TAP_WINDOW, cmd);
	cmd = 0x4;			//second tap expires after 2ms
	acc_i2c_write(ACC_TAP_LATENT, cmd);
	
	cmd = 0x18;			//inactivity threshold
	acc_i2c_write(ACC_INACT_THRESH, cmd);
	cmd = 0xFF;			//generate interrupt after 255 secs
	acc_i2c_write(ACC_INACT_DUR, cmd);
	
	cmd = 0x00;
	cmd |= BIT(3);		//enable inactivity interrupt
	cmd |= BIT(5);		//enable double tap interrupt
	acc_i2c_write(ACC_INT_EN, cmd);
	
	return TRUE;
}

static uint8_t acc_i2c_read(uint8_t reg, uint8_t* result)
{
	//TODO specify timeout instead of infinite loops
	while(I2C1_ACCESS->SR2 & 0x4);		// Wait for BUSY line
	I2C1_ACCESS->CR1 |= 0x2;				// Generate START condition

	while (!(I2C1_ACCESS->SR1 & 0x80)); 		// Wait for EV5
	I2C1_ACCESS->DR = acc_accessor.acc_addr<<1;					// Write device address (W)

	while (!(I2C1_ACCESS->SR1 & 0x40));	// Wait for EV6
    (void)I2C1_ACCESS->SR2;						// Read SR2

	while (!(I2C1_ACCESS->SR1 & 0x8));		// Wait for EV8_1
	I2C1_ACCESS->DR = reg;

	I2C1_ACCESS->CR1 |= 0x4;				// Generate STOP condition


	I2C1_ACCESS->CR1 |= 0x2;				// Generate START condition

	while (!(I2C1_ACCESS->SR1 & 0x80)); 		// Wait for EV5
	I2C1_ACCESS->DR = (acc_accessor.acc_addr << 1 ) | 1;			// Write device address (R)

	while (!(I2C1_ACCESS->SR1 & 0x6));	// Wait for EV6
    I2C1_ACCESS->CR1 &= ~0x200;              // No ACK
    (void)I2C1_ACCESS->SR2;						// Read SR2

	while (!(I2C1_ACCESS->SR1 & 0x20));	// Wait for EV7_1
    *result = (uint8_t)I2C1_ACCESS->DR;      // Read value

    I2C1_ACCESS->CR1 |= 0x4;			    // Generate STOP condition
		
	return TRUE;
}

static uint8_t acc_i2c_read_multi(uint8_t reg, uint8_t * result, uint8_t length)
{
	//TODO specify timeout instead of infinite loops
	while(I2C1_ACCESS->SR2 & 0x4);		// Wait for BUSY

	I2C1_ACCESS->CR1 |= 0x2;				// Generate START

	while (!(I2C1_ACCESS->SR1 & 0x80)); 		// Wait for EV5
	I2C1_ACCESS->DR = acc_accessor.acc_addr<<1;					// Write device address

	while (!(I2C1_ACCESS->SR1 & 0x40));	// Wait for EV6
    (void)I2C1_ACCESS->SR2;						// Read SR2

	while (!(I2C1_ACCESS->SR1 & 0x8));		// Wait for EV8_1
	I2C1_ACCESS->DR = reg;					// Write reg address

	I2C1_ACCESS->CR1 |= 0x4;				// Generate STOP


	I2C1_ACCESS->CR1 |= 0x2;				// Generate START

	while (!(I2C1_ACCESS->SR1 & 0x50)); 		// Wait for EV5
	I2C1_ACCESS->DR = (acc_accessor.acc_addr << 1 ) | 1;			// Write device address

	//2-byte reception requires POS bit specification
	if(length==2)
	{
	    while (!(I2C1_ACCESS->SR1 & 0x40));	// Wait for EV6

	    I2C1_ACCESS->CR1 &= ~0x200;              // No ACK
	    I2C1_ACCESS->CR1 |= 0x400;               // POS
	    (void)I2C1_ACCESS->SR2;

	    while (!(I2C1_ACCESS->SR1 & 0x800));	    // Wait for BTF
	    I2C1_ACCESS->CR1 |= 0x4;			    // Generate STOP

	    *result++ = (uint8_t)I2C1_ACCESS->DR;          // Read value
	    *result++ = (uint8_t)I2C1_ACCESS->DR;          // Read value
	}
	if(length>2)
	{
	    while (!(I2C1_ACCESS->SR1 & 0x40));	// Wait for EV6
	    (void)I2C1_ACCESS->SR2;

	    length--;
	    while(length--)
	    {
		    while (!(I2C1_ACCESS->SR1 & 0x100));	    // Wait for BTF
		    *result++ = (uint8_t)I2C1_ACCESS->DR;          // Read value

		    if(length==1)
		    {
			    I2C1_ACCESS->CR1 &= ~0x200;              // No ACK
			    I2C1_ACCESS->CR1 |= 0x4;			    // Generate STOP
		    }
	    }

	    *result++ = (uint8_t)I2C1_ACCESS->DR;          // Read value
	}
	
	return TRUE;
}

static uint8_t acc_i2c_write(uint8_t reg, uint8_t data)
{
	//TODO specify timeout instead of infinite loops
	I2C1_ACCESS->CR1 |= 0x2;				// Generate START condition

	while (!(I2C1_ACCESS->SR1 & 0x80)); 		// Wait for EV5
	I2C1_ACCESS->DR = acc_accessor.acc_addr<<1;					// Write device addres

	while (!(I2C1_ACCESS->SR1 & 0x40));	// Wait for EV6
    (void)I2C1_ACCESS->SR2;						// Read SR2

	while (!(I2C1_ACCESS->SR1 & 0x8));		// Wait for EV8_1
	I2C1_ACCESS->DR = reg;					// Write reg address

	while (!(I2C1_ACCESS->SR1 & 0x100));	    // Wait for BTF
	I2C1_ACCESS->DR = data;

	I2C1_ACCESS->CR1 |= 0x4;			    // Generate STOP
	
	return TRUE;
}

static uint8_t acc_i2c_write_multi(uint8_t reg, uint8_t* buf, uint32_t length)
{
	//TODO specify timeout instead of infinite loops
	I2C1_ACCESS->CR1 |= 0x2;				// Generate START

	while (!(I2C1_ACCESS->SR1 & 0x80)); 		// Wait for EV5
	I2C1_ACCESS->DR = acc_accessor.acc_addr<<1;					// Write device address

	while (!(I2C1_ACCESS->SR1 & 0x40));	// Wait for EV6
    (void) I2C1_ACCESS->SR2;						// Read SR2

	while (!(I2C1_ACCESS->SR1 & 0x8));		// Wait for EV8_1
	I2C1_ACCESS->DR = reg;					// Write reg address

	while (--length)
	{
		while (!(I2C1_ACCESS->SR1 & 0x100));	    // Wait for BTF
		I2C1_ACCESS->DR = *buf++;
	}

	while (!(I2C1_ACCESS->SR1 & 0x100));	    // Wait for BTF

	I2C1_ACCESS->CR1 |= 0x4;			    // Generate STOP
	
	return TRUE;
}
