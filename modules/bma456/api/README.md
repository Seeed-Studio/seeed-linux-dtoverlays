# BMA4xy Sensor API
## Introduction
This package contains Bosch Sensortec's BMA4xy accelerometer family's sensor API

## Sensor API Revisions
Files         | Revision | Release date
--------------|----------|-------------
bma4_defs.h   | 2.1.9    | 2017.10.12
bma4.h        | 2.1.9    | 2017.10.12
bma4.c        | 2.1.9    | 2017.10.12
bma456.h      | 1.1.3    | 2017.10.12
bma456.c      | 1.1.3    | 2017.10.12

## Sensor API integration

Inside your project, include the common files *bma4_defs.h*, *bma4.h*, *bma4.c* and the variant specific source and include, *bma4xy.c* and *bma4xy.h*.

If you are using an auxilliary magnetometer, either the BMM150 or the AKM9916, include to respective source and include, *aux_bmm150.c* and *aux_bmm150.h*, or, *aux_akm9916.c* and *aux_akm9916.h*.

In your code, include the variant specific header *bma4xy.h*

``` c
#include "bma422.h"
```

## User guide

### Initializing sequence
First application setup examples algorithms:
After correct power up by setting the correct voltage to the appropriate external pins, the
BMA421 enters automatically into the Power On Reset (POR) sequence. In order to properly
make use of the BMA421, certain steps from host processor side are needed. 
a. Reading the chip id.
b. Performing initialization sequence.
c. Checking the correct status of the interrupt feature engine.

``` c
#include "bma421.h"

uint16_t main(void) {
	uint16_t rslt = BMA4_OK;
	uint8_t init_seq_status = 0;
	
	/* Declare an instance of the BMA4xy device */
	struct bma4_dev dev;
	
	/* Modify the parameters */
	dev.dev_addr 		= BMA4_I2C_ADDR_PRIMARY;
	dev.interface 		= BMA4_I2C_INTERFACE;
	dev.bus_read 		= USER_i2c_read;
	dev.bus_write 		= USER_i2c_write;
	dev.delay 			= USER_delay_ms;
	dev.read_write_len = 8;
	dev.resolution 		= 12;
	dev.feature_len 	= BMA421_FEATURE_SIZE;
	
	/* a. Reading the chip id. */
	rslt |= bma421_init(&dev);
	
	/* b. Performing initialization sequence. 
		c. Checking the correct status of the initialization sequence.
	*/
	rslt |= bma421_write_config_file(&dev);
	
	return rslt;
}
```

### Sensor API initialization for the I2C protocol
``` c
#include "bma422.h"

uint16_t main(void) {
	uint16_t rslt = BMA4_OK;
	
	/* Declare an instance of the BMA4xy device */
	struct bma4_dev accel;
	
	/* Modify the parameters */
	accel.dev_addr = BMA4_I2C_ADDR_PRIMARY;
	accel.interface = BMA4_I2C_INTERFACE;
	accel.bus_read = USER_i2c_read;
	accel.bus_write = USER_i2c_write;
	accel.delay = USER_delay_ms;
	accel.read_write_len = 8;
	
	/* Initialize the instance */
	rslt |= bma422_init(&accel);
	
	return rslt;
}
```

### Configuring the accelerometer
``` c
#include "bma422.h"

uint16_t main(void) {
	uint16_t rslt = BMA4_OK;

	/* Initialize the device instance as per the initialization example */
	
	/* Enable the accelerometer */
	bma4_set_accel_enable(ACCEL_ENABLE, dev);
	
	/* Declare an accelerometer configuration structure */
	struct bma4_accel_config accel_conf;
	
	/* Assign the desired settings */
	accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
	accel_conf.range = BMA4_ACCEL_RANGE_2G;
	accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
	
	/* Set the configuration */
	rslt |= bma4_set_accel_config(&accel_conf, &accel);
	
	return rslt;
}
```

### Reading out the accelerometer data
``` c
#include "bma422.h"

uint16_t main(void) {
	uint16_t rslt = BMA4_OK;

	/* Initialize the device instance as per the initialization example */
	
	/* Configure the accelerometer */
	
	/* Declare an instance of the sensor data structure */
	struct bma4_accel sens_data;
	
	/* Loop forever */
	while (1) {
		/* Read the sensor data into the sensor data instance */
		rslt |= bma4_read_accel_xyz(&sens_data, &accel);
		
		/* Exit the program in case of a failure */
		if (rslt != BMA4_OK)
			return rslt;
		
		/* Use the data */
		printf("X: %d, Y: %d, Z: %d\n", sens_data.x, sens_data.y, sens_data.z);
		
		/* Pause for 10ms, 100Hz output data rate */
		USER_delay_ms(10);
	}
	
	return rslt;
}
```

### Configuring reading out sensor data from the FIFO buffer 
``` c
#include "bma422.h"

uint16_t main(void) {
	uint16_t rslt = BMA4_OK;

	/* Initialize the device instance as per the initialization example */
	
	/* Configure the accelerometer */
	
	/* Setup and configure the FIFO buffer */
	/* Declare memory to store the raw FIFO buffer information */	
	uint8_t fifo_buff[255];
	struct bma4_fifo_frame fifo_frame;
	
	/* Modify the FIFO buffer instance and link to the device instance */
	fifo_frame.data = fifo_buff;
	fifo_frame.length = 255;
	fifo_frame.fifo_data_enable = BMA4_ENABLE;
	fifo_frame.fifo_header_enable = BMA4_ENABLE;
	accel.fifo = &fifo_frame;
	
	/* Disable the advanced power save mode to configure the FIFO buffer */
	rslt |= bma4_set_advance_power_save(BMA4_DISABLE, &accel);
	
	/* Configure the FIFO buffer */
	rslt |= bma4_set_fifo_config((BMA4_FIFO_ACCEL | BMA4_FIFO_HEADER), BMA4_ENABLE, &accel);
	
	/* Declare instances of the sensor data structure */
	struct bma4_accel sens_data[36]; // 255 bytes / ~7bytes per frame = 36 instances
	uint16_t n_instances, i;
	
	/* Loop forever */
	while (1) {
		/* Read data from the sensor FIFO buffer */
		rslt |= bma4_read_fifo_data(&accel); // Read FIFO data
		
		/* Exit the program in case of a failure */
		if (rslt != BMA4_OK)
			return rslt;
		
		/* Reset the maximum number of requried sensor data instances */
		n_instances = 36;
		
		/* Parse the FIFO until there are less frames than requested */
		while (n_instances == 36) {
			/* Parse the FIFO buffer and extract requried number of accelerometer data frames */
			rslt |= bma4_extract_accel(sens_data, &n_instances, &accel);
			
			/* Exit the program in case of a failure */
			if (rslt != BMA4_OK)
				return rslt;
			
			/* Use the accelerometer data frames */
			for (i = 0; i < n_instances; i++)
				printf("X:%d, Y:%d, Z:%d\n", sens_data[i].x, sens_data[i].y, sens_data[i].z);
		}
		
		// At 100Hz, The FIFO buffer will have 36 frames ready in (1 / 100) * 36 = ~0.36s
		USER_delay_ms(360); 
	}
	
	return rslt;
}
```

### Enabling and mapping line interrupt to that of BMA422 sensor interrupt

This example shows mapping of a line interrupt with two feature interrupts simultaneously in
variant BMA422.
Note: There are two interrupts - feature interrupt and hardware interrupts. You can map more 
than one interrupt with a single line interrupt. If a feature interrupt is mapped with a line 
interrupt, one can  map the other line interrupt with that of hardware interrupt. This example
can be done for other variants as well.
``` c 
#include "bma422.h"

uint16_t main(void)
{
    uint16_t result = BMA4_OK;
     /* Variable to define two interrupt lines */
    uint8_t  int_line[2] = {BMA4_INTR1_MAP, BMA4_INTR2_MAP};
    /* Variable to define feature interrupts to be mapped*/
    uint16_t int_map  = (BMA422_STEP_CNTR_INT | BMA422_WAKEUP_INT);    
    
    /* Initialize the device instance as per the initialization example */
    
    /* Configure the accelerometer as per the example */
    
    /* Mapping line interrupt 1 with that of two sensor feature interrupts -
     * Step counter and wake up interrupt */
    result =  bma422_map_interrupt(int_line[0], int_map, BMA4_ENABLE, dev);
    
    if(result == BMA4_OK) {
       printf("Interrupt mapping successful\r\n");
    }
    else {
        printf("Interrupt mapping failed\r\n");
        printf("Error code = %d\r\n", result);
    }
    
    return result;
}
```

### Reading interrupt status register
This example is in continuation of the  previous example: Enabling and mapping line interrupt.
After the interrupts are mapped, interrupt status register is read in an interrupt service 
routine to perform the corresponding tasks on an interrupt. 
``` c
#include "bma422.h"

void interrupt_handler(void)
{ 
    uint16_t result = BMA4_OK;
    /* Define a variable to get the status */
    uint16_t int_status = 0;    
   
    /* Read the interrupt status register */
    result =  bma422_read_int_status(&int_status, dev)

    if(result == BMA4_OK) {
  
        if(int_status & BMA422_STEP_CNTR_INT) {
   
            /* Call the function to be performed on step counter interrupt */
        
        } else if(int_status & BMA422_WAKEUP_INT) {
    
            /* Call the function to be performed on wake up interrupt */        
    }
}
```
### Configuring the auxiliary sensor BMM150

### Initialization of auxiliary interface to access BMM150
```
    /* Structure declaration */
    struct bma4_dev *dev;
    
    /* Switch on the the auxiliary interface mode */
    dev->aux_config.if_mode = BMA4_ENABLE_AUX_IF_MODE;
    /* Set the I2C device address of auxiliary device */
    /* Device address of BMM150 */
    dev->aux_config.aux_dev_addr = BMA4_I2C_BMM150_ADDR;
    /* Set aux interface to manual mode */
    dev->aux_config.manual_enable = BMA4_MANUAL_ENABLE;
    /* Set the number of bytes for burst read */
    dev->aux_config.burst_read_length = BMA4_AUX_READ_LEN_0;
    
    /* Initialize the auxiliary interface */
    bma4_aux_interface_init(&sensor);
```
### Reading of data from auxiliary sensor BMM150  
Initialize the auxiliary interface as shown above. Set the power mode to sleep mode and the operation mode 
to forced mode. Then validate the sensor by reading the chip id.

```  
    uint8_t aux_data[5] = {0};
    uint8_t aux_chip_id = 0;
    uint8_t aux_write;
    
    /* Enable the power control bit for sleep mode in 0x4B register of BMM150 */
    aux_write = 0x01;
    result |= bma4_aux_write(0x4B, &aux_write, 1, dev);

    /* Enable forced mode in 0x4C register of BMM150 */
    aux_write = 0x02;
    result |= bma4_aux_write(0x4c, &aux_write, 1, dev);
    
    /* Read the chip-id of the auxiliary sensor */    
    result = bma4_aux_read(BMA4_AUX_CHIP_ID_ADDR, &aux_chip_id, 1, dev);
    if(aux_chip_id == BMM150_CHIP_ID) {
        result = bma4_aux_read(aux_read_addr, aux_data, 1, dev);
    }    
```

### Writing of data from auxiliary sensor BMM150 
Initialize the auxiliary interface as shown above. Set the power mode to sleep mode and the operation mode 
to forced mode. Then validate the sensor by reading the chip id before writing..

```    
    uint8_t aux_write_data[5] = {0xFF, 0xAA, 0xFD, 0x78, 0x99};
    uint8_t aux_chip_id = 0;
    uint8_t aux_write;
    
    /* Enable the power control bit for sleep mode in 0x4B register of BMM150 */
    aux_write = 0x01;
    result |= bma4_aux_write(0x4B, &aux_write, 1, dev);

    /* Enable forced mode in 0x4C register of BMM150 */
    aux_write = 0x02;
    result |= bma4_aux_write(0x4c, &aux_write, 1, dev);
            
    /* Read the chip-id of the auxiliary sensor */    
    result = bma4_aux_read(BMA4_AUX_CHIP_ID_ADDR, &aux_chip_id, 1, dev);
    if(aux_chip_id == BMM150_CHIP_ID) {
        result = bma4_aux_write(0x50, aux_write_data, 4, dev);
    }
    
```

### Accessing of auxiliary sensor BMM150 with the help of BMM150 APIs via BMA4 secondary interface.
User has to create a  wrapper function over bma4_aux_read and bma4_aux_write in order to map with 
bmi150 read and write

```
/* Structure declaration */
struct bmm150_dev bmm150;

/* function declaration */
int8_t bma4_user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bma4_user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);

/* Update bmm150 structure */
bmm150.read = bma4_user_aux_read;
bmm150.write = bma4_user_aux_write;
bmm150.id = BMM150_DEFAULT_I2C_ADDRESS;
bmm150.delay_ms = delay_ms;
bmm150.interface = BMM150_I2C_INTF;

/* Initialize bmm150 sensor */
bmm150_init(bmm150);
```

### Wrapper functions for auxiliary read and write
    
```
/* Wrapper function to map bmm150.read */
int8_t bma4_user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
       int8_t result;

       if (dev->aux_config.aux_dev_addr == id)
              result = bma4_aux_read(reg_addr, aux_data, len, dev);

       return result;
}

/* Wrapper function to map bmm150.write */
int8_t bma4_user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
       int8_t result;

       if (dev->aux_config.aux_dev_addr == id)
              result = bma4_aux_write(reg_addr, aux_data, len, dev);

       return result;
}
```

### Get temperature from BMA4 sensor, eg., BMA422 
A scaling factor of 1000 has to be used to convert the read temperature back to its 
float value.

```
void main()
{
    int8_t rslt;
    int32_t get_temp_C = 0;
    int32_t get_temp_F = 0;
    int32_t get_temp_K = 0;
    float actual_temp = 0;
    
    /* Initialize bma4 sensor to get the chip id */
    rslt = bma4_init(dev);
    
    if(rslt == BMA4_OK) {
        if(dev->chip_id == BMA422_CHIP_ID) {
            rslt = bma422_init(dev);
            
            if(rslt == BMA4_OK) {
                /* Reset the accelerometer */
                bma4_set_command_register(0xB6, dev);
                dev->delay(1);
            
                /* Enable the accelerometer */
                rslt = bma4_set_accel_enable(1, dev);
                dev->delay(10);
                
                /* Get temperature in degree C */
                rslt = bma4_get_temperature(&get_temp_C, BMA4_DEG, dev);
                /* Get temperature in degree F */
                rslt = bma4_get_temperature(&get_temp_F, BMA4_FAHREN, dev);
                /* Get temperature in degree K */
                rslt = bma4_get_temperature(&get_temp_K, BMA4_KELVIN, dev);
            
                /* Divide the temperature read with the scaling factor to get 
                the actual temperature */
                if(rslt == BMA4_OK) {
                    actual_temp = (float)get_temp_C / (float)BMA4_SCALE_TEMP;
                    printf("Actual temperature in degree celsius is %10.2f degrees C\r\n", actual_temp);
                    
                    actual_temp = (float)get_temp_F / (float)BMA4_SCALE_TEMP;
                    printf("Actual temperature in degree fahranheit is %10.2f degrees F\r\n", actual_temp);
                    
                    actual_temp = (float)get_temp_K / (float)BMA4_SCALE_TEMP;
                    printf("Actual temperature in degree kelvin is %10.2f degrees K\r\n", actual_temp);
                    
                    /* 0x80 - temp read from the register and 23 is the ambient temp added.
                     * If the temp read from register is 0x80, it means no valid
                     * information is available */
                    if(((get_temp_C - 23) / BMA4_SCALE_TEMP) == 0x80) {
                        printf("No valid temperature information available\r\n");
                    }
               }
            }                
        }
    }
}
```
### Selection of Any-motion and No-motion feature.
This snippet of code reflects that how to switch between any-motion and no-motion.
Given snippet of code is for BMA421 sensor. Same steps are applicable for others sensors.

```	
	/* Enable/select the no-motion feature */
	bma421_feature_enable(BMA421_NO_MOTION, 1, dev);
	
	/*Enable the axis as per requirement for Any/no-motion. Here all axis has been enabled */
	bma421_anymotion_enable_axis(BMA421_ALL_AXIS_EN, dev);
	
	/* Enable/select the Any-motion feature */
	bma421_feature_enable(BMA421_ANY_MOTION, 1, dev);

```

### Activity(still/walking/running/invalid) recognition.

```	
	/* Enable the activity feature */
	 rslt = bma421_feature_enable(BMA421_ACTIVITY, 1, dev);
	
	/* Map the activity out interupt to INT pin1 */
	bma421_map_interrupt(BMA4_INTR1_MAP,BMA421_ACTIVITY_INT, BMA4_ENABLE, dev);
	
```
### ISR for activity interrupt recognition
``` c
	uint16_t int_status = 0;
	uint8_t activity_out = 0;
 
	/* Read the Interrupt status reg. */
	bma421_read_int_status(&int_status, dev);

        if(int_status & BMA421_ACTIVITY_INT) {	
		/* Read the activity out register for user's activity*/
		bma421_activity_output(&activity_out, dev);
		/* compare the output of activity register and perform the action as per requirement */
		switch (activity_out) {

			 case BMA421_USER_STATIONARY:
			     /* User state is stationary */
			     /* Perform the respective action according to this state */
			     break;
			 case BMA421_USER_WALKING:
			     /* User state is walking */
			     /* Perform the respective action according to this state */
			     break;
			 case BMA421_USER_RUNNING:
			     /* User state is running */
			     /* Perform the respective action according to this state */
			     break;
			 case BMA421_STATE_INVALID:
			    /* User state is invalid */
			    /* Perform the respective action according to this state */
			     break;
		}
	}
```

### Platform(wrist/phone) selection.
Note:- Default configuration is wrist.
```	
	/* Select the phone configuration for the sensor */
	bma421_select_platform(BMA421_PHONE_CONFIG, dev);
	
	/* Select the wrist configuration for the sensor */
	bma421_select_platform(BMA421_WRIST_CONFIG, dev);
	
```