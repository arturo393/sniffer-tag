#include "uwb.h"
#include "Application.h"
extern uint8_t running_device;
volatile unsigned long time32_incr;
/******************************************
*          Get Tick Count
*******************************************/
unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

/******************************************
*   
*******************************************/
void sleep_ms(unsigned int time_ms)
{
	unsigned long end = portGetTickCnt() + time_ms;
	while ((signed long)(portGetTickCnt() - end) <= 0)
	    ;
}

/******************************************
*          sleep functions
*******************************************/
void Sleep(uint32_t time_ms)
{
	sleep_ms(time_ms);
}


void deca_usleep(unsigned int usec)
{
    unsigned int i;

	usec*=12;
	for(i=0;i<usec;i++)
	{
		__NOP();
	}
}

void deca_sleep(unsigned int time_ms)
{
	Sleep(time_ms);
}


ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
	ITStatus bitstatus = RESET;
	uint32_t enablestatus = 0;
	/* Check the parameters */
	assert_param(IS_GET_EXTI_LINE(EXTI_Line));

	enablestatus =  EXTI->IMR & EXTI_Line;
	if (enablestatus != (uint32_t)RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}




/******************************************
*             reset UWB
*******************************************/
void reset_DWIC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    if(running_device == DEV_UWB3000F27)
    {
        // Enable GPIO used for DW3000 reset
        GPIO_InitStructure.Pin = DW3000_RST_PIN;
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStructure.Pull = GPIO_PULLDOWN;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
        HAL_GPIO_WritePin(GPIOA, DW3000_RST_PIN, GPIO_PIN_RESET);
        
        //drive the RSTn pin low
        HAL_GPIO_DeInit(GPIOA, DW3000_RST_PIN);
        //put the pin back to tri-state ... as input
        GPIO_InitStructure.Pin = DW3000_RST_PIN;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);    
        Sleep(3);
    }
    else
    {
        // Enable GPIO used for DW3000 reset
        GPIO_InitStructure.Pin = DW3000_RST1_PIN;
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStructure.Pull = GPIO_PULLDOWN;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
        HAL_GPIO_WritePin(GPIOA, DW3000_RST1_PIN, GPIO_PIN_RESET);
        
        //drive the RSTn pin low
        HAL_GPIO_DeInit(GPIOA, DW3000_RST1_PIN);
        //put the pin back to tri-state ... as input
        GPIO_InitStructure.Pin = DW3000_RST1_PIN;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);    
        Sleep(3);
    }

}

/******************************************
*               CS wakeup
*******************************************/
void wakeup_device_with_io(void)
{
    
	port_SPIx_clear_chip_select(); 
	Sleep(1);   
	port_SPIx_set_chip_select();  
	Sleep(7);
    
}


int writetospi_serial( uint16_t headerLength,
			   	    const uint8_t *headerBuffer,
					uint32_t bodylength,
					const uint8_t *bodyBuffer
				  )
{
    int i, j;  

    uint8_t buf[100] = {0};
    for(i = 0; i < headerLength; i++)
    {
        buf[i] = headerBuffer[i];
    }
    for(j = 0; j < bodylength; j++)
    {
        buf[i+j] = bodyBuffer[j];
    }
    if(running_device == DEV_UWB3000F27)
    {
        port_SPIx_set_chip_select();
        HAL_SPI_Transmit(&hspi1, buf, i+j, 0xffff);
        port_SPIx_clear_chip_select();
    }
    else
    {
        port_SPI2_set_chip_select();
        HAL_SPI_Transmit(&hspi2, buf, i+j, 0xffff);
        port_SPI2_clear_chip_select();
    }
    
    return 0;
}

int readfromspi_serial( uint16_t	headerLength,
			    	 const uint8_t *headerBuffer,
					 uint32_t readlength,
					 uint8_t *readBuffer )
{
    if(running_device == DEV_UWB3000F27)
    {
        port_SPIx_set_chip_select();
        HAL_SPI_Transmit(&hspi1, (uint8_t*)headerBuffer, headerLength, 0xffff);
        HAL_SPI_Receive(&hspi1, readBuffer, readlength, 0xffff);
        port_SPIx_clear_chip_select();
    }
    else
    {
        port_SPI2_set_chip_select();
        HAL_SPI_Transmit(&hspi2, (uint8_t*)headerBuffer, headerLength, 0xffff);
        HAL_SPI_Receive(&hspi2, readBuffer, readlength, 0xffff);
        port_SPI2_clear_chip_select();
    }
    return 0;
}
