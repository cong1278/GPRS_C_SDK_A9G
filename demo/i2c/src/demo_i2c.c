
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "api_os.h"
#include "api_debug.h"
#include "api_event.h"
#include "api_hal_i2c.h"




#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "Main Test Task"

#define SECOND_TASK_STACK_SIZE    (2048 * 2)
#define SECOND_TASK_PRIORITY      1
#define SECOND_TASK_NAME          "Second Test Task"

static HANDLE mainTaskHandle = NULL;
static HANDLE secondTaskHandle = NULL;
#define I2C_ACC I2C2


#define PCF8574_ADDR 	0X21    
#define PCF8575_ADDR 	0X20 
#define I2C_TIMEOUT         10 

void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            break;

        case API_EVENT_ID_SYSTEM_READY:
            Trace(1,"system initialize complete");
            break;

        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(2,"network register success");
            break;

        default:
            break;
    }
}
uint8_t PCF8575_WriteByte(uint16_t DataToWrite)
{				   	  	    						
    uint8_t data[2];	
    data[0] =  DataToWrite&0x00ff;
    data[1] = (DataToWrite&0xff00)>>8;	
    uint8_t get =  I2C_Transmit(I2C_ACC, PCF8575_ADDR, data, 2, I2C_TIMEOUT);
    return get;
    
}
uint16_t PCF8575_ReadByte(void)
{				  
    uint8_t temp[2];		
    uint16_t temp1=0;  	    																     
    uint8_t Error = I2C_Receive(I2C_ACC, PCF8575_ADDR, temp, 2, I2C_TIMEOUT);
    if(Error != 0)
    {
    Trace(1,"I2C_Error_t :%d",Error);
    }
    temp1 |= temp[1]<<8;
    temp1 |= temp[0];
    return temp1;
}

void SecondTask(void *pData)
{
    uint8_t accId;
    uint8_t W=5;
    I2C_Config_t config;

    config.freq = I2C_FREQ_100K;
    I2C_Init(I2C_ACC, config);
    // uint8_t i=0;
    uint8_t j=0;
    // int ret=-1;

    while(1)
    {
        //read accelerator chip ID: 0x33  0X21
        // for(j=0;j<255;j++)
        // {
        //     int ret=I2C_ReadMem(I2C_ACC, j, i, 1, &accId, 1, I2C_DEFAULT_TIME_OUT);
        //     Trace(1,"accelerator id shold be 0x33, read:0X%02x i:%d j:%d ret=%d",accId,i,j,ret);
        //     // if(accId!=0xfd&&accId!=0xff&&ret==0)
        //     // {
        //     //     Trace(1,"accelerator id shold be 1111, read:0X%02x ",accId);
        //     //     OS_Sleep(9000);
        //     // }
            
        //     OS_Sleep(100);
        // }
        // i++;
        // ret=I2C_WriteMem(I2C2,0x20,4,1,&W,1,I2C_DEFAULT_TIME_OUT);
        // Trace(1,"accelerator write, read:0X%02x  ret=%d i=%d",accId,ret);
        // ret=I2C_ReadMem(I2C_ACC, 0X20, 4, 1, &accId, 1, I2C_DEFAULT_TIME_OUT);
        // Trace(1,"accelerator id shold be 0x33, read:0X%02x  ret=%d i=%d",accId,ret,j);
        // if(accId!=4 &&ret!=0)
        // {
        //     Trace(1,"accelerator id shold be 1111, read:0X%02x ",accId);
        //     OS_Sleep(9000);
        // }
        // OS_Sleep(50);
        // j++;
        uint8_t error=PCF8575_WriteByte(j);
        if(error != 0)
        {
        Trace(1,"I2C_Error_t WRITE :%d",error);
        }
        else
        {
            Trace(1,"I2C write ok");
        }
        
        uint16_t get=PCF8575_ReadByte();
        Trace(1,"I2C  read :%d  i=%d",get,j);
        if(get!=j)
        {
            Trace(1,"I2C read fail");
        }
        j++;
        OS_Sleep(50);
    }
}

void MainTask(void *pData)
{
    API_Event_t* event=NULL;

    secondTaskHandle = OS_CreateTask(SecondTask,
        NULL, NULL, SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0, SECOND_TASK_NAME);

    while(1)
    {
        if(OS_WaitEvent(mainTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

void i2c_Main(void)
{
    mainTaskHandle = OS_CreateTask(MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}
