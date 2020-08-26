#include "api_hal_gpio.h"
#include "api_hal_pm.h"
#include "api_gps.h"

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include "api_os.h"
#include "api_debug.h"
#include "api_event.h"
#include "api_fs.h"

#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_inc_time.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include <api_socket.h>
#include <api_network.h>
#include "api_hal_adc.h"
#include "api_lbs.h"

#include "zk_tools.h"

#define const_DT_CM180 0
#define const_DT_MC8332 1
#define const_DT_ML5510 2
#define const_DT_ML810	3
#define config_DeviceType const_DT_ML810

#define const_WT_Dustbin 0
#define const_WT_GPRSMOD 1
#define config_WorkType const_WT_Dustbin

#define const_PowerLevel 9000
#define const_DustLevel 3000

#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "Main Test Task"

#define SECOND_TASK_STACK_SIZE    (2048 * 2)
#define SECOND_TASK_PRIORITY      1
#define SECOND_TASK_NAME          "Second Test Task"
#define FILE_TASK_NAME			"FileCtrl Task"

#define GPS_TASK_STACK_SIZE    (2048 * 2)
#define GPS_TASK_PRIORITY      1
#define GPS_TASK_NAME          "GPS Test Task"

#define GPRS_TASK_STACK_SIZE    (2048 * 2)
#define GPRS_TASK_PRIORITY      1
#define GPRS_TASK_NAME          "GPRS Test Task"

#define ADC_TASK_STACK_SIZE    (2048 * 2)
#define ADC_TASK_PRIORITY      1
#define ADC_TASK_NAME          "ADC Test Task"

#define LBS_TASK_STACK_SIZE    (2048 * 2)
#define LBS_TASK_PRIORITY      1
#define LBS_TASK_NAME          "LBS Test Task"

#define UART1_TASK_STACK_SIZE    (2048 * 2)
#define UART1_TASK_PRIORITY      1
#define UART1_TASK_NAME         "UART1 Test Task"

#define const_LBS_Location 1
#define const_GPS_Location 2

#define const_LEDTime_Wait 100
#define const_LEDTime_Find 500
#define const_LEDTime_Connected 2000

#define CONFIG_FILE_NAME "/MacNum.conf"

static HANDLE mainTaskHandle = NULL;
static HANDLE ledRunTaskHandle = NULL;
static HANDLE ledGPSTaskHandle = NULL;
static HANDLE ledGPRSTaskHandle = NULL;
static HANDLE GPSTaskHandle = NULL;
static HANDLE GPRSTaskHandle = NULL;
static HANDLE ADCTaskHandle = NULL;
static HANDLE LBSTaskHandle = NULL;
static HANDLE uart1TaskHandle = NULL;
static HANDLE FileCtrlHandle = NULL;

unsigned char uchar_MacNum[12] = "000000000000";
unsigned char uchar_IMEI[15] = "000000000000000";
unsigned char uchar_CCID[20] = "00000000000000000000";

unsigned char uchar_RUNLED_Status;
unsigned int uint_RUNLED_Time;
unsigned char uchar_GPSLED_Status;
unsigned int uint_GPSLED_Time;
unsigned char uchar_GPRSLED_Status;
unsigned int uint_GPRSLED_Time;

static bool flag = false, flag2 = true;

unsigned char uchar_LocationStatus;
unsigned char uchar_CDMA_Status;

char GPS_latitude[15];
char GPS_longitude[15];
struct timespec GPS_TIME;
unsigned int uint_ADC0, uint_ADC1;
unsigned char uchar_RevBuff[100];

unsigned char uchar_Counter;
unsigned char uchar_SendTimer;

unsigned char uchar_DustStatus;			//垃圾状态：0-未满；1-已满；其它-未知
unsigned char uchar_PowerStatus;		//电源状态：0-欠压；1-正常；其它-未知
unsigned char uchar_GPSStatus;			//GPS状态：0-未定位；1-已定位
unsigned char uchar_MustSendErr;	//需要发送报警信息标志：0-不需要发送；1-需要发送垃圾状态；2-需要发送电压状态
unsigned char uchar_GPSTimer;			//GPS发送计时器
unsigned char uchar_GPSData[178] =
		"YYYYMMDDhhmmssWD000000000000000000000000000000000000004EJD000000000000000000000000000000000000004500000000000000000000000000000000000000000000000000000000000000000000000000000000";
unsigned char uchar_NetStatus;			//网络状态：1-内网；2-外网
unsigned char uchar_NetErrTimer;		//网络连接错误计数器

/*******************************************************************/
/////////////////////////socket configuration////////////////////////
#define DNS_DOMAIN  "113.204.181.226"
#define SERVER_PORT 7077
#define RECEIVE_BUFFER_MAX_LENGTH 300
int socketFd = -1;
uint8_t buffer[RECEIVE_BUFFER_MAX_LENGTH];
HANDLE sem = NULL;
int errorCode = 0;
/*******************************************************************/

GPIO_config_t gpioLED1 = { .mode = GPIO_MODE_OUTPUT, .pin = GPIO_PIN30,
		.defaultLevel = GPIO_LEVEL_LOW };
GPIO_config_t gpioGPSLED = { .mode = GPIO_MODE_OUTPUT, .pin = GPIO_PIN28,
		.defaultLevel = GPIO_LEVEL_LOW };
GPIO_config_t gpioGPRSLED = { .mode = GPIO_MODE_OUTPUT, .pin = GPIO_PIN27,
		.defaultLevel = GPIO_LEVEL_LOW };

bool SaveData(unsigned char *data, unsigned char Len) {
	int32_t fd;
	int32_t ret;
	uint8_t *path = CONFIG_FILE_NAME;

	fd = API_FS_Open(path, FS_O_RDWR | FS_O_CREAT, 0);
	if (fd < 0) {
		Trace(2, "Open file failed:%d", fd);
		return false;
	}
	ret = API_FS_Write(fd, data, Len);
	API_FS_Close(fd);
	if (ret <= 0)
		return false;
	return true;
}

bool ReadData(unsigned char *data, unsigned char Len) {
	int32_t fd;
	int32_t ret;
	uint8_t *path = CONFIG_FILE_NAME;

	fd = API_FS_Open(path, (FS_O_RDONLY | FS_O_CREAT), 0);
	if (fd < 0) {
		Trace(2, "Open file failed:%d", fd);
		return false;
	}
	ret = API_FS_Read(fd, data, Len);
	Trace(2, "ReadConfigFileData:%s", data);
	API_FS_Close(fd);
	if (ret <= 0)
		return false;
	return true;
}

void EventDispatch(API_Event_t* pEvent) {
	unsigned char uchar_ShowedHEX[200], i;
	switch (pEvent->id) {
	/*GPS*/
	case API_EVENT_ID_GPS_UART_RECEIVED:
		// Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
		GPS_Update(pEvent->pParam1, pEvent->param1);
		break;
	case API_EVENT_ID_UART_RECEIVED:
		if (pEvent->param1 == UART1) {
			uint8_t data[pEvent->param2 + 1];
			data[pEvent->param2] = 0;
			unsigned char result;
			memcpy(data, pEvent->pParam1, pEvent->param2);
			Trace(1, "uart received data,length:%d,data:%s", pEvent->param2,
					data);
#if config_DeviceType==const_DT_CM180
			//有方CM180
			//如果收到的有SHUTDOWN字样，则需要重新上电
			if (zk_str_findstr(data, pEvent->param2, "SHUTDOWN", 8) > 0) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 60;
			}
			switch (uchar_CDMA_Status) {
			case 0:
				result = zk_str_findstr(data, pEvent->param2, "+CIND:RUIM", 10);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 1:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0)
					uchar_Counter++;
				if (uchar_Counter >= 3) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 3:
				result = zk_str_findstr(data, pEvent->param2, ",", 1);
				if (result > 0) {
					//判断信号强度
					unsigned char IRQ;
					Buff_ASC2DEC(&data[result - 3], 2, &IRQ);
					Trace(1, "CSQ:%d", IRQ);
					if ((IRQ >= 15) && (IRQ <= 31)) {
						uchar_CDMA_Status++;
						uchar_Counter = 0;
					}
				}
				break;
			case 2:
			case 4:
			case 5:
			case 6:
			case 8:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 7:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				result += zk_str_findstr(data, pEvent->param2, "ERROR", 5);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 9:
				result = zk_str_findstr(data, pEvent->param2, "OPENED", 6);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				} else {
					result = zk_str_findstr(data, pEvent->param2, "CLOSED", 6);
					if (result > 0) {
						uchar_CDMA_Status--;
						uchar_Counter = 0;
					}
				}
				break;
			case 10:
				result = zk_str_findstr(data, pEvent->param2, "+TCPSETUP:0",
						11);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 11:
				result = zk_str_findstr(data, pEvent->param2, "CLOSED", 6);
				if (result > 0) {
					uchar_CDMA_Status--;
					uchar_Counter = 0;
				}
				Buff_HEX2ASC(data, pEvent->param2, uchar_ShowedHEX);
				Trace(1, "Translationed,length:%d,data:%s",
						(pEvent->param2) * 2, uchar_ShowedHEX);
				break;
			default:
				break;
			}
#endif
#if config_DeviceType==const_DT_MC8332
			//中兴MC8332
			switch (uchar_CDMA_Status) {
			case 0:
				result = zk_str_findstr(data, pEvent->param2, "MC8332", 6);
				if (result > 0)
					uchar_CDMA_Status++;
				break;
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 8:
			case 10:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0)
					uchar_CDMA_Status++;
				break;
			case 7:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				result += zk_str_findstr(data, pEvent->param2, "ERROR", 5);
				if (result > 0)
					uchar_CDMA_Status++;
				break;
			case 9:
				result = zk_str_findstr(data, pEvent->param2, "172.", 4);
				if (result > 0)
					uchar_CDMA_Status++;
				break;
			case 11:
				result = zk_str_findstr(data, pEvent->param2, "ESTABLISHED",
						11);
				if (result > 0) {
					uchar_CDMA_Status++;
					break;
				}
				result = zk_str_findstr(data, pEvent->param2, "CLOSED", 6);
				if (result > 0) {
					uchar_CDMA_Status--;
				}
				break;
			default:
				break;
			}
#endif
#if config_DeviceType==const_DT_ML5510
			switch (uchar_CDMA_Status) {
			case 2:
				result = zk_str_findstr(data, pEvent->param2, ":", 1);
				if (result > 0) {
					//缓存IMEI
					for (unsigned char i = 0; i < 15; i++)
						uchar_IMEI[i] = data[result + i];
//					Trace(1, "IMEI:%s", uchar_IMEI);
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 0:
			case 1:
			case 3:
			case 4:
			case 5:
			case 6:
			case 9:
			case 10:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 7:
				result = zk_str_findstr(data, pEvent->param2, ",", 1);
				if (result > 0) {
					//判断信号强度
					unsigned char IRQ;
					Buff_ASC2DEC(&data[result - 3], 2, &IRQ);
					Trace(1, "CSQ:%d", IRQ);
					if ((IRQ >= 10) && (IRQ <= 31)) {
						uchar_CDMA_Status++;
						uchar_Counter = 0;
					}
				}
				break;
			case 8:
				result = zk_str_findstr(data, pEvent->param2, "1", 1);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 11:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					uchar_Counter = 0;
				}
				result = zk_str_findstr(data, pEvent->param2, "CLOSED", 6);
				if (result > 0) {
					uchar_CDMA_Status--;
					uchar_Counter = 0;
				}
				result = zk_str_findstr(data, pEvent->param2, "ERROR", 5);
				if (result > 0) {
					uchar_CDMA_Status = 0;
					uchar_Counter = 0;
				}
				result = zk_str_findstr(data, pEvent->param2, "+NNMI:", 6);
				if (result > 0) {
					//取出长度
					unsigned char RevLen;
					Buff_ASC2DEC(&data[result + 5], 1, &RevLen);
					Trace(1, "Received,length:%d", RevLen);
					//取出内容
					unsigned char RevBuff[RevLen * 2];
					for (int i = 0; i < RevLen * 2; i++)
						uchar_RevBuff[i] = data[result + 7 + i];
					//解析数据
					if ((uchar_RevBuff[0] == '0')
							&& (uchar_RevBuff[1] == '1')) {
						Trace(1, "Received CMD");
						unsigned char SendBuff_ALL[22] =
								"AT+NMGS=5,0211110022\r\n";
						//复制mid
						for (int i = 0; i < 4; i++)
							SendBuff_ALL[12 + i] = uchar_RevBuff[2 + i];
						//复制CMD
						for (int i = 0; i < 2; i++)
							SendBuff_ALL[18 + i] = uchar_RevBuff[6 + i];
						//发送
						uint8_t UART1_temp[100];
						snprintf(UART1_temp, 22, SendBuff_ALL);
						UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
						Trace(1, "UART_Write:%s", UART1_temp);
					}
					uchar_Counter = 0;
				}
//				Buff_HEX2ASC(data, pEvent->param2, uchar_ShowedHEX);
//				Trace(1, "Translationed,length:%d,data:%s",
//						(pEvent->param2) * 2, uchar_ShowedHEX);
				break;
			case 12:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					uchar_CDMA_Status = 11;
					uchar_Counter = 0;
				}
				break;
			default:
				break;
			}
#endif
#if config_DeviceType==const_DT_ML810
			switch (uchar_CDMA_Status) {
			case 0:
				break;
			case 5:
			case 6:
			case 7:
			case 10:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 1:
				result = zk_str_findstr(data, pEvent->param2, "QCCID:", 6);
				if (result > 0) {
					for (i = 0; i < 20; i++)
						uchar_CCID[i] = data[result + i + 6];
					Trace(2, "CCID:%s", uchar_CCID);
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 2:
				result = zk_str_findstr(data, pEvent->param2, "CHN-CT", 6);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 3:
				result = zk_str_findstr(data, pEvent->param2, ",", 1);
				if (result > 0) {
					//判断是否注册
					unsigned char REG;
					Buff_ASC2DEC(&data[result], 1, &REG);
					Trace(1, "CEREG:%d", REG);
					if (REG == 1) {
						uchar_CDMA_Status++;
						uchar_Counter = 0;
					}
				}
				break;
			case 4:
				result = zk_str_findstr(data, pEvent->param2, ",", 1);
				if (result > 0) {
					//判断信号强度
					unsigned char IRQ;
					Buff_ASC2DEC(&data[result - 3], 2, &IRQ);
					Trace(1, "CSQ:%d", IRQ);
					if ((IRQ >= 10) && (IRQ <= 31)) {
						uchar_CDMA_Status++;
						uchar_Counter = 0;
					}
				}
				break;
//			case 4:
//				result = zk_str_findstr(data, pEvent->param2, ",", 1);
//				if (result > 0) {
//					//判断是否注册
//					unsigned char REG;
//					Buff_ASC2DEC(&data[result], 1, &REG);
//					Trace(1, "CGREG:%d", REG);
//					if (REG == 1) {
//						uchar_CDMA_Status++;
//						uchar_Counter = 0;
//					}
//				}
//				break;
			case 8:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					uchar_NetErrTimer = 0;
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				result = zk_str_findstr(data, pEvent->param2, "ERROR", 5);
				if (result > 0) {
					uchar_NetErrTimer++;
					Trace(2, "NetErrTimer:%d", uchar_NetErrTimer);
					if (uchar_NetErrTimer >= 5) {
						//如果连接失败次数达到5次以上，则切换到外网，获取电话号码
						uchar_NetErrTimer = 0;
						uchar_NetStatus = 2;
					}
					uchar_CDMA_Status = 0;
				}
				break;
			case 9:
//				result = zk_str_findstr(data, pEvent->param2, "5,1,1", 5);
				result = zk_str_findstr(data, pEvent->param2, "1,1,1", 5);
				if (result > 0) {
					//查找172.的网段，以确定是否为内网
					result = zk_str_findstr(data, pEvent->param2, "172.", 4);
					if (result > 0)
						uchar_NetStatus = 1;			//内网
					else
						uchar_NetStatus = 2;			//外网
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 11:
//				result = zk_str_findstr(data, pEvent->param2, "43437,0,2,", 10);
				if (uchar_NetStatus == 1)
					result = zk_str_findstr(data, pEvent->param2, "7004,0,2,",
							9);
				else
					result = zk_str_findstr(data, pEvent->param2, "7073,0,2,",
							9);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				break;
			case 12:
				result = zk_str_findstr(data, pEvent->param2, "recv", 4);
				if (result > 0) {
					uchar_CDMA_Status++;
					uchar_Counter = 0;
				}
				result = zk_str_findstr(data, pEvent->param2, "closed", 6);
				if (result > 0) {
					uchar_CDMA_Status = 0;
				}
				result = zk_str_findstr(data, pEvent->param2, "ERROR", 5);
				if (result > 0) {
					uchar_CDMA_Status = 0;
				}
				break;
			case 13:
				result = zk_str_findstr(data, pEvent->param2, "OK", 2);
				if (result > 0) {
					//*****取出收到的数据*****
					result = zk_str_findstr(data, pEvent->param2, "+QIRD:", 6);
					//取出长度位数
					unsigned char uchar_Tempbyte[300];
					for (i = 0; i < pEvent->param2 - result; i++)
						uchar_Tempbyte[i] = data[result + i + 6];
					result = zk_str_findstr(uchar_Tempbyte, 300, "\r\n", 2);
					unsigned char uchar_Len;
					Buff_ASC2DEC(&uchar_Tempbyte[0], result - 1, &uchar_Len);
					//取出内容
					unsigned char uchar_RevByte[uchar_Len];
					for (i = 0; i < uchar_Len; i++)
						uchar_RevByte[i] = uchar_Tempbyte[i + result + 1];
					//数据处理
					if ((uchar_RevByte[0] == 0xAA) && (uchar_RevByte[1] == 0xBB)
							&& (uchar_RevByte[uchar_Len - 2] == 0xBB)
							&& (uchar_RevByte[uchar_Len - 1] == 0xAA)) {
						unsigned char tmp_sendbuff[uchar_Len * 2];
						Buff_HEX2ASC(uchar_RevByte, uchar_Len, tmp_sendbuff);
						Trace(2, "RevLen:%d, Data Received:%s", uchar_Len,
								tmp_sendbuff);
						if (uchar_NetStatus == 1) {
							//内网数据处理
						} else {
							//外网数据处理
							//取出电话号码，写入配置文件
							unsigned char uchar_PhoneNum[12] = "000000000000";
							for (i = 0; i < 11; i++)
								uchar_PhoneNum[1 + i] = uchar_RevByte[2 + i];
							Trace(2, "GetPhoneNum:%s", uchar_PhoneNum);
							SaveData(uchar_PhoneNum, 12);
							for (i = 0; i < 12; i++)
								uchar_MacNum[i] = uchar_PhoneNum[i];
							//进入内网工作模式，并重启模块
							uchar_NetStatus = 1;
							uchar_CDMA_Status = 1;			//因为后面要-1，所以这里赋值为1
						}
					}
					uchar_CDMA_Status--;
				}
				break;
			default:
				break;
			}
#endif
		}
		break;
		/*GPRS*/
	case API_EVENT_ID_NO_SIMCARD:
		Trace(10, "TCP:!!NO SIM CARD%d!!!!", pEvent->param1);
		break;

	case API_EVENT_ID_NETWORK_REGISTERED_HOME:
	case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
		Trace(2, "TCP:network register success");
		uint_GPRSLED_Time = const_LEDTime_Find;
		flag = true;
		Network_StartAttach();
		break;

	case API_EVENT_ID_NETWORK_ATTACHED:
		Trace(2, "TCP:network attach success");
		Network_PDP_Context_t context = { .apn = "cmnet", .userName = "",
				.userPasswd = "" };
		Network_StartActive(context);
		break;

	case API_EVENT_ID_NETWORK_ACTIVATED:
		Trace(2, "TCP:network activate success");
		// if(sem)
		//     OS_ReleaseSemaphore(sem);
		sem = 1;
		break;

	case API_EVENT_ID_SOCKET_CONNECTED:
		Trace(2, "TCP:event connect");
		uint_GPRSLED_Time = const_LEDTime_Connected;
		// if(sem)
		//     OS_ReleaseSemaphore(sem);
		sem = 1;
		break;

	case API_EVENT_ID_SOCKET_SENT:
		// if(sem)
		//     OS_ReleaseSemaphore(sem);
		sem = 1;
		break;
	case API_EVENT_ID_SOCKET_RECEIVED: {
		int fd = pEvent->param1;
		int length =
				pEvent->param2 > RECEIVE_BUFFER_MAX_LENGTH ?
				RECEIVE_BUFFER_MAX_LENGTH :
																pEvent->param2;
		memset(buffer, 0, sizeof(buffer));
		length = Socket_TcpipRead(fd, buffer, length);
		Trace(2, "TCP:socket %d received %d bytes data:%s", fd, length, buffer);
		break;
	}
	case API_EVENT_ID_SOCKET_CLOSED: {
		int fd = pEvent->param1;
		Trace(2, "TCP:socket %d closed", fd);
		// if(sem)
		//     OS_ReleaseSemaphore(sem);
		sem = 1;
		break;
	}
	case API_EVENT_ID_SOCKET_ERROR: {
		int fd = pEvent->param1;
		Trace(2, "TCP:socket %d error occurred,cause:%d", fd, pEvent->param2);
		errorCode = pEvent->param2;
		// if(sem)
		//     OS_ReleaseSemaphore(sem);
		sem = 1;
		break;
	}
		/*LBS*/
	case API_EVENT_ID_NETWORK_CELL_INFO: {
		uint8_t number = pEvent->param1;
		Network_Location_t* location = pEvent->pParam1;
		Trace(2,
				"LBS:network cell infomation,serving cell number:1, neighbor cell number:%d",
				number - 1);

		for (int i = 0; i < number; ++i) {
			Trace(2, "LBS:cell %d info:%d%d%d,%d%d%d,%d,%d,%d,%d,%d,%d", i,
					location[i].sMcc[0], location[i].sMcc[1],
					location[i].sMcc[2], location[i].sMnc[0],
					location[i].sMnc[1], location[i].sMnc[2], location[i].sLac,
					location[i].sCellID, location[i].iBsic, location[i].iRxLev,
					location[i].iRxLevSub, location[i].nArfcn);
		}

		float longitude, latitude;
		if (!LBS_GetLocation(location, number, 15, &longitude, &latitude)) {
			Trace(1, "LBS:===LBS get location fail===");
		} else {
			Trace(1,
					"LBS:===LBS get location success,latitude:%d.%d,longitude:%d.%d===",
					(int) latitude,
					(int) ((latitude - (int) latitude) * 100000),
					(int) longitude,
					(int) ((longitude - (int) longitude) * 100000));
			uchar_LocationStatus = const_LBS_Location;
		}
		flag2 = true;
		break;
	}
	default:
		break;
	}
}

static void OnUart1ReceivedData(UART_Callback_Param_t param) {
	UART_Write(UART1, param.buf, param.length);
	Trace(1, "uart1 interrupt received data,length:%d,read:,data:%s",
			param.length, param.buf);
}

void UART1Task() {
#if config_WorkType == const_WT_Dustbin
#if config_DeviceType==const_DT_CM180
	//有方CM180
	UART_Config_t config = { .baudRate = UART_BAUD_RATE_115200, .dataBits =
			UART_DATA_BITS_8, .stopBits = UART_STOP_BITS_1, .parity =
			UART_PARITY_NONE, .rxCallback = OnUart1ReceivedData, .useEvent =
	false, };
#endif
#if config_DeviceType==const_DT_MC8332
	//中兴MC8332
	UART_Config_t config = { .baudRate = UART_BAUD_RATE_9600, .dataBits =
			UART_DATA_BITS_8, .stopBits = UART_STOP_BITS_1, .parity =
			UART_PARITY_NONE, .rxCallback = OnUart1ReceivedData, .useEvent =
	false, };
#endif
#if config_DeviceType==const_DT_ML5510
	UART_Config_t config = { .baudRate = UART_BAUD_RATE_9600, .dataBits =
			UART_DATA_BITS_8, .stopBits = UART_STOP_BITS_1, .parity =
			UART_PARITY_NONE, .rxCallback = OnUart1ReceivedData, .useEvent =
	false, };
#endif
#if config_DeviceType==const_DT_ML810
	UART_Config_t config = { .baudRate = UART_BAUD_RATE_115200, .dataBits =
			UART_DATA_BITS_8, .stopBits = UART_STOP_BITS_1, .parity =
			UART_PARITY_NONE, .rxCallback = OnUart1ReceivedData, .useEvent =
	false, };
#endif

	uint32_t UART1_times = 0;
	API_Event_t* event = NULL;
	config.useEvent = true;
	config.rxCallback = NULL;
	unsigned char i;

	UART_Init(UART1, config);
	config.rxCallback = NULL;

	uchar_CDMA_Status = 0;
	while (1) {
//        if(config.useEvent == false)
//        {
		uint8_t UART1_temp[1000];
		uint8_t UART1_buffer[1000];
		uchar_Counter = 0;

		//开机
//		OS_Sleep(3000);
//		GPIO_SetLevel(gpioLED1, 1);
//		GPIO_SetLevel(gpioLED1, 0);
//		OS_Sleep(1000);
//		GPIO_SetLevel(gpioLED1, 1);

#if config_DeviceType==const_DT_CM180
		//有方CM180
		while (uchar_CDMA_Status == 0) {
			if (uchar_Counter++ >= 30) {
				Trace(1, "Restarting CM180");
				GPIO_SetLevel(gpioLED1, 1);
				GPIO_SetLevel(gpioLED1, 0);
				OS_Sleep(1000);
				GPIO_SetLevel(gpioLED1, 1);
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 1) {
			snprintf(UART1_temp, 4, "AT\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 2) {
			snprintf(UART1_temp, 6, "ATE0\r\n");
//			snprintf(UART1_temp, 9, "AT+CGMR\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 3) {
			snprintf(UART1_temp, 9, "AT+CSQ?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 4) {
			snprintf(UART1_temp, 14, "AT+PNUM=#777\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 5) {
			snprintf(UART1_temp, 48,
					"AT+PLOGIN=1064915292453@wlwkchb.vpdn.cq,123456\r\n");
//			snprintf(UART1_temp, 22, "AT+ZPIDPWD=card,card\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 6) {
			snprintf(UART1_temp, 12, "AT+PLOGIN?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 7) {
			snprintf(UART1_temp, 13, "AT+PPPCLOSE\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 8) {
			snprintf(UART1_temp, 12, "AT+PPPOPEN\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 9) {
			snprintf(UART1_temp, 14, "AT+PPPSTATUS\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 10) {
			snprintf(UART1_temp, 33, "AT+TCPSETUP=0,172.20.1.200,7003\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 11) {
			snprintf(UART1_temp, 17, "AT+TCPSEND=0,25\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			OS_Sleep(130);
			UART_Write(UART1, uchar_HeartBeat, 27);
			unsigned char tmp_sendbuff[54];
			Buff_HEX2ASC(uchar_HeartBeat,27,tmp_sendbuff);
			Trace(1, "%s", tmp_sendbuff);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 20) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(30000);
		}
#endif
#if config_DeviceType==const_DT_MC8332
		//中兴MC8332
		while (uchar_CDMA_Status == 0) {
			snprintf(UART1_temp, 9, "AT+CGMM\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 1) {
			snprintf(UART1_temp, 6, "ATE0\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 2) {
			snprintf(UART1_temp, 13, "AT+GETICCID\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 3) {
			snprintf(UART1_temp, 9, "AT+CSQ?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 4) {
			snprintf(UART1_temp, 15, "AT+ZPNUM=#777\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 5) {
			snprintf(UART1_temp, 49,
					"AT+ZPIDPWD=1064915292453@wlwkchb.vpdn.cq,123456\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 6) {
			snprintf(UART1_temp, 13, "AT+ZPIDPWD?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 7) {
			snprintf(UART1_temp, 14, "AT+ZPPPCLOSE\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 8) {
			snprintf(UART1_temp, 13, "AT+ZPPPOPEN\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 9) {
			snprintf(UART1_temp, 13, "AT+ZIPGETIP\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 10) {
			snprintf(UART1_temp, 33, "AT+ZIPSETUP=0,172.20.1.200,7003\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 11) {
			snprintf(UART1_temp, 16, "AT+ZIPSTATUS=0\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			OS_Sleep(3000);
		}
#endif
#if config_DeviceType==const_DT_ML5510
		while (uchar_CDMA_Status == 0) {
			uint_GPRSLED_Time = const_LEDTime_Wait;
			snprintf(UART1_temp, 30, "AT+NCONFIG=AUTOCONNECT,FALSE\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				Trace(1, "Restarting ML5510");
				GPIO_SetLevel(gpioLED1, 1);
				GPIO_SetLevel(gpioLED1, 0);
				OS_Sleep(1000);
				GPIO_SetLevel(gpioLED1, 1);
				uchar_Counter = 0;
			}
			OS_Sleep(3000);
		}
		while (uchar_CDMA_Status == 1) {
			snprintf(UART1_temp, 11, "AT+CFUN=0\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 2) {
			snprintf(UART1_temp, 11, "AT+CGSN=1\r\n");
//			snprintf(UART1_temp, 9, "AT+CGMR\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 3) {
			snprintf(UART1_temp, 11, "AT+CGSN=1\r\n");
//			unsigned char uchar_sendbuff[61] = "AT+NSETPSK=000000000000000,0123456789ABCDEF0123456789ABCDEF\r\n";
//			for(i=0;i<15;i++) uchar_sendbuff[11+i] = uchar_IMEI[i];
//			snprintf(UART1_temp, 61, uchar_sendbuff);
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 4) {
			snprintf(UART1_temp, 11, "AT+CGSN=1\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 5) {
			snprintf(UART1_temp, 11, "AT+CFUN=1\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 6) {
			uint_GPRSLED_Time = const_LEDTime_Find;
			snprintf(UART1_temp, 12, "AT+CGATT=1\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 7) {
			snprintf(UART1_temp, 8, "AT+CSQ\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 8) {
			snprintf(UART1_temp, 12, "AT+CGATT?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 9) {
			snprintf(UART1_temp, 25, "AT+NCDP=180.101.147.115\r\n");
//			snprintf(UART1_temp, 26, "AT+NPING=180.101.147.115\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 10) {
			snprintf(UART1_temp, 11, "AT+NNMI=1\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 11) {
			uint_GPRSLED_Time = const_LEDTime_Connected;
			unsigned char SendBuff_ADC[4];
			unsigned char SendBuff_ALL[59] =
					"AT+NMGS=23,0011112222444444444444444444555555555555555555\r\n";
			SendBuff_ADC[0] = uint_ADC0 >> 8;
			SendBuff_ADC[1] = uint_ADC0 & 0x00FF;
			SendBuff_ADC[2] = uint_ADC1 >> 8;
			SendBuff_ADC[3] = uint_ADC1 & 0x00FF;
			Buff_HEX2ASC(SendBuff_ADC, 4, &SendBuff_ALL[13]);
			Buff_HEX2ASC(GPS_latitude, 9, &SendBuff_ALL[21]);
			Buff_HEX2ASC(GPS_longitude, 9, &SendBuff_ALL[39]);
			snprintf(UART1_temp, 59, SendBuff_ALL);
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			if (uchar_Counter++ >= 20) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(30000);
		}
		while (uchar_CDMA_Status == 12) {
			unsigned char SendBuff_ALL[22] = "AT+NMGS=5,0211110001\r\n";
			//复制mid
			for (int i = 0; i < 4; i++)
				SendBuff_ALL[12 + i] = uchar_RevBuff[2 + i];
			//复制CMD
			for (int i = 0; i < 2; i++)
				SendBuff_ALL[18 + i] = uchar_RevBuff[6 + i];
			//发送
			snprintf(UART1_temp, 22, SendBuff_ALL);
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s", UART1_temp);
			if (uchar_Counter++ >= 20) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
#endif
#if config_DeviceType==const_DT_ML810
		while (uchar_CDMA_Status == 0) {
			Trace(2, "Restarting ML810");
			uint_GPRSLED_Time = const_LEDTime_Wait;
			GPIO_SetLevel(gpioLED1, 1);
			GPIO_SetLevel(gpioLED1, 0);
			OS_Sleep(1000);
			GPIO_SetLevel(gpioLED1, 1);
			uchar_Counter = 0;
			OS_Sleep(1000);
			uchar_CDMA_Status++;
		}
		while (uchar_CDMA_Status == 1) {
			snprintf(UART1_temp, 10, "AT+QCCID\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 2) {
			snprintf(UART1_temp, 10, "AT+COPS?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 3) {
			snprintf(UART1_temp, 11, "AT+CEREG?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 4) {
			snprintf(UART1_temp, 8, "AT+CSQ\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 5) {
			uint_GPRSLED_Time = const_LEDTime_Find;
			snprintf(UART1_temp, 6, "ATE0\r\n");
//			snprintf(UART1_temp, 13, "AT+CGDCONT?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 6) {
			if (uchar_NetStatus == 1) {
				snprintf(UART1_temp, 68,
						"AT+QICSGP=1,1,\"ctvpdn\",\"00000000000@kchb.vpdn.cq\",\"123456\",1\r\n");
				for (i = 0; i < 11; i++)
					UART1_temp[24 + i] = uchar_MacNum[1 + i];
			} else {
				snprintf(UART1_temp, 39,
						"AT+QICSGP=1,1,\"ctnet\",\"0000\",\"0000\",1\r\n");
			}
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 7) {
			snprintf(UART1_temp, 14, "AT+QIDEACT=1\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(5000);
		}
		while (uchar_CDMA_Status == 8) {
			snprintf(UART1_temp, 12, "AT+QIACT=1\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(5000);
		}
		while (uchar_CDMA_Status == 9) {
			snprintf(UART1_temp, 11, "AT+QIACT?\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 10) {
			//内网
			if (uchar_NetStatus == 1) {
				snprintf(UART1_temp, 49,
						"AT+QIOPEN=1,0,\"TCP\",\"172.20.1.200\",7004,0,0\r\n");
				UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
				Trace(1, "UART_Write:%s.....Count:%d", UART1_temp,
						uchar_Counter);
				memset(UART1_buffer, 0, sizeof(UART1_buffer));
				if (uchar_Counter++ >= 30) {
					uchar_CDMA_Status = 0;
					uchar_Counter = 0;
				}
			}
			//外网
			else {
				snprintf(UART1_temp, 49,
						"AT+QIOPEN=1,0,\"TCP\",\"113.204.181.226\",7073,0,0\r\n");
				UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
				Trace(1, "UART_Write:%s.....Count:%d", UART1_temp,
						uchar_Counter);
				memset(UART1_buffer, 0, sizeof(UART1_buffer));
				if (uchar_Counter++ >= 30) {
					uchar_CDMA_Status = 0;
					uchar_Counter = 0;
				}
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 11) {
			uchar_SendTimer = 50;
			snprintf(UART1_temp, 16, "AT+QISTATE=1,0\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 30) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(1000);
		}
		while (uchar_CDMA_Status == 12) {
			uint_GPRSLED_Time = const_LEDTime_Connected;
			//内网
			if (uchar_NetStatus == 1) {
				if (uchar_GPSStatus == 1)
					uchar_GPSTimer++;
				else
					uchar_GPSTimer = 0;
				uchar_SendTimer++;
				if (uchar_GPSTimer >= 30)		//需要发送GPS数据
						{
					uchar_GPSTimer = 0;
					unsigned char SendBuff_ALL[246] =
							"AT+QISENDEX=0,\"AABB303030303030303030303030EB90FF13A65900010203040506070809000102030405060708090001020304050607080900010203040506070809000102030405060708090001020304050607080900010203040506070809000102030405060708090001020304050607080900EFBBAA\"\r\n";
					//赋予机号
					Buff_HEX2ASC(uchar_MacNum, 12, &SendBuff_ALL[19]);
					//赋予数据内容
					for (i = 0; i < 178; i++)
						SendBuff_ALL[57 + i] = uchar_GPSData[i];
					//计算校验
					zk_CntChkSUM(&SendBuff_ALL[43], 192, &SendBuff_ALL[235]);

					snprintf(UART1_temp, 246, SendBuff_ALL);
					UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
					Trace(2, "GPS Data:%s", UART1_temp);
					memset(UART1_buffer, 0, sizeof(UART1_buffer));

				} else		//如果不需要发送GPS数据，则判断是否发送报警信息或者心跳包
				{
					if (uchar_SendTimer >= 15) {
						uchar_SendTimer = 0;
						if (uchar_MustSendErr == 1)		//需要发送垃圾状态
								{
							uchar_MustSendErr = 0;
							unsigned char SendBuff_Full[88] =
									"AT+QISENDEX=0,\"AABB303030303030303030303030EB90FF13A80A00A083573E57F25DE16E00EFBBAA\"\r\n";
							unsigned char SendBuff_Emputy[88] =
									"AT+QISENDEX=0,\"AABB303030303030303030303030EB90FF13A80A00A083573E57F25D649600EFBBAA\"\r\n";
							unsigned char SendBuff_ALL[88];
							if (uchar_DustStatus == 1) {
								for (i = 0; i < 88; i++)
									SendBuff_ALL[i] = SendBuff_Full[i];
							} else {
								for (i = 0; i < 88; i++)
									SendBuff_ALL[i] = SendBuff_Emputy[i];
							}
							//赋予机号
							Buff_HEX2ASC(uchar_MacNum, 12, &SendBuff_ALL[19]);
							//计算校验
							zk_CntChkSUM(&SendBuff_ALL[43], 32,
									&SendBuff_ALL[75]);
							//准备发送的字符串
							snprintf(UART1_temp, 88, SendBuff_ALL);
							UART_Write(UART1, UART1_temp,
									strlen(UART1_temp) + 1);
							Trace(2, "DustStatus:%s", UART1_temp);
							memset(UART1_buffer, 0, sizeof(UART1_buffer));

						} else if (uchar_MustSendErr == 2)		//需要发送电压状态
								{
							uchar_MustSendErr = 0;
							//低压故障：4e4f8b5345659c96
							unsigned char SendBuff_ALL[88] =
									"AT+QISENDEX=0,\"AABB303030303030303030303030EB90FF13A80A00A04E4F8B5345659C9600EFBBAA\"\r\n";
							//赋予机号
							Buff_HEX2ASC(uchar_MacNum, 12, &SendBuff_ALL[19]);
							//计算校验
							zk_CntChkSUM(&SendBuff_ALL[43], 32,
									&SendBuff_ALL[75]);
							//准备发送的字符串
							snprintf(UART1_temp, 88, SendBuff_ALL);
							UART_Write(UART1, UART1_temp,
									strlen(UART1_temp) + 1);
							Trace(2, "LowPower:%s", UART1_temp);
							memset(UART1_buffer, 0, sizeof(UART1_buffer));

						} else {
							//准备发送心跳包
							unsigned char SendBuff_ALL[70] =
									"AT+QISENDEX=0,\"AABB303030303030303030303030EB90FF13A5000000EFBBAA\"\r\n";
							//赋予机号
							Buff_HEX2ASC(uchar_MacNum, 12, &SendBuff_ALL[19]);
							//计算校验
							zk_CntChkSUM(&SendBuff_ALL[43], 14,
									&SendBuff_ALL[57]);
							//准备发送的字符串
							snprintf(UART1_temp, 70, SendBuff_ALL);
							UART_Write(UART1, UART1_temp,
									strlen(UART1_temp) + 1);
							Trace(2, "HeartBeat:%s", UART1_temp);
							memset(UART1_buffer, 0, sizeof(UART1_buffer));
							if (uchar_Counter++ >= 20) {
								uchar_CDMA_Status = 0;
								uchar_Counter = 0;
							}
						}
					}
				}
			}
			//外网
			else {
				uchar_SendTimer++;
				if (uchar_SendTimer >= 15) {
					uchar_SendTimer = 0;
					//准备发送心跳包
					unsigned char SendBuff_ALL[66] =
							"AT+QISENDEX=0,\"AABB3030303030303030303030303030303030303030BBAA\"\r\n";
					//赋予机号
					Buff_HEX2ASC(uchar_CCID, 20, &SendBuff_ALL[19]);
					//准备发送的字符串
					snprintf(UART1_temp, 66, SendBuff_ALL);
					UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
					Trace(2, "SendCCID:%s", UART1_temp);
					memset(UART1_buffer, 0, sizeof(UART1_buffer));
					if (uchar_Counter++ >= 20) {
						uchar_CDMA_Status = 0;
						uchar_Counter = 0;
					}
				}
			}
			OS_Sleep(2000);
		}
		while (uchar_CDMA_Status == 13) {
			snprintf(UART1_temp, 16, "AT+QIRD=0,1500\r\n");
			UART_Write(UART1, UART1_temp, strlen(UART1_temp) + 1);
			Trace(1, "UART_Write:%s.....Count:%d", UART1_temp, uchar_Counter);
			memset(UART1_buffer, 0, sizeof(UART1_buffer));
			if (uchar_Counter++ >= 20) {
				uchar_CDMA_Status = 0;
				uchar_Counter = 0;
			}
			OS_Sleep(2000);
		}
#endif

		OS_Sleep(100);
	}
#endif
}

void LEDRUNTask(void *pData) {

	while (1) {
		GPIO_SetLevel(gpioLED1, uchar_RUNLED_Status);
		uchar_RUNLED_Status = !uchar_RUNLED_Status;
		OS_Sleep(uint_RUNLED_Time);
	}
}
void LEDGPSTask(void *pData) {

	while (1) {
		GPIO_SetLevel(gpioGPSLED, uchar_GPSLED_Status);
		uchar_GPSLED_Status = !uchar_GPSLED_Status;
		OS_Sleep(uint_GPSLED_Time);
	}
}
void LEDGPRSTask(void *pData) {

	while (1) {
		GPIO_SetLevel(gpioGPRSLED, uchar_GPRSLED_Status);
		uchar_GPRSLED_Status = !uchar_GPRSLED_Status;
		OS_Sleep(uint_GPRSLED_Time);
	}
}

void LBSTask(void *pData) {
//	char ip[16];

	int LBS_count = 0;

	while (!flag) {
		Trace(1, "LBS:wait for network register");
		OS_Sleep(3000);
	}

	while (1) {
		if (uchar_LocationStatus != const_GPS_Location) {
			flag2 = false;
			while (!flag2) {
				if (!Network_GetCellInfoRequst()) {
					Trace(1, "LBS:network get cell info fail");
				} else {
					flag2 = false;
				}
				Trace(1, "LBS:wait for lbs result");
				OS_Sleep(5000);
			}
			Trace(1, "LBS:times count:%d", ++LBS_count);
		}
		OS_Sleep(10000);
	}
}

void gps_testTask(void *pData) {
	GPS_Info_t* gpsInfo = Gps_GetInfo();
	uint8_t tmpbuffer[250];
	char buff1[15], buff2[15];

//wait for gprs register complete
//The process of GPRS registration network may cause the power supply voltage of GPS to drop,
//which resulting in GPS restart.
//	while (!flag) {
//		Trace(3, "GPS:wait for gprs regiter complete");
//		OS_Sleep(2000);
//	}

//open GPS hardware(UART2 open either)
	GPS_Init();
	GPS_Open(NULL);

//wait for gps start up, or gps will not response command
	while (gpsInfo->rmc.latitude.value == 0)
		OS_Sleep(1000);

// set gps nmea output interval
	for (uint8_t i = 0; i < 3; ++i) {
		bool ret = GPS_SetOutputInterval(10000);
		Trace(3, "GPS:set gps ret:%d", ret);
		if (ret)
			break;
		OS_Sleep(1000);
	}

//	 if(!GPS_ClearInfoInFlash())
//	     Trace(3,"erase gps fail");
//
//	 if(!GPS_SetQzssOutput(false))
//	     Trace(3,"enable qzss nmea output fail");
//
//	 if(!GPS_SetSearchMode(true,false,true,false))
//	     Trace(3,"set search mode fail");
//
//	 if(!GPS_SetSBASEnable(true))
//	     Trace(3,"enable sbas fail");

	if (!GPS_GetVersion(tmpbuffer, 150))
		Trace(3, "GPS:get gps firmware version fail");
	else
		Trace(3, "GPS:gps firmware version:%s", tmpbuffer);

//	 if(!GPS_SetFixMode(GPS_FIX_MODE_LOW_SPEED))
//	 Trace(3,"set fix mode fail");

	if (!GPS_SetOutputInterval(1000))
		Trace(3, "GPS:set nmea output interval fail");

	Trace(3, "GPS:init ok");

	while (1) {
		//show fix info
		uint8_t isFixed =
				gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ?
						gpsInfo->gsa[0].fix_type : gpsInfo->gsa[1].fix_type;
		char* isFixedStr;
		isFixedStr = "no fix";
		if (isFixed == 2) {
			uint_GPSLED_Time = const_LEDTime_Connected;
			uchar_LocationStatus = const_LBS_Location;
			uchar_GPSStatus = 1;
			isFixedStr = "2D fix";
		} else if (isFixed == 3) {
			uint_GPSLED_Time = const_LEDTime_Connected;
			uchar_LocationStatus = const_GPS_Location;
			uchar_GPSStatus = 1;
			if (gpsInfo->gga.fix_quality == 1)
				isFixedStr = "3D fix";
			else if (gpsInfo->gga.fix_quality == 2)
				isFixedStr = "3D/DGPS fix";
		} else {
			isFixedStr = "no fix";
			uint_GPSLED_Time = const_LEDTime_Wait;
			uchar_LocationStatus = const_LBS_Location;
			uchar_GPSStatus = 0;
		}

		//you can copy ` buff1,buff2 `(latitude,longitude) to http://www.gpsspg.com/maps.htm check location on map

		//convert unit ddmm.mmmm to degree(掳)
		int temp = (int) (gpsInfo->rmc.latitude.value
				/ gpsInfo->rmc.latitude.scale / 100);
		double latitude = temp
				+ (double) (gpsInfo->rmc.latitude.value
						- temp * gpsInfo->rmc.latitude.scale * 100)
						/ gpsInfo->rmc.latitude.scale / 60.0;
		temp = (int) (gpsInfo->rmc.longitude.value
				/ gpsInfo->rmc.longitude.scale / 100);
		double longitude = temp
				+ (double) (gpsInfo->rmc.longitude.value
						- temp * gpsInfo->rmc.longitude.scale * 100)
						/ gpsInfo->rmc.longitude.scale / 60.0;

		latitude *= 100;	//这里统一需要乘以100
		longitude *= 100;
		gcvt(latitude, 6, buff1);
		gcvt(longitude, 6, buff2);
		gcvt(latitude, 6, GPS_latitude);
		gcvt(longitude, 6, GPS_longitude);

//		//测试发送其它数据
//		snprintf(tmpbuffer, sizeof(tmpbuffer),
//				"GPStest!!!Date:%d-%d-%d,Time:%d-%d-%d-%d,Height:%d,Speed:%d",
//				gpsInfo->rmc.date.year, gpsInfo->rmc.date.month,
//				gpsInfo->rmc.date.day, gpsInfo->rmc.time.hours,
//				gpsInfo->rmc.time.minutes, gpsInfo->rmc.time.seconds,
//				gpsInfo->rmc.time.microseconds, gpsInfo->gga.altitude,
//				gpsInfo->rmc.speed);
//		Trace(3, tmpbuffer);

		//将数据按格式填入待发送字节
		if (isFixed == 3) {
			//年份
			unsigned int uint_Year = gpsInfo->rmc.date.year + 2000;
			unsigned char uchar_Year[2];
			uchar_Year[0] = uint_Year & 0x00FF;
			uchar_Year[1] = (uint_Year >> 8) & 0x00FF;
			Buff_HEX2ASC(uchar_Year, 2, &uchar_GPSData[0]);
			//月
			unsigned char uchar_Temp = gpsInfo->rmc.date.month;
			Buff_HEX2ASC(&uchar_Temp, 1, &uchar_GPSData[4]);
			//日
			uchar_Temp = gpsInfo->rmc.date.day;
			Buff_HEX2ASC(&uchar_Temp, 1, &uchar_GPSData[6]);
			//时
			uchar_Temp = gpsInfo->rmc.time.hours;
			uchar_Temp += 8;	//北京时间为格林威治时间+8
			if (uchar_Temp >= 24)
				uchar_Temp -= 24;
			Buff_HEX2ASC(&uchar_Temp, 1, &uchar_GPSData[8]);
			//分
			uchar_Temp = gpsInfo->rmc.time.minutes;
			Buff_HEX2ASC(&uchar_Temp, 1, &uchar_GPSData[10]);
			//秒
			uchar_Temp = gpsInfo->rmc.time.seconds;
			Buff_HEX2ASC(&uchar_Temp, 1, &uchar_GPSData[12]);
			//纬度
			Buff_HEX2ASC(GPS_latitude, 9, &uchar_GPSData[14]);
			//经度
			Buff_HEX2ASC(GPS_longitude, 9, &uchar_GPSData[56]);
			//高度：暂时全0
			//速度：暂时全0
		}

		//you can copy ` latitude,longitude ` to http://www.gpsspg.com/maps.htm check location on map

		snprintf(tmpbuffer, sizeof(tmpbuffer),
				"GPS:GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f",
				gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
				gpsInfo->gga.fix_quality, gpsInfo->gga.satellites_tracked,
				gpsInfo->gsv[0].total_sats, isFixedStr, latitude, longitude,
				gpsInfo->gga.altitude);
		//show in tracer
		Trace(3, tmpbuffer);
//		Trace(3, "GPS: %s,%s", buff1, buff2);
//        //send to UART1
//        UART_Write(UART1,tmpbuffer,strlen(tmpbuffer));
//        UART_Write(UART1,"\r\n\r\n",4);

		OS_Sleep(5000);
	}
}

void CreateSem(HANDLE* sem_) {
	*sem_ = 0;
// *sem = OS_CreateSemaphore(0);
}

void WaitSem(HANDLE* sem_) {
// OS_WaitForSemaphore(*sem,OS_WAIT_FOREVER);
// OS_DeleteSemaphore(*sem);
// *sem = NULL;
	while (*sem_ == 0)
		OS_Sleep(1);
	*sem_ = 0;
}

bool Connect() {
	memset(buffer, 0, sizeof(buffer));
	if (DNS_GetHostByName2(DNS_DOMAIN, (char*) buffer) != 0)
		return false;
	Trace(2, "TCP:DNS,domain:%s,ip:%s,strlen(ip):%d", DNS_DOMAIN, buffer,
			strlen(buffer));
	CreateSem(&sem);
	socketFd = Socket_TcpipConnect(TCP, buffer, SERVER_PORT);
	Trace(2, "TCP:connect tcp server,socketFd:%d", socketFd);
	WaitSem(&sem);
	Trace(2, "TCP:connect end");
	if (errorCode != 0) {
		errorCode = 0;
		Trace(2, "TCP:error ocurred");
		return false;
	}
	return true;
}
bool Write(uint8_t* data, uint16_t len) {
	Trace(2, "TCP:Write");
	CreateSem(&sem);
	int ret = Socket_TcpipWrite(socketFd, data, len);
	if (ret <= 0) {
		Trace(2, "TCP:socket write fail:%d", ret);
		return false;
	}
	Trace(2, "TCP:### socket %d send %d bytes data to server:%s,ret:%d",
			socketFd, len, data, ret);
	WaitSem(&sem);
	Trace(2, "TCP:### write end");
	if (errorCode != 0) {
		errorCode = 0;
		Trace(2, "TCP:error ocurred");
		return false;
	}
	return true;
}

bool Close() {
	CreateSem(&sem);
	Socket_TcpipClose(socketFd);
	WaitSem(&sem);
	return true;
}

void socketTestTask(void* param) {
	int i, failCount = 0;
	int TCP_count = 0;
	char Sendbuff[39] = "CharlesDMay's here:          ,         ";
	WaitSem(&sem);
	Trace(2, "TCP:sem:%d,%p", (int) sem, (void*) sem);
	Trace(1, "TCP:start connect now");
	Connect();
	while (1) {
		if (failCount == 5) {
			Close();
		}
		if (failCount >= 5) {
			if (Connect())
				failCount = 0;
			else
				++failCount;
		} else {
			for (i = 0; i < 9; i++) {
				Sendbuff[20 + i] = GPS_latitude[i];
				Sendbuff[30 + i] = GPS_longitude[i];
			}
			if (!Write(Sendbuff, 39)) {
				++failCount;
				Trace(2, "TCP:write fail");
			}
		}
		Trace(2, "TCP:count:%d", TCP_count++);
		OS_Sleep(10000);
	}
}

void ADCTask(void *pData) {
	uint16_t value = 0, mV = 0;
	ADC_Config_t config0 = { .channel = ADC_CHANNEL_0, .samplePeriod =
			ADC_SAMPLE_PERIOD_100MS };
	ADC_Config_t config1 = { .channel = ADC_CHANNEL_1, .samplePeriod =
			ADC_SAMPLE_PERIOD_100MS };
	ADC_Init(config0);
	ADC_Init(config1);

	while (1) {
		if (ADC_Read(ADC_CHANNEL_0, &value, &mV)) {
			Trace(1, "ADC0 value:%d, %dmV", value, mV * 13);
			uint_ADC0 = mV * 13;
			//判断工作电压
			if ((uint_ADC0 < const_PowerLevel) && (uchar_PowerStatus != 0)
					&& (uchar_MustSendErr == 0)) {
				uchar_MustSendErr = 2;
				uchar_PowerStatus = 0;
			} else if ((uint_ADC0 >= const_PowerLevel)
					&& (uchar_PowerStatus != 1) && (uchar_MustSendErr == 0)) {
				uchar_PowerStatus = 1;
			}
		}
		if (ADC_Read(ADC_CHANNEL_1, &value, &mV)) {
			Trace(1, "ADC1 value:%d, %dmV", value, mV * 13);
			uint_ADC1 = mV * 13;
			//判断是否有垃圾
			if ((uint_ADC1 >= const_DustLevel) && (uchar_DustStatus != 1)
					&& (uchar_MustSendErr == 0)) {
				uchar_MustSendErr = 1;
				uchar_DustStatus = 1;
			} else if ((uint_ADC1 < const_DustLevel) && (uchar_DustStatus != 0)
					&& (uchar_MustSendErr == 0)) {
				uchar_MustSendErr = 1;
				uchar_DustStatus = 0;
			}
		}
		OS_Sleep(9000);
	}
}

void File_CtrlTask(void *pData) {
	OS_Sleep(3000);
//读取MacNum
	if (ReadData(uchar_MacNum, 12) == false) {
		Trace(2, "Read FILE failed,Start to Write.");
		for (int i = 0; i < 12; i++)
			uchar_MacNum[i] = 0x30;
		SaveData(uchar_MacNum, 12);
	}
	while (1)
		OS_Sleep(10000);
}

void MainTask(void *pData) {
	API_Event_t* event = NULL;

	uchar_LocationStatus = 0;
//    PM_PowerEnable(POWER_TYPE_VPAD,true);
	GPIO_Init(gpioLED1);
	GPIO_Init(gpioGPRSLED);
	GPIO_Init(gpioGPSLED);
	uchar_RUNLED_Status = 0;
	uint_RUNLED_Time = 1000;	//运行指示灯默认1秒
	uint_GPSLED_Time = const_LEDTime_Wait;
	uint_GPRSLED_Time = const_LEDTime_Wait;

	uchar_DustStatus = 2;		//垃圾状态上电默认为其它
	uchar_PowerStatus = 1;		//电源状态上电默认为正常
	uchar_MustSendErr = 0;		//需要发送报警信息标志：0-不需要发送；1-需要发送垃圾状态；2-需要发送电压状态
	uchar_GPSStatus = 0;
	uchar_GPSTimer = 0;			//GPS发送计时器

	uchar_NetStatus = 1;		//启动默认运行在内网模式
	uchar_NetErrTimer = 0;

#if config_WorkType == const_WT_Dustbin
	//打开FILE的进程
		FileCtrlHandle = OS_CreateTask(File_CtrlTask,
		NULL, NULL, SECOND_TASK_STACK_SIZE, SECOND_TASK_STACK_SIZE, 0, 0,
		FILE_TASK_NAME);

	//打开LED的进程
	//	ledRunTaskHandle = OS_CreateTask(LEDRUNTask, NULL, NULL,
	//	SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0,
	//	SECOND_TASK_NAME);
		ledGPSTaskHandle = OS_CreateTask(LEDGPSTask, NULL, NULL,
		SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0,
		SECOND_TASK_NAME);
		ledGPRSTaskHandle = OS_CreateTask(LEDGPRSTask, NULL, NULL,
		SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0,
		SECOND_TASK_NAME);

	//	//打开LBS进程，每10秒获取一次定位信息
	//	LBSTaskHandle = OS_CreateTask(LBSTask,
	//	NULL, NULL, LBS_TASK_STACK_SIZE, LBS_TASK_PRIORITY, 0, 0,
	//	LBS_TASK_NAME);

	//打开GPS进程，每5秒获取一次定位信息
	GPSTaskHandle = OS_CreateTask(gps_testTask,
	NULL, NULL, GPS_TASK_STACK_SIZE, GPS_TASK_PRIORITY, 0, 0,
	GPS_TASK_NAME);

	//打开ADC进程，每9秒获取一次ADC信息
	ADCTaskHandle = OS_CreateTask(ADCTask,
	NULL, NULL, ADC_TASK_STACK_SIZE, ADC_TASK_PRIORITY, 0, 0,
	ADC_TASK_NAME);

#endif

#if config_WorkType == const_WT_GPRSMOD
	//打开GPRS进程
	CreateSem(&sem);
	GPRSTaskHandle = OS_CreateTask(socketTestTask,
	NULL, NULL, GPRS_TASK_STACK_SIZE, GPRS_TASK_PRIORITY, 0, 0, GPRS_TASK_NAME);
#endif

//打开UART1进程
	uart1TaskHandle = OS_CreateTask(UART1Task,
	NULL, NULL, UART1_TASK_STACK_SIZE, UART1_TASK_PRIORITY, 0, 0,
	UART1_TASK_NAME);

//	GPIO_SetLevel(gpioLED1, 1);
//	GPIO_SetLevel(gpioLED1, 0);
//	OS_Sleep(1000);
//	GPIO_SetLevel(gpioLED1, 1);

	while (1) {
		if (OS_WaitEvent(mainTaskHandle, (void**) &event,
		OS_TIME_OUT_WAIT_FOREVER)) {
			EventDispatch(event);
			OS_Free(event->pParam1);
			OS_Free(event->pParam2);
			OS_Free(event);
		}
	}
}

void ES_Dustbin_Main(void) {
	mainTaskHandle = OS_CreateTask(MainTask, NULL, NULL,
	MAIN_TASK_STACK_SIZE,
	MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
	OS_SetUserMainHandle(&mainTaskHandle);
}
