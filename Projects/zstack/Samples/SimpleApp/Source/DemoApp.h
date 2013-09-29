#ifndef SIMPLE_APP_H
#define SIMPLE_APP_H

/******************************************************************************
 * CONSTANTS
 */

#define MY_PROFILE_ID                     0x0F20
#define MY_ENDPOINT_ID                    0x02

// Define devices
#define DEV_ID_SENSOR                     1
#define DEV_ID_COLLECTOR                  2
#define DEV_ID_ROUTER                     3
#define DEVICE_VERSION_SENSOR             1
#define DEVICE_VERSION_COLLECTOR          1
#define DEVICE_VERSION_ROUTER             1
// Define the Command ID's used in this application
#define SENSOR_REPORT_CMD_ID              2
#define DUMMY_REPORT_CMD_ID               3
#define SENSOR_ALARM_CMD_ID               4
// Sensor report data format
#define SENSOR_TEMP_OFFSET                0
#define SENSOR_VOLTAGE_OFFSET             1
#define SENSOR_PARENT_OFFSET              2
#define SENSOR_REPORT_LENGTH              4

#define RX_BUF_LEN                        128

//同步头宏定义
#define HEAD                            0X5A            //同步头
//终端有关宏定义
#define  ACC_VALUE	                0X02		//报告三轴加速度值命令
#define  ALARM_STATUS		        0x04	        //报警状态命令

#define  TUMBLE				0X01		//跌倒
#define  WALK				0X03		//步行
#define  UPSTAIRS			0X05		//上楼
#define  DOWNSTAIRS			0X07		//下楼
#define	 URGENTCALL			0X09		//紧急呼叫

//协调器有关宏定义
#define  ACK			        0x06		//应答命令
#define	 CONFIG				0x08		//配置命令
#define	 GETINFOR			0x0A		//获取软硬件版本号
#define  RESTART			0X0C		//重启设备
#define  GETACC				0X0E		//获取三轴加速度值


#define  POWER				0X09		//配置功率
#define	 CHANNEL		        0X0B		//频道选择
#define	 PERIOD				0X0D		//发射周期
#define MAX_sousuo          15      //匹配字符串最大长度



/******************************************************************************
 * PUBLIC FUNCTIONS
 */

void initUart(halUARTCBack_t pf);
void uartRxCB( uint8 port, uint8 event );

#endif // SIMPLE_APP_H




