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

//ͬ��ͷ�궨��
#define HEAD                            0X5A            //ͬ��ͷ
//�ն��йغ궨��
#define  ACC_VALUE	                0X02		//����������ٶ�ֵ����
#define  ALARM_STATUS		        0x04	        //����״̬����

#define  TUMBLE				0X01		//����
#define  WALK				0X03		//����
#define  UPSTAIRS			0X05		//��¥
#define  DOWNSTAIRS			0X07		//��¥
#define	 URGENTCALL			0X09		//��������

//Э�����йغ궨��
#define  ACK			        0x06		//Ӧ������
#define	 CONFIG				0x08		//��������
#define	 GETINFOR			0x0A		//��ȡ��Ӳ���汾��
#define  RESTART			0X0C		//�����豸
#define  GETACC				0X0E		//��ȡ������ٶ�ֵ


#define  POWER				0X09		//���ù���
#define	 CHANNEL		        0X0B		//Ƶ��ѡ��
#define	 PERIOD				0X0D		//��������
#define MAX_sousuo          15      //ƥ���ַ�����󳤶�



/******************************************************************************
 * PUBLIC FUNCTIONS
 */

void initUart(halUARTCBack_t pf);
void uartRxCB( uint8 port, uint8 event );

#endif // SIMPLE_APP_H




