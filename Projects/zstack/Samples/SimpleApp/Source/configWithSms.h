//通过短信内容配置信道等参数

#ifndef __CONFIG_WITH_SMS_H__
#define __CONFIG_WITH_SMS_H__
#include "hal_types.h"
#include "sim900.h"



extern bool match_phoneNum(uint8 *p, PHONE_NUM_t *phoneNum);

extern void match_SmsData(uint8 *p, uint8 *data);



#endif //__CONFIG_WITH_SMS_H__


