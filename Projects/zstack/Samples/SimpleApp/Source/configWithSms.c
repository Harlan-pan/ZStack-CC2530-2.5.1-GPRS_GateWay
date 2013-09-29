#include "configwithsms.h"
#include "osal.h"
#include "sim900.h"

/*
+CMGR: "REC READ","+8613918186089","02/01/30,20:40:31+00"
This is a test
   
OK
*/
bool match_phoneNum(uint8 *p, PHONE_NUM_t *phoneNum)
{
  uint8 *ptp = p;
  uint8 *ptphoneNum = phoneNum->number;
  uint8 sybCnt = 0;
  
  while(1)
  {
    if (*(ptp++) == '"')
    {
      sybCnt ++;
    }
    
    //找到第三个双引号时就是手机号码
    if (sybCnt == 3)
    {
      sybCnt = 0;
      while (*ptp != '"')
      {
        *(ptphoneNum++) = *(ptp++);
        sybCnt++; //计算号码长度
        
        if (sybCnt>MAX_PHONE_NUMBER_LENGHT)
        {
          return FALSE;
        }
      }
      
      phoneNum->lenght = sybCnt;
      return TRUE;
    }
  }
}


void match_SmsData(uint8 *p, uint8 *data)
{
}