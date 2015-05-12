#ifndef _COMM_APP_H
#define _COMM_APP_H

enum {DISARMED=0,REQ_ARM,ARMED,REQ_DISARM};

#define MSP_SET_THRO  1
#define MSP_SET_YAW		2
#define MSP_SET_PITCH	3
#define MSP_SET_ROLL	4
#define MSP_ARM_IT		5
#define MSP_DISARM_IT	6
#define MSP_SET_4CON	7
#define MSP_SETOFF		8
#define MSP_LAND_DOWN 9
#define MSP_HOLD_ALT 	10
#define MSP_STOP_HOLD_ALT 11
#define MSP_HEAD_FREE 12
#define MSP_STOP_HEAD_FREE	13
#define MSP_POS_HOLD 14
#define MSP_STOP_POS_HOLD 15
#define MSP_FLY_STATE	16
#define MSP_ACC_CALI	205

#define APP_YAW_DB	 70 //dead band 
#define APP_PR_DB		 50




#define CUT_DB(x,mid,DB) {if(fabs(x-mid)<DB) x=mid; \
											else if(x-mid>0) x=x-DB;\
											else if(x-mid<0) x=x+DB;}

extern uint16_t rcData[4];
extern uint8_t appCmdFlag;
extern uint32_t lastGetRCTime;
extern uint8_t flyLogApp;
extern uint8_t armState;

void CommAppCmdProcess(void);
void CommApp(unsigned char ch);
void CommAppUpload(void);
void RCDataProcess(void);

#endif

