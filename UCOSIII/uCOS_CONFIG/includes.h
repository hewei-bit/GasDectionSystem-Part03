/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : includes.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  INCLUDES_MODULES_PRESENT
#define  INCLUDES_MODULES_PRESENT


/*
*********************************************************************************************************
*                                         STANDARD LIBRARIES
*********************************************************************************************************
*/


#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include  <math.h>


/*
*********************************************************************************************************
*                                                 OS
*********************************************************************************************************
*/

#include  <os.h>


/*
*********************************************************************************************************
*                                              LIBRARIES
*********************************************************************************************************
*/

#include  <cpu.h>
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <lib_str.h>

/*
*********************************************************************************************************
*                                              APP / BSP
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <bsp.h>
//#include  <bsp_int.h>

//�¼���־�飺Ӳ��ʹ�õ��ı�־λ
#define FLAG_GRP_RTC_WAKEUP					0x01
#define FLAG_GRP_RTC_ALARM_A				0x02
#define FLAG_GRP_RTC_ALARM_B				0x04

#define FLAG_GRP_KEY0_DOWN					0x10
#define FLAG_GRP_KEY1_DOWN					0x20
#define FLAG_GRP_KEY2_DOWN					0x40
#define FLAG_GRP_WK_UP_DOWN					0x80


#define MAX_CONVERTED_VALUE 	4095  	//���ת��ֵ
#define VREF					3300	//����ѹֵ
#define QUEUE_NUM		10				//��Ϣ���г���
#define CORE_OBJ_NUM	2				//�ں˶��������һ��3����2���ź�����һ����Ϣ����						


//�ڵ�ṹ��
typedef struct 
{
	int device_id;
	double lora_address;
	int lora_channel;
	char temperature[10];
	char humidity[10];
	char CH4concentration[10];
} NODE;


extern OS_FLAG_GRP			g_flag_grp;		

extern OS_MUTEX				g_mutex_printf;		//�������Ķ���

extern OS_Q	 				g_queue_usart1;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart2;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart3;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart4;				//��Ϣ���еĶ���
extern OS_Q	 				g_queue_usart5;				//��Ϣ���еĶ���


extern void dgb_printf_safe(const char *format, ...);





#endif



