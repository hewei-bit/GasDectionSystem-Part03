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

//??????????????????????????????
#define FLAG_GRP_RTC_WAKEUP					0x01
#define FLAG_GRP_RTC_ALARM_A				0x02
#define FLAG_GRP_RTC_ALARM_B				0x04

#define FLAG_GRP_KEY0_DOWN					0x10
#define FLAG_GRP_KEY1_DOWN					0x20
#define FLAG_GRP_KEY2_DOWN					0x40
#define FLAG_GRP_WK_UP_DOWN					0x80


#define MAX_CONVERTED_VALUE 	4095  	//??????????
#define VREF					3300	//??????????
#define QUEUE_NUM		10				//????????????
#define CORE_OBJ_NUM	2				//??????????????????3????2??????????????????????						





extern OS_FLAG_GRP			g_flag_grp;		

extern OS_MUTEX				g_mutex_printf;		//????????????

extern OS_Q	 				g_queue_usart1;				//??????????????
extern OS_Q	 				g_queue_usart2;				//??????????????
extern OS_Q	 				g_queue_usart3;				//??????????????
extern OS_Q	 				g_queue_usart4;				//??????????????
extern OS_Q	 				g_queue_usart5;				//??????????????


extern void dgb_printf_safe(const char *format, ...);





#endif



