//ºËÐÄÍ·ÎÄ¼þ
#include "includes.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
//´®¿Ú
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
//ÍâÉè
#include "lora_app.h"
#include "led.h"
#include "beep.h"
#include "dht11.h"
#include "mq135.h"
#include "oled.h"
#include "key.h"
#include "rtc.h"
#include "lcd.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

#include "malloc.h"
#include "sdio_sdcard.h"   



/****************************************************************
*Ãû    ³Æ:ÊäÆøÕ¾³¡ÆøÌåÐ¹Â©¼ì²âÏµÍ³
*×÷    Õß:ºÎÎµ
*´´½¨ÈÕÆÚ:2020/10/14 ½¨Á¢ÏµÍ³¿ò¼Ü
*µ±Ç°ÈÕÆÚ:2020/11/16 Íê³ÉV1.0
*µ±Ç°ÈÕÆÚ:2020/12/24 Íê³ÉV2.0
*µ±Ç°ÈÕÆÚ:2021/01/08 Íê³ÉV3.0
*µ±Ç°ÈÕÆÚ:2021/03/09 Íê³ÉV4.0
*µ±Ç°ÈÕÆÚ:2021/03/10 Íê³É¶¨µã¼ì²â½Úµã´úÂë
*µ±Ç°ÈÕÆÚ:2021/03/11 Íê³ÉÒ£²â¼ì²â½Úµã´úÂë
*µ±Ç°ÈÕÆÚ:2021/03/12 Íê³ÉÖÐ¼Ì½Úµã´úÂë



*ÈÎÎñ£º
	1.¿ªÊ¼ÈÎÎñ ´´½¨ÐÅºÅÁ¿ ÏûÏ¢¶ÓÁÐ »¥³âËø ÊÂ¼þ±êÖ¾×é ÈÎÎñ
	2.DHT11ÎÂÊª¶È²É¼¯,µ¥×ÜÏß²É¼¯£¬  	»¥³âËø lcdÏ	ÔÊ¾		Êý¾ÝÍ¨¹ýÏûÏ¢¶ÓÁÐ·¢ËÍÊý¾Ýµ½Ïß³Ì´æ´¢¡¢×ª·¢ÈÎÎñ
	3.TDLASÆøÌåÅ¨¶È²É¼¯£¬´®¿Ú½ÓÊÕÅ¨¶ÈÊý¾Ý£¬»¥³âËø lcdÏÔÊ¾	Êý¾ÝÍ¨¹ýÏûÏ¢¶ÓÁÐ·¢ËÍÊý¾Ýµ½Ïß³Ì´æ´¢¡¢×ª·¢ÈÎÎñ        
	4.MQ135 Å¨¶È²É¼¯
	5.MQ4 Å¨¶È²É¼¯
	6.¿´ÃÅ¹·
	7.±¾µØ´æ´¢ÈÎÎñ µÈ´ý¶à¸öÄÚºË¶ÔÏó ÏûÏ¢¶ÓÁÐ½ÓÊÕÊý¾Ý ÒÔtxt¸ñÊ½´æÈëSD¿¨
	8.LORA×ª·¢ µÈ´ý¶à¸öÄÚºË¶ÔÏó ÏûÏ¢¶ÓÁÐ½ÓÊÕÊý¾Ý usart3 ·¢ËÍÖÁÉÏÎ»»ú
	9.RTCÊ±¼äÏÔÊ¾ »¥³âËø oledÏÔÊ¾	
	10.LED0 ÐÅºÅÁ¿½ÓÊÜÊý¾Ý ±¨¾¯
	11.LED1 ÏµÍ³ÔËÐÐÌáÊ¾
	12.BEEP ÐÅºÅÁ¿½ÓÊÜÊý¾Ý ±¨¾¯
	13.KEY 
	14.ÈÎÎñ×´Ì¬
	15.ÏµÍ³ÔËÐÐ¼ì²é
	16.mpu6050 ÁùÖá´«¸ÐÆ÷ ÏÔÊ¾µ±Ç°ÔÆÌ¨½Ç¶ÈÊý¾Ý 
	
*Ëµ  Ã÷:		
	µ±Ç°´úÂë¾¡¿ÉÄÜÊµÏÖÁËÄ£¿é»¯±à³Ì£¬Ò»¸öÈÎÎñ¹ÜÀíÒ»¸öÓ²¼þ¡£×î¼òµ¥µÄ
	led¡¢·äÃùÆ÷¶¼ÓÉµ¥¶ÀÈÎÎñ¹ÜÀí¡£
*****************************************************************/

//V1.0 Íê³ÉÈÎÎñºÍÄÚºË´´½¨ ´«¸ÐÆ÷Êý¾Ý²É¼¯ jsonÊý¾Ý·â×°
//V2.0 Íê³ÉlcdÏÔÊ¾ ´®¿ÚÏìÓ¦  LORAÈÎÎñÖÐdht11ºÍTDLASµÄÏûÏ¢¶ÓÁÐ´«Êä
//V3.0 Íê³ÉÉÏÎ»»ú¶ÔÏÂÎ»»úµÄÊý¾ÝÎÊÑ¯
//V4.0 Íê³ÉloraÐÅÏ¢´«Êä 

/*****************************¶¨ÒåÈÎÎñ¶ÑÕ»*************************************/
//UCOSIIIÖÐÒÔÏÂÓÅÏÈ¼¶ÓÃ»§³ÌÐò²»ÄÜÊ¹ÓÃ£¬ALIENTEK
//½«ÕâÐ©ÓÅÏÈ¼¶·ÖÅä¸øÁËUCOSIIIµÄ5¸öÏµÍ³ÄÚ²¿ÈÎÎñ
//ÓÅÏÈ¼¶0£ºÖÐ¶Ï·þÎñ·þÎñ¹ÜÀíÈÎÎñ OS_IntQTask()
//ÓÅÏÈ¼¶1£ºÊ±ÖÓ½ÚÅÄÈÎÎñ OS_TickTask()
//ÓÅÏÈ¼¶2£º¶¨Ê±ÈÎÎñ OS_TmrTask()
//ÓÅÏÈ¼¶OS_CFG_PRIO_MAX-2£ºÍ³¼ÆÈÎÎñ OS_StatTask()
//ÓÅÏÈ¼¶OS_CFG_PRIO_MAX-1£º¿ÕÏÐÈÎÎñ OS_IdleTask()


//ÈÎÎñ1 ¿ªÊ¼ÈÎÎñ
#define START_TASK_PRIO		3	
#define START_STK_SIZE 		512
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

//ÈÎÎñ2 DHT11ÎÂÊª¶È²É¼¯ 
#define DHT11_TASK_PRIO 	4
#define DHT11_STK_SIZE		128
OS_TCB DHT11_Task_TCB;
CPU_STK DHT11_TASK_STK[DHT11_STK_SIZE];
void DHT11_task(void *parg);

//ÈÎÎñ3 TDLAS ÆøÌåÅ¨¶È²É¼¯	usart2
#define TDLAS_TASK_PRIO 	5
#define TDLAS_STK_SIZE		512
OS_TCB TDLAS_Task_TCB;
CPU_STK TDLAS_TASK_STK[TDLAS_STK_SIZE];
void TDLAS_task(void *parg);

//ÈÎÎñ4  mq135 ÆøÌåÅ¨¶È²É¼¯	
#define MQ135_TASK_PRIO 	5
#define MQ135_STK_SIZE		128
OS_TCB MQ135_Task_TCB;
CPU_STK MQ135_TASK_STK[MQ135_STK_SIZE];
void MQ135_task(void *parg);

//ÈÎÎñ5 mq4 ÆøÌåÅ¨¶È²É¼¯	
#define MQ4_TASK_PRIO 	5
#define MQ4_STK_SIZE		128
OS_TCB MQ4_Task_TCB;
CPU_STK MQ4_TASK_STK[MQ4_STK_SIZE];
void MQ4_task(void *parg);

//ÈÎÎñ6 ¿´ÃÅ¹· 
#define IWG_TASK_PRIO 		6
#define IWG_STK_SIZE		128
OS_TCB IWG_Task_TCB;
CPU_STK IWG_TASK_STK[IWG_STK_SIZE];
void IWG_task(void *parg);

//ÈÎÎñ7 µÈ´ý¶à¸öÄÚºË¶ÔÏó ÏûÏ¢¶ÓÁÐ½ÓÊÕÊý¾Ý ÒÔtxt¸ñÊ½´æÈëSD¿¨
#define SAVE_TASK_PRIO 		9
#define SAVE_STK_SIZE		512
OS_TCB SAVE_Task_TCB;
CPU_STK SAVE_TASK_STK[SAVE_STK_SIZE];
void SAVE_task(void *parg);

//ÈÎÎñ8 LORA×ª·¢ µÈ´ý¶à¸öÄÚºË¶ÔÏó ÏûÏ¢¶ÓÁÐ½ÓÊÕÊý¾Ý usart3 ·¢ËÍÖÁÉÏÎ»»ú
#define LORA_TASK_PRIO 		8
#define LORA_STK_SIZE		512
OS_TCB LORA_Task_TCB;
CPU_STK LORA_TASK_STK[LORA_STK_SIZE];
void LORA_task(void *parg);

//ÈÎÎñ9 rtcÊ±¼äÏÔÊ¾	»¥³âËø oledÏÔÊ¾	
#define RTC_TASK_PRIO 		9
#define RTC_STK_SIZE		128
OS_TCB RTC_Task_TCB;
CPU_STK RTC_TASK_STK[RTC_STK_SIZE];
void RTC_task(void *parg);

//ÈÎÎñ10 LED0ÈÎÎñ 
#define LED0_TASK_PRIO		10
#define LED0_STK_SIZE 		128
OS_TCB Led0TaskTCB;
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
void led0_task(void *p_arg);

//ÈÎÎñ11 LED1ÈÎÎñ
#define LED1_TASK_PRIO		11	
#define LED1_STK_SIZE 		128
OS_TCB Led1TaskTCB;
CPU_STK LED1_TASK_STK[LED1_STK_SIZE];
void led1_task(void *p_arg);

//ÈÎÎñ12.BEEP
#define BEEP_TASK_PRIO		12
#define BEEP_STK_SIZE 		128
OS_TCB BEEP_Task_TCB;
CPU_STK BEEP_TASK_STK[BEEP_STK_SIZE];
void BEEP_task(void *p_arg);

//ÈÎÎñ13.key
#define KEY_TASK_PRIO		13
#define KEY_STK_SIZE 		128
OS_TCB KEY_Task_TCB;
CPU_STK KEY_TASK_STK[KEY_STK_SIZE];
void KEY_task(void *p_arg);

//ÈÎÎñ14 MPU6050
#define MPU6050_TASK_PRIO		14
#define MPU6050_STK_SIZE		128
OS_TCB	Mpu6050TaskTCB;
__align(8) CPU_STK	MPU6050_TASK_STK[MPU6050_STK_SIZE];
void mpu6050_task(void *p_arg);



//ÈÎÎñ15.ÈÎÎñ×´Ì¬
#define TASK_STA_TASK_PRIO		20
#define TASK_STA_STK_SIZE		128
OS_TCB	TASK_STA_Task_TCB;
CPU_STK	TASK_STA_TASK_STK[TASK_STA_STK_SIZE];
void TASK_STA_task(void *p_arg);


//ÈÎÎñ16 ÈÎÎñÔËÐÐÌáÊ¾
#define FLOAT_TASK_PRIO		21
#define FLOAT_STK_SIZE		128
OS_TCB	FloatTaskTCB;
__align(8) CPU_STK	FLOAT_TASK_STK[FLOAT_STK_SIZE];
void float_task(void *p_arg);


/*****************************ÊÂ¼þ±êÖ¾×éµÄ¶ÔÏó******************************/
OS_FLAG_GRP				g_flag_grp;			

/*******************************ÐÅºÅÁ¿µÄ¶ÔÏó******************************/
OS_SEM					g_sem_led;		
OS_SEM					g_sem_beep;			

/*******************************»¥³âËøµÄ¶ÔÏó******************************/
OS_MUTEX				g_mutex_printf;	
OS_MUTEX				g_mutex_oled;		
OS_MUTEX				g_mutex_lcd;
OS_MUTEX				g_mutex_TDLAS;
OS_MUTEX				g_mutex_LORA;
OS_MUTEX				g_mutex_DHT11;
OS_MUTEX				g_mutex_NODE;

/*****************************ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó*******************************/
OS_Q	 				g_queue_usart1;	
OS_Q	 				g_queue_usart2;				
OS_Q	 				g_queue_usart3;	
OS_Q	 				g_queue_usart4;	
OS_Q	 				g_queue_usart5;	

OS_Q					g_queue_dht11_to_lora;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó
OS_Q					g_queue_dht11_to_txt;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó

OS_Q					g_queue_TDLAS_to_lora;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó
OS_Q					g_queue_TDLAS_to_txt;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó

OS_Q					g_queue_MQ135_to_lora;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó
OS_Q					g_queue_MQ135_to_txt;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó

OS_Q					g_queue_MQ4_to_lora;		//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó
OS_Q					g_queue_MQ4_to_txt;			//ÏûÏ¢¶ÓÁÐµÄ¶ÔÏó

#define CORE_OBJ_NUM	2	//ÄÚºË¶ÔÏó¸öÊý£¬Ò»¹²3¸ö£º2¸öÐÅºÅÁ¿ºÍÒ»¸öÏûÏ¢¶ÓÁÐ						

uint32_t 				g_oled_display_flag=1;
uint32_t 				g_oled_display_time_count=0;


//´æ·ÅDHT11ºÍtdlas´«¸ÐÆ÷Êý¾Ý
char temp_buf[16] = {0};
char humi_buf[16] = {0};
char TDLAS[20] = {0};

char LORA[20] = {0};

char MQ135[20] = {0};
char MQ4[20] = {0};

extern __IO u16 MQ135_ADC_ConvertedValue;

//ÐÞ¸Ä½Úµã½á¹¹Ìå
NODE node_1;
//½Úµã³õÊ¼»¯
void node_init(void)
{
	node_1.device_id = 6;
	node_1.lora_address = LORA_ADDR;
	node_1.lora_channel = LORA_CHN;
	strcpy(node_1.temperature,"25.0");
	strcpy(node_1.humidity,"50.0");
	strcpy(node_1.CH4concentration,"000.0");
}

//»¥³â·ÃÎÊusart1
#define DEBUG_PRINTF_EN	1
void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_PRINTF_EN	
	OS_ERR err;
	
	va_list args;
	va_start(args, format);
	
	OSMutexPend(&g_mutex_printf,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
	vprintf(format, args);
	OSMutexPost(&g_mutex_printf,OS_OPT_POST_NONE,&err);
	
	va_end(args);
#else
	(void)0;
#endif
}
//¹Ø±Õ£¬´ò¿ªÖÐ¶Ï
static void NVIC_Usart2_Disable()
{
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //ÇÀÕ¼ÓÅÏÈ¼¶3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //×ÓÓÅÏÈ¼¶3
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;           //IRQÍ¨µÀÊ¹ÄÜ
    NVIC_Init(&NVIC_InitStructure); 
}
static void NVIC_Usart2_Enable()
{
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //ÇÀÕ¼ÓÅÏÈ¼¶3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //×ÓÓÅÏÈ¼¶3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQÍ¨µÀÊ¹ÄÜ
    NVIC_Init(&NVIC_InitStructure); 
}

//Í¨¹ý´®¿Ú´òÓ¡SD¿¨Ïà¹ØÐÅÏ¢
void show_sdcard_info(void)
{
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
		case SDIO_HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
		case SDIO_MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
	}	
  	printf("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//ÖÆÔìÉÌID
 	printf("Card RCA:%d\r\n",SDCardInfo.RCA);								//¿¨Ïà¶ÔµØÖ·
	printf("Card Capacity:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//ÏÔÊ¾ÈÝÁ¿
 	printf("Card BlockSize:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//ÏÔÊ¾¿é´óÐ¡
}  






/*******************************************
		1.Ó²¼þ³õÊ¼»¯
		2.´®¿Ú³õÊ¼»¯
		3.start_task´´½¨
*********************************************/
int main(void)
{
	OS_ERR err;
	char node_message_1[16] = {0};
	char node_message_2[16] = {0};
	u8 lora_addrh, lora_addrl = 0;
	CPU_SR_ALLOC();
	
	node_init();		//node½á¹¹Ìå³õÊ¼»¯
	
	delay_init();       //ÑÓÊ±³õÊ¼»¯
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //ÖÐ¶Ï·Ö×éÅäÖÃ
	
	uart_init(115200);    	//´®¿Ú²¨ÌØÂÊÉèÖÃ
	usart2_init(115200);    //´®¿Ú²¨ÌØÂÊÉèÖÃ
	NVIC_Usart2_Disable();
	uart4_init(115200);    	//´®¿Ú²¨ÌØÂÊÉèÖÃ

	while(LoRa_Init())		//³õÊ¼»¯ATK-LORA-01Ä£¿é
	{
		printf("Î´¼ì²âµ½    LORA   Ä£¿é!!! \r\n");		
		delay_ms(300);
	}
	LoRa_Set();				//³õÊ¼»¯ATK-LORA-01Ä£¿é
	Lora_mode = 0;      //±ê¼Ç"½ÓÊÕÄ£Ê½"
	set_Already = 1;
	
	
	
	LED_Init();         //LED³õÊ¼»¯
	KEY_Init();			//KEY³õÊ¼»¯ 
	BEEP_Init(); 		//BEEP³õÊ¼»¯	
	LCD_Init();			//³õÊ¼»¯LCD 
	POINT_COLOR=RED;	//ÉèÖÃ×ÖÌåÎªºìÉ«

//	while(DHT11_Init())//ÎÂÊª¶È´«¸ÐÆ÷µÄ³õÊ¼»¯
//	{
//		LCD_ShowString(30,0,200,16,16,"DHT11 Error!");
//		delay_ms(500);					
//		LCD_ShowString(30,0,200,16,16,"Please Check! ");
//		delay_ms(500);	
//		delay_ms(300);
//	}
	
	while(SD_Init())//¼ì²â²»µ½SD¿¨
	{
		LCD_ShowString(30,20,200,16,16,"SD Card Error!");
		delay_ms(500);					
		LCD_ShowString(30,20,200,16,16,"Please Check! ");
		delay_ms(500);
	}
	show_sdcard_info();
	
	
#if 0  
	//ÖÐ¼Ì½Úµã²»²É¼¯´«¸ÐÆ÷Êý¾Ý
	//±£ÁôDHT11½øÐÐµ÷ÊÔ

	mq135_init();		//mq135³õÊ¼»¯

	//Ä£¿éËð»µ
	MPU_Init();			//³õÊ¼»¯MPU6050
	while(mpu_dmp_init())
	{
		LCD_ShowString(30,20,200,16,16,"MPU6050 Error");
		delay_ms(500);
		LCD_Fill(30,130,239,130+16,WHITE);
		delay_ms(500);
	}  
#endif

	//ÏÔÊ¾Éè±¸ID
	lora_addrh = (My_LoRa_CFG.addr >> 8) & 0xff;
    lora_addrl = My_LoRa_CFG.addr & 0xff;
	sprintf(node_message_1,"Node ID:%d",node_1.device_id);
	LCD_ShowString(30,50,200,24,24,(u8 *)node_message_1);
	//sprintf(node_message_2,"CHN:%d  ADDR=%02x%02x",My_LoRa_CFG.chn,lora_addrh, lora_addrl);
	sprintf(node_message_2,"CHN:%d  ADDR=%04x",My_LoRa_CFG.chn,My_LoRa_CFG.addr);
	LCD_ShowString(30,110,250,24,24,(u8 *)node_message_2);
	
	//RTC³õÊ¼»¯
	RTC_Init();

	//ÆäÓàÓ²¼þ³õÊ¼»¯Íê³É£¬
	NVIC_Usart2_Enable();



	OSInit(&err);		//³õÊ¼»¯UCOSIII
	OS_CRITICAL_ENTER();//½øÈëÁÙ½çÇø
	//´´½¨¿ªÊ¼ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//ÈÎÎñ¿ØÖÆ¿é
				 (CPU_CHAR	* )"start task", 		//ÈÎÎñÃû×Ö
                 (OS_TASK_PTR )start_task, 			//ÈÎÎñº¯Êý
                 (void		* )0,					//´«µÝ¸øÈÎÎñº¯ÊýµÄ²ÎÊý
                 (OS_PRIO	  )START_TASK_PRIO,     //ÈÎÎñÓÅÏÈ¼¶
                 (CPU_STK   * )&START_TASK_STK[0],	//ÈÎÎñ¶ÑÕ»»ùµØÖ·
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//ÈÎÎñ¶ÑÕ»Éî¶ÈÏÞÎ»
                 (CPU_STK_SIZE)START_STK_SIZE,		//ÈÎÎñ¶ÑÕ»´óÐ¡
                 (OS_MSG_QTY  )0,					//ÈÎÎñÄÚ²¿ÏûÏ¢¶ÓÁÐÄÜ¹»½ÓÊÕµÄ×î´óÏûÏ¢ÊýÄ¿,Îª0Ê±½ûÖ¹½ÓÊÕÏûÏ¢
                 (OS_TICK	  )0,					//µ±Ê¹ÄÜÊ±¼äÆ¬ÂÖ×ªÊ±µÄÊ±¼äÆ¬³¤¶È£¬Îª0Ê±ÎªÄ¬ÈÏ³¤¶È£¬
                 (void   	* )0,					//ÓÃ»§²¹³äµÄ´æ´¢Çø
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //ÈÎÎñÑ¡Ïî
                 (OS_ERR 	* )&err);				//´æ·Å¸Ãº¯Êý´íÎóÊ±µÄ·µ»ØÖµ
	OS_CRITICAL_EXIT();	//ÍË³öÁÙ½çÇø	 
	OSStart(&err);  //¿ªÆôUCOSIII
	while(1);
}

//¿ªÊ¼ÈÎÎñº¯Êý
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//Í³¼ÆÈÎÎñ                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//Èç¹ûÊ¹ÄÜÁË²âÁ¿ÖÐ¶Ï¹Ø±ÕÊ±¼ä
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //µ±Ê¹ÓÃÊ±¼äÆ¬ÂÖ×ªµÄÊ±ºò
	 //Ê¹ÄÜÊ±¼äÆ¬ÂÖ×ªµ÷¶È¹¦ÄÜ,Ê±¼äÆ¬³¤¶ÈÎª1¸öÏµÍ³Ê±ÖÓ½ÚÅÄ£¬¼È1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//½øÈëÁÙ½çÇø
	
	//´´½¨ÊÂ¼þ±êÖ¾×é£¬ËùÓÐ±êÖ¾Î»³õÖµÎª0
	OSFlagCreate(&g_flag_grp,		"g_flag_grp",0,&err);
	
	//´´½¨ÐÅºÅÁ¿£¬³õÖµÎª0£¬ÓÐÒ»¸ö×ÊÔ´
	OSSemCreate(&g_sem_led,"g_sem_led",0,&err);
	OSSemCreate(&g_sem_beep,"g_sem_beep",0,&err);	
	
	//´´½¨»¥³âÁ¿
	OSMutexCreate(&g_mutex_printf,	"g_mutex_printf",&err);	
	OSMutexCreate(&g_mutex_oled,	"g_mutex_oled",&err);
	OSMutexCreate(&g_mutex_lcd,		"g_mutex_olcd",&err);
	OSMutexCreate(&g_mutex_LORA,	"g_mutex_lora",&err);
	
	
	//´´½¨ÏûÏ¢¶ÓÁÐ£¬ÓÃÓÚusart2·¢ËÍÖÁTDLAS
	OSQCreate(&g_queue_usart1,"g_queue_usart1",16,&err);
	OSQCreate(&g_queue_usart2,"g_queue_usart2",16,&err);
	OSQCreate(&g_queue_usart3,"g_queue_usart3",16,&err);
	OSQCreate(&g_queue_usart4,"g_queue_usart4",16,&err);
	OSQCreate(&g_queue_usart5,"g_queue_usart5",16,&err);
	
	//´´½¨ÏûÏ¢¶ÓÁÐ£¬ÓÃÓÚdht11·¢ËÍÖÁlora
	OSQCreate(&g_queue_dht11_to_lora,"g_queue_dht11_to_lora",16,&err);
	OSQCreate(&g_queue_dht11_to_txt,"g_queue_dht11_to_txt",16,&err);
	
	OSQCreate(&g_queue_TDLAS_to_lora,"g_queue_TDLAS_to_lora",16,&err);
	OSQCreate(&g_queue_TDLAS_to_txt,"g_queue_TDLAS_to_txt",16,&err);
	
	OSQCreate(&g_queue_MQ135_to_lora,"g_queue_MQ135_to_lora",16,&err);
	OSQCreate(&g_queue_MQ135_to_txt,"g_queue_MQ135_to_txt",16,&err);
	
	OSQCreate(&g_queue_MQ4_to_lora,"g_queue_MQ4_to_lora",16,&err);
	OSQCreate(&g_queue_MQ4_to_txt,"g_queue_MQ4_to_txt",16,&err);
	
	dgb_printf_safe("start_task task running\r\n");
	
	
	
	//2.´´½¨DHT11ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&DHT11_Task_TCB,		
				 (CPU_CHAR	* )"DHT11 task", 		
                 (OS_TASK_PTR )DHT11_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )DHT11_TASK_PRIO,     	
                 (CPU_STK   * )&DHT11_TASK_STK[0],	
                 (CPU_STK_SIZE)DHT11_STK_SIZE/10,	
                 (CPU_STK_SIZE)DHT11_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);		
	
	//3.´´½¨TDLASÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&TDLAS_Task_TCB,		
				 (CPU_CHAR	* )"TDLAS task", 		
                 (OS_TASK_PTR )TDLAS_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TDLAS_TASK_PRIO,     	
                 (CPU_STK   * )&TDLAS_TASK_STK[0],	
                 (CPU_STK_SIZE)TDLAS_STK_SIZE/10,	
                 (CPU_STK_SIZE)TDLAS_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	
	//4.´´½¨mq135ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&MQ135_Task_TCB,		
				 (CPU_CHAR	* )"MQ135 task", 		
                 (OS_TASK_PTR )MQ135_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MQ135_TASK_PRIO,     	
                 (CPU_STK   * )&MQ135_TASK_STK[0],	
                 (CPU_STK_SIZE)MQ135_STK_SIZE/10,	
                 (CPU_STK_SIZE)MQ135_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//5.´´½¨mq4ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&MQ4_Task_TCB,		
				 (CPU_CHAR	* )"MQ4 task", 		
                 (OS_TASK_PTR )MQ4_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MQ4_TASK_PRIO,     	
                 (CPU_STK   * )&MQ4_TASK_STK[0],	
                 (CPU_STK_SIZE)MQ4_STK_SIZE/10,	
                 (CPU_STK_SIZE)MQ4_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
		
	//7.´´½¨SAVEÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&SAVE_Task_TCB,		
				 (CPU_CHAR	* )"SAVE task", 		
                 (OS_TASK_PTR )SAVE_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )SAVE_TASK_PRIO,     	
                 (CPU_STK   * )&SAVE_TASK_STK[0],	
                 (CPU_STK_SIZE)SAVE_STK_SIZE/10,	
                 (CPU_STK_SIZE)SAVE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//8.´´½¨LORAÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&LORA_Task_TCB,		
				 (CPU_CHAR	* )"LORA task", 		
                 (OS_TASK_PTR )LORA_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LORA_TASK_PRIO,     	
                 (CPU_STK   * )&LORA_TASK_STK[0],	
                 (CPU_STK_SIZE)LORA_STK_SIZE/10,	
                 (CPU_STK_SIZE)LORA_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	
	//9.´´½¨RTCÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&RTC_Task_TCB,		
				 (CPU_CHAR	* )"RTC task", 		
                 (OS_TASK_PTR )RTC_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )RTC_TASK_PRIO,     	
                 (CPU_STK   * )&RTC_TASK_STK[0],	
                 (CPU_STK_SIZE)RTC_STK_SIZE/10,	
                 (CPU_STK_SIZE)RTC_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	
	//10.´´½¨LED0ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	//11.´´½¨LED1ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&Led1TaskTCB,		
				 (CPU_CHAR	* )"led1 task", 		
                 (OS_TASK_PTR )led1_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED1_TASK_PRIO,     	
                 (CPU_STK   * )&LED1_TASK_STK[0],	
                 (CPU_STK_SIZE)LED1_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED1_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
	
	//12.´´½¨BEEPÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&BEEP_Task_TCB,		
				 (CPU_CHAR	* )"BEEP task", 		
                 (OS_TASK_PTR )BEEP_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )BEEP_TASK_PRIO,     	
                 (CPU_STK   * )&BEEP_TASK_STK[0],	
                 (CPU_STK_SIZE)BEEP_STK_SIZE/10,	
                 (CPU_STK_SIZE)BEEP_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			 
	
	//13.´´½¨KEYÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&KEY_Task_TCB,		
				 (CPU_CHAR	* )"KEY task", 		
                 (OS_TASK_PTR )KEY_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )KEY_TASK_PRIO,     	
                 (CPU_STK   * )&KEY_TASK_STK[0],	
                 (CPU_STK_SIZE)KEY_STK_SIZE/10,	
                 (CPU_STK_SIZE)KEY_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
	
	//14.´´½¨MPU6050ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&Mpu6050TaskTCB,		
				 (CPU_CHAR	* )"MPU6050 task", 		
                 (OS_TASK_PTR )mpu6050_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MPU6050_TASK_PRIO,     	
                 (CPU_STK   * )&MPU6050_TASK_STK[0],	
                 (CPU_STK_SIZE)MPU6050_STK_SIZE/10,	
                 (CPU_STK_SIZE)MPU6050_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//15.´´½¨TASK_STAÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&TASK_STA_Task_TCB,		
				 (CPU_CHAR	* )"TASK_STA task", 		
                 (OS_TASK_PTR )TASK_STA_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK_STA_TASK_PRIO,     	
                 (CPU_STK   * )&TASK_STA_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK_STA_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK_STA_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
				 
	//16.´´½¨¸¡µã²âÊÔÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&FloatTaskTCB,		
				 (CPU_CHAR	* )"float test task", 		
                 (OS_TASK_PTR )float_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )FLOAT_TASK_PRIO,     	
                 (CPU_STK   * )&FLOAT_TASK_STK[0],	
                 (CPU_STK_SIZE)FLOAT_STK_SIZE/10,	
                 (CPU_STK_SIZE)FLOAT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//¹ÒÆð¿ªÊ¼ÈÎÎñ			 
	OS_CRITICAL_EXIT();	//½øÈëÁÙ½çÇø
}


//ÈÎÎñ2 DHT11ÎÂÊª¶È²É¼¯ 
void DHT11_task(void *p_arg)
{
	OS_ERR err;
	
	//DHT11 ÎÂÊª¶È
	uint8_t dht11_data[5] = {0};
	char buf[16] = {0};
	uint8_t temp_buf[16];
	uint8_t humi_buf[16];
	//µ÷ÊÔ
	dgb_printf_safe("DHT11 task running\r\n");
	
	//LCD_ShowString(30,110,200,24,24,(u8 *)"DHT11 OK");
	POINT_COLOR=BLUE;//ÉèÖÃ×ÖÌåÎªÀ¶É« 
	
	while(1)
	{
		//¶ÁÈ¡ÎÂÊª¶ÈÖµ							  
		DHT11_Read_Data(dht11_data);	
		
		//×éºÏ´ÊÌõ
		sprintf((char *)buf,"T:%02d.%dC H:%02d.%d%%",dht11_data[2],dht11_data[3],dht11_data[0],dht11_data[1]);
		sprintf((char *)temp_buf,"Temp:%02d.%dC",dht11_data[2],dht11_data[3]);
		sprintf((char *)humi_buf,"Humi:%02d.%d%%",dht11_data[0],dht11_data[1]);
		
		//¸³Öµ½á¹¹Ìå
		OSMutexPend(&g_mutex_DHT11,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		sprintf((char *)node_1.temperature,"%02d.%d",dht11_data[2],dht11_data[3]);
		sprintf((char *)node_1.humidity,"%02d.%d",dht11_data[0],dht11_data[1]);
		OSMutexPost(&g_mutex_DHT11,OS_OPT_POST_NONE,&err);
		
		//LCDÏÔÊ¾
		OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LCD_ShowString(30,150,100,24,24,temp_buf);
		LCD_ShowString(30,180,100,24,24,humi_buf);
		OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);		
		
		//µ÷ÊÔ×¨ÓÃ
		//·¢ËÍÖÁusart1,½øÐÐµ÷ÊÔ
		//dgb_printf_safe("%s\r\n",buf);
		
#if 0	
		//ÏûÏ¢¶ÓÁÐ
		//·¢ËÍÏûÏ¢¸øLORAÈÎÎñ
		OSQPost((OS_Q*		)&g_queue_dht11_to_lora,		
				(void*		)dht11_data,
				(OS_MSG_SIZE)sizeof(dht11_data),
				(OS_OPT		)OS_OPT_POST_FIFO,
				(OS_ERR*	)&err);
				
		//·¢ËÍ¸øtxtÈÎÎñ
		//OSQPost(&g_queue_dht11_to_txt,
				//(void *)dht11_data,
				//sizeof(dht11_data),
				//OS_OPT_POST_FIFO,
				//&err);
#endif

		//ÑÓÊ±·¢ÉúÈÎÎñµ÷¶È
		delay_ms(1000);
	}
}


//ÈÎÎñ3TDLASÆøÌåÅ¨¶È²É¼¯£¬´®¿Ú½ÓÊÕÅ¨¶ÈÊý¾Ý£¬»¥³âËø oledÏÔÊ¾	Êý¾ÝÍ¨¹ýÏûÏ¢¶ÓÁÐ·¢ËÍÊý¾Ýµ½Ïß³Ì´æ´¢¡¢×ª·¢ÈÎÎñ
void TDLAS_task(void *p_arg)
{
	OS_ERR err;	
	
	int i = 0;
	int flag = 0;
	
	//¶¨µãÄ£¿éÅ¨¶È
	int concen = 0;
	
	//Ò£²âÄ£¿éÅ¨¶È
	int gq_val = 0;
	char gq_str[2] = {0};
	int nd_val = 0;
	char nd_str[5] = {0};
	
	//ÏûÏ¢¶ÓÁÐ½ÓÊÕ½á¹û
	uint8_t *TDLAS_res=NULL;
	OS_MSG_SIZE TDLAS_size;
	
	//×îÖÕÊä³ö
	char Fix_result[20] = {0};
	char telemetry_result_light[20] = "Light: 00";
	char telemetry_result_CH4[20] = "CH4(TDLAS): 00000";
	
	
	dgb_printf_safe("TDLAS task running\r\n");
	
	while(1)
	{	
		
#if 0
		//²âÊÔ´úÂë	
		if(flag){
			concen += 100;
		}
		else{
			concen -= 100;
		}
		
		if (concen > 500 || concen < 0)
		{
			flag = ~flag;
		}	
		//¸³ÖµTDLAS
		OSMutexPend(&g_mutex_TDLAS,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
			sprintf(TDLAS,"%d",concen);
		OSMutexPost(&g_mutex_TDLAS,OS_OPT_POST_NONE,&err);
		
		//OLEDÏÔÊ¾
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			sprintf(result,"TDLAS: %d ppm",x);
			OLED_ShowString(0,4,(uint8_t *)result,20);
		OSMutexPost(&g_mutex_oled,OS_OPT_NONE,&err);
#endif		
		
		//µÈ´ýÏûÏ¢¶ÓÁÐ
		TDLAS_res = OSQPend((OS_Q*			)&g_queue_usart2,
							(OS_TICK		)0,
							(OS_OPT			)OS_OPT_PEND_BLOCKING,
							(OS_MSG_SIZE*	)&TDLAS_size,
							(CPU_TS*		)NULL,
							(OS_ERR*		)&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[TDLAS_task_usart2][OSQPend]Error Code = %d\r\n",err);		
		}
		
		//×ª»»TDLASÊýÖµ£¬²¢ºÏ³ÉÊä³ö×Ö·û´®
		OSMutexPend(&g_mutex_TDLAS,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		if(USART2_RX_STA&0x8000)
		{                                           
			int len=USART2_RX_STA&0x3FFF;//µÃµ½´Ë´Î½ÓÊÕÊý¾ÝµÄ³¤¶È
			for(i = 0;i < len;i++)
			{
				TDLAS[i] = USART2_RX_BUF[i];
			}

			//dgb_printf_safe("TDLAS:%s\r\n",USART2_RX_BUF);
			//dgb_printf_safe("TDLAS:%s\r\n",TDLAS);
			USART2_RX_STA = 0;
		}	
		OSMutexPost(&g_mutex_TDLAS,OS_OPT_POST_NONE,&err);
#if 1		
		//Ò£²âÄ£¿éÐèÒª½ØÈ¡×Ö·û´®
		//¹âÕÕÇ¿¶È²¿·Ö×ª»»
		strncpy(gq_str, TDLAS+8, 2);
		//dgb_printf_safe(" %s \r\n",gq_str);
		gq_val = atoi(gq_str);
		//dgb_printf_safe("Light intensity : %d \r\n",gq_val);
		//sprintf(telemetry_result_light,"Light: %d",gq_val);
		telemetry_result_light[7] = gq_str[0];
		telemetry_result_light[8] = gq_str[1];
		
		//Å¨¶È²¿·Ö×ª»»
		strncpy(nd_str, TDLAS+21, 5);
		//dgb_printf_safe(" %s \r\n",nd_str);
		nd_val = atoi(nd_str);
		//dgb_printf_safe("CH4 concentration : %d \r\n",nd_val);
		//sprintf(telemetry_result_CH4,"CH4: %d",nd_val);
		telemetry_result_CH4[12] = nd_str[0];
		telemetry_result_CH4[13] = nd_str[1];
		telemetry_result_CH4[14] = nd_str[2];
		telemetry_result_CH4[15] = nd_str[3];
		telemetry_result_CH4[16] = nd_str[4];


		//ÔÚLCDÉÏÏÔÊ¾
		OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LCD_ShowString(30,210,200,24,24,(u8 *)telemetry_result_light);
		LCD_ShowString(30,240,200,24,24,(u8 *)telemetry_result_CH4);
		OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
		
#endif

#if 0		
		//¶¨µãÄ£¿éÖ±½Ó×ª»»¼´¿É
		OSMutexPend(&g_mutex_TDLAS,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		concen = atoi(TDLAS);
		sprintf(Fix_result,"TDLAS: %d ppm",concen);	
		OSMutexPost(&g_mutex_TDLAS,OS_OPT_POST_NONE,&err);

		//ÔÚOLEDÉÏÏÔÊ¾
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,4,(uint8_t *)Fix_result,20);
		OSMutexPost(&g_mutex_oled,OS_OPT_NONE,&err);
#endif

	
	
		//ÑÓÊ±·¢ÉúÈÎÎñµ÷¶È
		//delay_ms(1000);
	}
}
	
//ÈÎÎñ4  mq135 °±Æø¡¢Áò»¯Îï¡¢±¿ÏµÕôÆû ÆøÌåÅ¨¶È²É¼¯	
void MQ135_task(void *parg)
{
	OS_ERR err;

	//mq135×ª»»Öµ
	float MQ135_ADC_ConvertedValue_Local;
	float AIR_Quality;
	float AIR_Quality2;
	uint8_t air_1_buf[16] = {0};
	uint8_t air_2_buf[16] = {0};
	
	dgb_printf_safe("MQ135 task running\r\n");
	
	while(1)
	{
		
#if 0 
		//°ÑµçÆ½µÄÄ£ÄâÐÅºÅ×ª»»³ÉÊýÖµ  ¹«Ê½£º×ª»»ºó = ¸¡µãÊý ADCÖµ /4096 * 3.3
		MQ135_ADC_ConvertedValue_Local = (float)MQ135_ADC_ConvertedValue/4096*3.3;
		
		//¿ÕÆøÖÊÁ¿¼ì²âÖµµÄ×ª»»¹«Ê½ ¹«Ê½£ºADCÊýÖµ * 3300 /4095
		AIR_Quality = ((float)MQ135_ADC_ConvertedValue_Local * VREF)/MAX_CONVERTED_VALUE;
		
		//ÔÙÐÞÕýÒ»´Î
		//AIR_Quality2 = AIR_Quality/1000;
		
		//×éºÏ´ÊÌõ
		sprintf((char *)air_1_buf,"AIR_1:%2.3f",AIR_Quality);
		sprintf((char *)air_2_buf,"CH4:%2.3f ppm ",AIR_Quality);


	 	//OLEDÏÔÊ¾Å¨¶È
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,5,air_2_buf,16);
		OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
#endif
		
		//ÑÓÊ±·¢ÉúÈÎÎñµ÷¶È
		delay_ms(1000);
	}
}
	
//ÈÎÎñ5 mq4 ¼×Íé ÆøÌåÅ¨¶È²É¼¯	
void MQ4_task(void *parg)
{
	OS_ERR err;
	
	/*************************
	MQ4 Óë MQ135 Çø±ð²»´óÒò´ËÊ¹ÓÃÏàÍ¬µÄ×ª»»¿Ú£¬
	ÎÞÌìÈ»Æø»·¾³ÏÂ£¬Êµ²âAOUT¶ËµÄµçÑ¹Îª0.0.725V£¬
	µ±¼ì²âµ½ÌìÈ»ÆøÊ±£¬µçÑ¹Éý¸ß0.1V£¬Êµ¼Ê¼ì²âµ½µÄÆøÌåÅ¨¶ÈÔö¼Ó200ppm
	*******************************************/

	//mq135×ª»»Öµ
	float MQ135_ADC_ConvertedValue_Local;
	int CH4_ppm;
	int Voltage;
	
	dgb_printf_safe("MQ4 task running\r\n");
	
	while(1)
	{
		//°ÑµçÆ½µÄÄ£ÄâÐÅºÅ×ª»»³ÉÊýÖµ  ¹«Ê½£º×ª»»ºó = ¸¡µãÊý ADCÖµ /4096 * 3.3
		Voltage = MQ135_ADC_ConvertedValue/4096*3.3;
		
		//¿ÕÆøÖÊÁ¿¼ì²âÖµµÄ×ª»»¹«Ê½ ¹«Ê½£ºADCÊýÖµ * 3300 /4095
		CH4_ppm = (Voltage - 0.5) / 0.1 * 200;
		
		
		//×éºÏ´ÊÌõ
		sprintf(MQ4,"CH4(MQ4): %2.3fppm ",CH4_ppm);

#if 0
	 	//OLEDÏÔÊ¾Å¨¶È
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,4,(uint8_t *)MQ4,16);
		OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
#endif
		
#if 0
		//ÔÚLCDÉÏÏÔÊ¾
		OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LCD_ShowString(30,270,220,24,24,(u8 *)MQ4);
		OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
#endif		
		
		
		//ÑÓÊ±·¢ÉúÈÎÎñµ÷¶È
		delay_ms(2000);
	}
}

//ÈÎÎñ6 ¿´ÃÅ¹· 
void IWG_task(void *parg)
{
	
}
	
//ÈÎÎñ7 µÈ´ý¶à¸öÄÚºË¶ÔÏó ÏûÏ¢¶ÓÁÐ½ÓÊÕÊý¾Ý ÒÔtxt¸ñÊ½´æÈëSD¿¨
void SAVE_task(void *parg)
{
	
}


//ÈÎÎñ8 LORA×ª·¢ µÈ´ý¶à¸öÄÚºË¶ÔÏó ÏûÏ¢¶ÓÁÐ½ÓÊÕÊý¾Ý usart3 ·¢ËÍÖÁÉÏÎ»»ú
void LORA_task(void *p_arg)
{
	OS_ERR err; 
	//ÏûÏ¢¶ÓÁÐ½ÓÊÕ½á¹û
	
	char node_num_str[2] = {0};
	int node_num_int = 0;
	
	uint8_t *LORA_res=NULL;
	OS_MSG_SIZE LORA_size;
	
	dgb_printf_safe("LORA task running\r\n");

	int i=0;
	while(1)
	{
		
		//µÈ´ýÏûÏ¢¶ÓÁÐ
		LORA_res = OSQPend((OS_Q*			)&g_queue_usart3,
							(OS_TICK		)0,
							(OS_OPT			)OS_OPT_PEND_BLOCKING,
							(OS_MSG_SIZE*	)&LORA_size,
							(CPU_TS*		)NULL,
							(OS_ERR*		)&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[LORA_task][OSQPend]Error Code = %d\r\n",err);		
		}
		
		//Êý¾Ý½ÓÊÕ
		OSMutexPend(&g_mutex_LORA,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		
		if(USART3_RX_STA&0x8000)
		{                                           
			int len=USART3_RX_STA&0x3FFF;//µÃµ½´Ë´Î½ÓÊÕÊý¾ÝµÄ³¤¶È
			for(i = 0;i < len;i++)
			{
				LORA[i] = USART3_RX_BUF[i];
			}
			
			dgb_printf_safe("LORA:%s\r\n",LORA);
			
			//Ê¶±ð½Úµã
			node_num_str[0] =  LORA[4];
			node_num_str[1] =  LORA[5];
			node_num_int = atoi(node_num_str);
			
			
			OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			if(node_num_int == 10)
			{
				LCD_ShowString(30,250,300,24,24,LORA);
			}
			if(node_num_int == 15)
			{
				LCD_ShowString(30,280,300,24,24,LORA);
			}
			OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
			
			USART3_RX_STA = 0;
		}	
		
		OSMutexPost(&g_mutex_LORA,OS_OPT_NONE,&err);

		
		
		
	}

}
	
//ÈÎÎñ9 rtcÊ±¼äÏÔÊ¾	»¥³âËø oledÏÔÊ¾	
void RTC_task(void *parg)
{
	OS_ERR err;
	u8 t = 0;	
	char date_time[20] = {0};
	dgb_printf_safe("RTC task running\r\n");
	
	
//OLEDÏÔÊ¾Ê±¼ä		
#if 1		
	
	while(1)
	{
		if(t!=calendar.sec)
		{
			t=calendar.sec;
			OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			
			
			sprintf(date_time,"%d:%d:%d %d:%d",
			calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min);
			OLED_ShowString(0,7,(uint8_t *)date_time,20);
//			switch(calendar.week)
//			{
//				case 0:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Sunday   ");
//					break;
//				case 1:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Monday   ");
//					break;
//				case 2:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Tuesday  ");
//					break;
//				case 3:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Wednesday");
//					break;
//				case 4:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Thursday ");
//					break;
//				case 5:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Friday   ");
//					break;
//				case 6:
//					LCD_ShowString(60,232,200,16,16,(u8 *)"Saturday ");
//					break;  
//			}

			OSMutexPost(&g_mutex_oled,OS_OPT_NONE,&err);	
		}	
		delay_ms(1000);
	}
	
#endif	
	
	
//LCDÏÔÊ¾Ê±¼ä		
#if 0	
	//ÏÔÊ¾Ê±¼ä¿ò¿ò
	POINT_COLOR=BLUE;
	OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	LCD_ShowString(60,200,200,16,16,"    -  -  ");	   
	LCD_ShowString(60,216,200,16,16,"  :  :  ");	
	OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);		
	
	while(1)
	{
		if(t!=calendar.sec)
		{
			t=calendar.sec;
			OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			LCD_ShowNum(60,200,calendar.w_year,4,16);									  
			LCD_ShowNum(100,200,calendar.w_month,2,16);									  
			LCD_ShowNum(124,200,calendar.w_date,2,16);	 
			switch(calendar.week)
			{
				case 0:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Sunday   ");
					break;
				case 1:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Monday   ");
					break;
				case 2:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Tuesday  ");
					break;
				case 3:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Wednesday");
					break;
				case 4:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Thursday ");
					break;
				case 5:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Friday   ");
					break;
				case 6:
					LCD_ShowString(60,232,200,16,16,(u8 *)"Saturday ");
					break;  
			}
			LCD_ShowNum(60,216,calendar.hour,2,16);									  
			LCD_ShowNum(84,216,calendar.min,2,16);									  
			LCD_ShowNum(108,216,calendar.sec,2,16);
			OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
		}	
		delay_ms(1000);
	}
#endif	
	

}
	
//ÈÎÎñ10.ÏµÍ³ÔËÐÐÌáÊ¾ led0ÈÎÎñº¯Êý
void led0_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		LED0=0;
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		LED0=1;
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±500ms
	}
}

//ÈÎÎñ11.¹â±¨¾¯ÈÎÎñ led1ÈÎÎñº¯Êý
// LED1 = 1 Ê±·äÃùÆ÷Æô¶¯
// LED1 = 0 Ê±·äÃùÆ÷¹Ø±Õ
void led1_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	int concen = 0;
	while(1)
	{
		OSMutexPend(&g_mutex_TDLAS,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		concen = atoi(TDLAS);
		OSMutexPost(&g_mutex_TDLAS,OS_OPT_POST_NONE,&err);
		
		//dgb_printf_safe("concen: %d \r\n",concen);
		if(concen > 400)
		{	
			LED1 = 0;	
		}
		else{
			LED1 = 1;	
		}
		
		delay_ms(2000);
	}
}


//ÈÎÎñ12.ÉùÒô±¨¾¯ÈÎÎñ
// BEEP = 1 Ê±·äÃùÆ÷Æô¶¯
// BEEP = 0 Ê±·äÃùÆ÷¹Ø±Õ
void BEEP_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	int concen = 0;
			
	dgb_printf_safe("BEEP task running\r\n");

	while(1)
	{	
		OSMutexPend(&g_mutex_TDLAS,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		concen = atoi(TDLAS);
		OSMutexPost(&g_mutex_TDLAS,OS_OPT_POST_NONE,&err);
		
		//dgb_printf_safe("concen: %d \r\n",concen);
		if(concen > 400)
		{	
			BEEP = 0;	
		}
		else{
			BEEP = 0;	
		}
		

		delay_ms(1000);
	}
}
	

//ÈÎÎñ13.°´¼üÈÎÎñ
void KEY_task(void *p_arg)
{
	OS_ERR err;

	OS_FLAGS flags=0;
	
	//dgb_printf_safe("KEY task running\r\n");
	
	while(1)
	{
		//Ò»Ö±×èÈûµÈ´ýÊÂ¼þ±êÖ¾ÖÃ1£¬µÈ´ý³É¹¦ºó£¬½«¶ÔÓ¦Çå0
		flags = OSFlagPend(&g_flag_grp,FLAG_GRP_KEY0_DOWN
									|FLAG_GRP_KEY1_DOWN
									|FLAG_GRP_KEY2_DOWN
									|FLAG_GRP_WK_UP_DOWN,
									0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
		
		if(err != OS_ERR_NONE)
		{
			//dgb_printf_safe("[task1][OSFlagPend]Error Code = %d\r\n",err);
			continue;
		}
		
		//WK_UPÞôÏÂ
		if(flags & FLAG_GRP_WK_UP_DOWN){	
			//½ûÖ¹EXTI0´¥·¢ÖÐ¶Ï
			NVIC_DisableIRQ(EXTI0_IRQn);
			if(WK_UP == 1){
				delay_ms(20);//Ïû¶¶
				if(WK_UP == 1){
					//dgb_printf_safe("WK_UP happend\r\n");
					//µÈ´ý°´¼üWK_UPÊÍ·Å
					while(WK_UP == 1){
						delay_ms(1);
					}						
					
					BEEP = !BEEP;
				}
			}	
			//ÔÊÐíEXTI0´¥·¢ÖÐ¶Ï
			NVIC_EnableIRQ(EXTI0_IRQn);	
			//Çå¿ÕEXTI0ÖÐ¶Ï±êÖ¾Î»
			EXTI_ClearITPendingBit(EXTI_Line0);			
		}
		
		//KEY2ÞôÏÂ
		if(flags & FLAG_GRP_KEY2_DOWN){	
			if(KEY2 == 0){
				delay_ms(20);//Ïû¶¶
				if(KEY2 == 0){
					//dgb_printf_safe("key2 happend\r\n");
					//µÈ´ý°´¼ü2ÊÍ·Å
					while(KEY2==0){
						delay_ms(1);
					}	
					
					LED0 = !LED0;
				}
			}	
			//Çå¿ÕEXTI2ÖÐ¶Ï±êÖ¾Î»
			EXTI_ClearITPendingBit(EXTI_Line2);			
		}
		
		//KEY1ÞôÏÂ
		if(flags & FLAG_GRP_KEY1_DOWN){	
			if(KEY1 == 0){
				delay_ms(20);//Ïû¶¶
				if(KEY1 == 0){
					//dgb_printf_safe("key1 happend\r\n");
					//µÈ´ý°´¼ü1ÊÍ·Å
					while(KEY1==0){
						delay_ms(1);
					}	
					//·¢ËÍÏûÏ¢
//					OSSemPost(&g_json,OS_OPT_POST_1,&err);//·¢ËÍÐÅºÅÁ¿
				}
			}	
			//Çå¿ÕEXTI3ÖÐ¶Ï±êÖ¾Î»
			EXTI_ClearITPendingBit(EXTI_Line3);			
		}
		
//		//KEY0ÞôÏÂ
//		if(flags & FLAG_GRP_KEY0_DOWN){	
//			if(KEY0 == 0){
//				delay_ms(20);//Ïû¶¶
//				if(KEY0 == 0){
//					dgb_printf_safe("key0 happend\r\n");
//					//µÈ´ý°´¼ü0ÊÍ·Å
//					while(KEY0==0){
//						delay_ms(1);
//					}		
//					
//					LED1 = !LED1;
//					LED0 = !LED0;
//				}
//			}	
//			//Çå¿ÕEXTI4ÖÐ¶Ï±êÖ¾Î»
//			EXTI_ClearITPendingBit(EXTI_Line4);			
//		}
		
		delay_ms(1000);
	}
}
	

//ÈÎÎñ14 MPU6050
//aacx,aacy,aacz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄ¼ÓËÙ¶ÈÖµ
//gyrox,gyroy,gyroz:x,y,zÈý¸ö·½ÏòÉÏÃæµÄÍÓÂÝÒÇÖµ
//roll:ºá¹ö½Ç.µ¥Î»0.01¶È¡£ -18000 -> 18000 ¶ÔÓ¦ -180.00  ->  180.00¶È
//pitch:¸©Ñö½Ç.µ¥Î» 0.01¶È¡£-9000 - 9000 ¶ÔÓ¦ -90.00 -> 90.00 ¶È
//yaw:º½Ïò½Ç.µ¥Î»Îª0.1¶È 0 -> 3600  ¶ÔÓ¦ 0 -> 360.0¶È
void mpu6050_task(void *p_arg)
{
	OS_ERR err; 
	CPU_SR_ALLOC();
	int res = 0;
	u8 t=0;			//Ä¬ÈÏ¿ªÆôÉÏ±¨
	u8 key;
	float pitch,roll,yaw; 		//Å·À­½Ç
	short aacx,aacy,aacz;		//¼ÓËÙ¶È´«¸ÐÆ÷Ô­Ê¼Êý¾Ý
	short gyrox,gyroy,gyroz;	//ÍÓÂÝÒÇÔ­Ê¼Êý¾Ý
	short temp;					//ÎÂ¶È	
	
	dgb_printf_safe("MPU6050 task running\r\n");
//	LCD_ShowString(30,300,200,16,16," Temp:    . C");	
// 	LCD_ShowString(30,320,200,16,16,"Pitch:    . C");	
// 	LCD_ShowString(30,340,200,16,16," Roll:    . C");	 
// 	LCD_ShowString(30,360,200,16,16," Yaw :    . C");	
	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//µÃµ½ÎÂ¶ÈÖµ
			res = MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//µÃµ½¼ÓËÙ¶È´«¸ÐÆ÷Êý¾Ý
			printf("res_1: %d \r\n",res);
			res =MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//µÃµ½ÍÓÂÝÒÇÊý¾Ý
			printf("res_2: %d \r\n",res);
			if(temp<0)
			{
				LCD_ShowChar(30+48,300,'-',16,0);		//ÏÔÊ¾¸ººÅ
				temp=-temp;		//×ªÎªÕýÊý
			}
			else LCD_ShowChar(30+48,300,' ',16,0);		//È¥µô¸ººÅ 
			LCD_ShowNum(30+48+8,300,temp/100,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
			LCD_ShowNum(30+48+40,300,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö 
			
			temp=pitch*10;
			if(temp<0)
			{
				LCD_ShowChar(30+48,320,'-',16,0);		//ÏÔÊ¾¸ººÅ
				temp=-temp;		//×ªÎªÕýÊý
			}
			else LCD_ShowChar(30+48,320,' ',16,0);		//È¥µô¸ººÅ 
			LCD_ShowNum(30+48+8,320,temp/10,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
			LCD_ShowNum(30+48+40,320,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö 
			
			temp=roll*10;
			if(temp<0)
			{
				LCD_ShowChar(30+48,340,'-',16,0);		//ÏÔÊ¾¸ººÅ
				temp=-temp;		//×ªÎªÕýÊý
			}
			else LCD_ShowChar(30+48,340,' ',16,0);		//È¥µô¸ººÅ 
			LCD_ShowNum(30+48+8,340,temp/10,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
			LCD_ShowNum(30+48+40,340,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö 
			
			temp=yaw*10;
			if(temp<0)
			{
				LCD_ShowChar(30+48,360,'-',16,0);		//ÏÔÊ¾¸ººÅ
				temp=-temp;		//×ªÎªÕýÊý
			}
			else LCD_ShowChar(30+48,360,' ',16,0);		//È¥µô¸ººÅ 
			LCD_ShowNum(30+48+8,360,temp/10,3,16);		//ÏÔÊ¾ÕûÊý²¿·Ö	    
			LCD_ShowNum(30+48+40,360,temp%10,1,16);		//ÏÔÊ¾Ð¡Êý²¿·Ö  
			
		}
		delay_ms(2000);			//ÑÓÊ±500ms
		
	}
}



//ÈÎÎñ15.ÏµÍ³ÄÚ´æÕ¼ÓÃ¼àÊÓ
void TASK_STA_task(void *p_arg)
{
	OS_ERR err;  
	
	CPU_STK_SIZE free,used; 
	
	//dgb_printf_safe("TASK_STA task running\r\n");
	
	delay_ms(3000);
	
	while(1)
	{

#if 0
		OSTaskStkChk (&DHT11_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_ir    stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free)); 
	
		OSTaskStkChk (&LORA_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_key   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));
	
		OSTaskStkChk (&RTC_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_usart1   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));	
		
		OSTaskStkChk (&SAVE_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_mpu6050  stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));			

		OSTaskStkChk (&LED0_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_rtc   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));  	  			  

		OSTaskStkChk (&LED1_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_dht11 stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));  

		OSTaskStkChk (&BEEP_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("BEEP_task   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));

		OSTaskStkChk (&TASK_STA_Task_TCB,&free,&used,&err); 
		dgb_printf_safe("app_task_sta   stk[used/free:%d/%d usage:%d%%]\r\n",used,free,(used*100)/(used+free));
#endif		
		delay_ms(10000);
	}
}

//ÈÎÎñ16.¸¡µã²âÊÔÈÎÎñ
void float_task(void *p_arg)
{
	OS_ERR err; 
	CPU_SR_ALLOC();
	static float float_num=0.01;
	char node_message[16] = {0};
	while(1)
	{
		float_num+=0.01f;
		
		dgb_printf_safe("float_numµÄÖµÎª: %.4f\r\n",float_num);

		LoRa_SendData();
				
		delay_ms(6000);			//ÑÓÊ±500ms
		
	}
}
