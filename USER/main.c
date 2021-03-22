//����ͷ�ļ�
#include "includes.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
//����
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
//����
#include "lora_app.h"
#include "led.h"
#include "beep.h"
#include "dht11.h"
#include "mq135.h"
#include "oled.h"
#include "key.h"
#include "rtc.h"
#include "lcd.h"
//MPU6050
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
//SD�����ļ�ϵͳ
#include "sdio_sdcard.h"   
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h" 
#include "malloc.h"
#include "text.h"	
#include "malloc.h"
//JSON
#include "cjson.h"
#include "cJson_test.h"



/****************************************************************
*��    ��:����վ������й©���ϵͳ
*��    ��:��ε
*��������:2020/10/14 ����ϵͳ���
*��ǰ����:2020/11/16 ���V1.0
*��ǰ����:2020/12/24 ���V2.0
*��ǰ����:2021/01/08 ���V3.0
*��ǰ����:2021/03/09 ���V4.0
*��ǰ����:2021/03/14 ���V5.0
*��ǰ����:2021/03/14 ��ɶ�����ڵ����
*��ǰ����:2021/03/15 ���ң����ڵ����
*��ǰ����:2021/03/16 ����м̽ڵ����



*����
	1.��ʼ���� �����ź��� ��Ϣ���� ������ �¼���־�� ����
	2.DHT11��ʪ�Ȳɼ�,�����߲ɼ���  	������ lcd		����ͨ����Ϣ���з������ݵ��̴߳洢��ת������
	(�ýڵ㲻ʹ��)3.TDLAS����Ũ�Ȳɼ������ڽ���Ũ�����ݣ������� lcd��ʾ	����ͨ����Ϣ���з������ݵ��̴߳洢��ת������        
	(�ýڵ㲻ʹ��)4.MQ135 Ũ�Ȳɼ�
	(�ýڵ㲻ʹ��)5.MQ4 Ũ�Ȳɼ�
	(�ýڵ㲻ʹ��)6.���Ź�
	7.���ش洢���� �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
	8.LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 ��������λ��
	9.RTCʱ����ʾ ������ oled��ʾ	
	10.LED0 �ź����������� ����
	11.LED1 ϵͳ������ʾ
	12.BEEP �ź����������� ����
	13.KEY 
	(�ýڵ㲻ʹ��)14.mpu6050 ���ᴫ���� ��ʾ��ǰ��̨�Ƕ����� 
	15.����״̬
	16.ϵͳ���м��
	17.�ȴ���λ�������ϴ������ѯ�����ڵ��ϴ�����λ��
	
*˵  ��:		
	��ǰ���뾡����ʵ����ģ�黯��̣�һ���������һ��Ӳ������򵥵�
	led�����������ɵ����������
*****************************************************************/

//V1.0 ���������ں˴��� ���������ݲɼ� json���ݷ�װ
//V2.0 ���lcd��ʾ ������Ӧ  LORA������dht11��TDLAS����Ϣ���д���
//V3.0 �����λ������λ����������ѯ
//V4.0 ���lora��Ϣ���� 
//V5.0 ϵͳ����


/*****************************���������ջ*************************************/
//UCOSIII���������ȼ��û�������ʹ�ã�ALIENTEK
//����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
//���ȼ�0���жϷ������������� OS_IntQTask()
//���ȼ�1��ʱ�ӽ������� OS_TickTask()
//���ȼ�2����ʱ���� OS_TmrTask()
//���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
//���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()


//����1 ��ʼ����
#define START_TASK_PRIO		3	
#define START_STK_SIZE 		512
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

//����2 DHT11��ʪ�Ȳɼ� 
#define DHT11_TASK_PRIO 	4
#define DHT11_STK_SIZE		128
OS_TCB DHT11_Task_TCB;
CPU_STK DHT11_TASK_STK[DHT11_STK_SIZE];
void DHT11_task(void *parg);

//����3 TDLAS ����Ũ�Ȳɼ�	usart2
#define TDLAS_TASK_PRIO 	5
#define TDLAS_STK_SIZE		512
OS_TCB TDLAS_Task_TCB;
CPU_STK TDLAS_TASK_STK[TDLAS_STK_SIZE];
void TDLAS_task(void *parg);

//����4  mq135 ����Ũ�Ȳɼ�	
#define MQ135_TASK_PRIO 	5
#define MQ135_STK_SIZE		128
OS_TCB MQ135_Task_TCB;
CPU_STK MQ135_TASK_STK[MQ135_STK_SIZE];
void MQ135_task(void *parg);

//����5 mq4 ����Ũ�Ȳɼ�	
#define MQ4_TASK_PRIO 	5
#define MQ4_STK_SIZE		128
OS_TCB MQ4_Task_TCB;
CPU_STK MQ4_TASK_STK[MQ4_STK_SIZE];
void MQ4_task(void *parg);

//����6 ���Ź� 
#define IWG_TASK_PRIO 		6
#define IWG_STK_SIZE		128
OS_TCB IWG_Task_TCB;
CPU_STK IWG_TASK_STK[IWG_STK_SIZE];
void IWG_task(void *parg);

//����7 �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
#define SAVE_TASK_PRIO 		9
#define SAVE_STK_SIZE		512
OS_TCB SAVE_Task_TCB;
CPU_STK SAVE_TASK_STK[SAVE_STK_SIZE];
void SAVE_task(void *parg);

//����8 LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 ��������λ��
#define LORA_TASK_PRIO 		8
#define LORA_STK_SIZE		512
OS_TCB LORA_Task_TCB;
CPU_STK LORA_TASK_STK[LORA_STK_SIZE];
void LORA_task(void *parg);

//����9 rtcʱ����ʾ	������ oled��ʾ	
#define RTC_TASK_PRIO 		9
#define RTC_STK_SIZE		128
OS_TCB RTC_Task_TCB;
CPU_STK RTC_TASK_STK[RTC_STK_SIZE];
void RTC_task(void *parg);

//����10 LED0���� 
#define LED0_TASK_PRIO		10
#define LED0_STK_SIZE 		128
OS_TCB Led0TaskTCB;
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
void led0_task(void *p_arg);

//����11 LED1����
#define LED1_TASK_PRIO		11	
#define LED1_STK_SIZE 		128
OS_TCB Led1TaskTCB;
CPU_STK LED1_TASK_STK[LED1_STK_SIZE];
void led1_task(void *p_arg);

//����12.BEEP
#define BEEP_TASK_PRIO		12
#define BEEP_STK_SIZE 		128
OS_TCB BEEP_Task_TCB;
CPU_STK BEEP_TASK_STK[BEEP_STK_SIZE];
void BEEP_task(void *p_arg);

//����13.key
#define KEY_TASK_PRIO		13
#define KEY_STK_SIZE 		128
OS_TCB KEY_Task_TCB;
CPU_STK KEY_TASK_STK[KEY_STK_SIZE];
void KEY_task(void *p_arg);

//����14 MPU6050
#define MPU6050_TASK_PRIO		14
#define MPU6050_STK_SIZE		128
OS_TCB	Mpu6050TaskTCB;
__align(8) CPU_STK	MPU6050_TASK_STK[MPU6050_STK_SIZE];
void mpu6050_task(void *p_arg);


//����15.����״̬
#define TASK_STA_TASK_PRIO		20
#define TASK_STA_STK_SIZE		128
OS_TCB	TASK_STA_Task_TCB;
CPU_STK	TASK_STA_TASK_STK[TASK_STA_STK_SIZE];
void TASK_STA_task(void *p_arg);


//����16 ����������ʾ
#define FLOAT_TASK_PRIO		21
#define FLOAT_STK_SIZE		128
OS_TCB	FloatTaskTCB;
__align(8) CPU_STK	FLOAT_TASK_STK[FLOAT_STK_SIZE];
void float_task(void *p_arg);

//����17 �ȴ���λ�������ϴ����
//��ѯ�����ڵ��ϴ�����λ��
#define UPLOAD_TASK_PRIO		21
#define UPLOAD_STK_SIZE		128
OS_TCB	UPLOADTaskTCB;
__align(8) CPU_STK	UPLOAD_TASK_STK[UPLOAD_STK_SIZE];
void upload_task(void *p_arg);


/*****************************�¼���־��Ķ���******************************/
OS_FLAG_GRP				g_flag_grp;			

/*******************************�ź����Ķ���******************************/
OS_SEM					g_sem_led;		
OS_SEM					g_sem_beep;			

/*******************************�������Ķ���******************************/
OS_MUTEX				g_mutex_printf;	
OS_MUTEX				g_mutex_oled;		
OS_MUTEX				g_mutex_lcd;

OS_MUTEX				g_mutex_NODE;

/*****************************��Ϣ���еĶ���*******************************/
OS_Q	 				g_queue_usart1;	
OS_Q	 				g_queue_usart2;				
OS_Q	 				g_queue_usart3;	
OS_Q	 				g_queue_usart4;	
OS_Q	 				g_queue_usart5;	

OS_Q					g_queue_dht11_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_dht11_to_txt;		//��Ϣ���еĶ���

OS_Q					g_queue_TDLAS_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_TDLAS_to_txt;		//��Ϣ���еĶ���

OS_Q					g_queue_MQ135_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_MQ135_to_txt;		//��Ϣ���еĶ���

OS_Q					g_queue_MQ4_to_lora;		//��Ϣ���еĶ���
OS_Q					g_queue_MQ4_to_txt;			//��Ϣ���еĶ���

#define CORE_OBJ_NUM	2	//�ں˶��������һ��3����2���ź�����һ����Ϣ����						

uint32_t 				g_oled_display_flag=1;
uint32_t 				g_oled_display_time_count=0;


extern __IO int MQ135_ADC_ConvertedValue;

//�޸Ľڵ�ṹ��
//�ڵ�ṹ��
typedef struct 
{
	char device_id[4];
	char lora_address[4];
	char lora_channel[4];
	char temperature[8];
	char humidity[8];
	char CH4concentration[8];
	char Pitch[8];
	char Roll[8];
	char Yaw[8];
	char light[4];
	char warning[4];
	char over[4];
} NODE;

//���ڵ�ȫ�ֱ���
NODE node_temp;
NODE node_1;
NODE node_2;
NODE node_3;
NODE node_4;
NODE node_5;
NODE node_6;


//�ڵ��ʼ��
void node_init(void)
{
	strcpy(node_6.device_id,"6");
	sprintf(node_6.lora_address,"%d", My_LoRa_CFG.addr);
	sprintf(node_6.lora_channel,"%d", My_LoRa_CFG.chn);
	strcpy(node_6.temperature,"25.0");
	strcpy(node_6.temperature,"25.0");
	strcpy(node_6.CH4concentration,"000.00");
	strcpy(node_6.over,"\r\n");
}


//�������usart1
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

//�رգ���USART2�ж�
static void NVIC_Usart2_Disable()
{
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;           //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); 
}
static void NVIC_Usart2_Enable()
{
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); 
}


//ͨ�����ڴ�ӡSD�������Ϣ
void show_sdcard_info(void)
{
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
		case SDIO_HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
		case SDIO_MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
	}	
  	printf("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//������ID
 	printf("Card RCA:%d\r\n",SDCardInfo.RCA);								//����Ե�ַ
	printf("Card Capacity:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//��ʾ����
 	printf("Card BlockSize:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//��ʾ���С
}  


//json��װ����
void json_test(NODE* node)
{
	/* Build the JSON array [[1, 2], {"cool": true}] */
	/* print the version */
	cJSON *root = NULL;
	cJSON *fmt = NULL;
	cJSON *Node = NULL;

	//printf("Version: %s\n", cJSON_Version());

	/* Now some samplecode for building objects concisely: */	
	root = cJSON_CreateObject();
	cJSON_AddItemToObject(root,"node 1",Node = cJSON_CreateObject());
	cJSON_AddStringToObject(Node,"DeviceID",node->device_id);
	cJSON_AddStringToObject(Node,"LORA_ADD",node->lora_address);
	cJSON_AddStringToObject(Node,"LORA_CHN",node->lora_channel);
	cJSON_AddStringToObject(Node, "Temp", node->temperature);
	cJSON_AddStringToObject(Node, "humi", node->humidity);
	cJSON_AddStringToObject(Node, "CH4_concentration", node->CH4concentration);

	/* Print to text */
	if (print_preallocated(root) != 0) {
		cJSON_Delete(root);
		return ;
	}
	cJSON_Delete(root);		
}

void json_test_1(NODE* node)
{
	/* Build the JSON array [[1, 2], {"cool": true}] */
	/* print the version */
	cJSON *root = NULL;
	cJSON *fmt = NULL;
	cJSON *Node = NULL;

	//printf("Version: %s\n", cJSON_Version());

	/* Now some samplecode for building objects concisely: */	
	root = cJSON_CreateObject();
	cJSON_AddItemToObject(root,"node 1",Node = cJSON_CreateObject());
	cJSON_AddStringToObject(Node,"DeviceID",node->device_id);
	cJSON_AddStringToObject(Node,"LORA_ADD",node->lora_address);
	cJSON_AddStringToObject(Node,"LORA_CHN",node->lora_channel);
	cJSON_AddStringToObject(Node, "Temp", node->temperature);
	cJSON_AddStringToObject(Node, "humi", node->humidity);
	cJSON_AddStringToObject(Node, "light", node->light);
	cJSON_AddStringToObject(Node, "CH4_concentration", node->CH4concentration);
	cJSON_AddStringToObject(Node, "Pitch", node->Pitch);
	cJSON_AddStringToObject(Node, "Roll", node->Roll);
	cJSON_AddStringToObject(Node, "Yaw", node->Yaw);
	/* Print to text */
	if (print_preallocated(root) != 0) {
		cJSON_Delete(root);
		return ;
	}
	cJSON_Delete(root);		
}

/*******************************************
		1.Ӳ����ʼ��
		2.���ڳ�ʼ��
		3.start_task����
*********************************************/
int main(void)
{
	OS_ERR err;
	char node_message_1[16] = {0};
	char node_message_2[16] = {0};
	u8 lora_addrh, lora_addrl = 0;
	CPU_SR_ALLOC();
	
	node_init();		//node�ṹ���ʼ��
	
	delay_init();       //��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�жϷ�������
	
	uart_init(115200);    	//���ڲ���������
	usart2_init(115200);    //���ڲ���������
	uart4_init(115200);    	//���ڲ���������

	while(LoRa_Init())		//��ʼ��ATK-LORA-01ģ��
	{
		printf("δ��⵽    LORA   ģ��!!! \r\n");		
		delay_ms(300);
	}
	LoRa_Set();				//��ʼ��ATK-LORA-01ģ��
	Lora_mode = 0;      //���"����ģʽ"
	set_Already = 1;
	
	LED_Init();         //LED��ʼ��
	KEY_Init();			//KEY��ʼ�� 
	BEEP_Init(); 		//BEEP��ʼ��	
	LCD_Init();			//��ʼ��LCD 
	POINT_COLOR=RED;	//��������Ϊ��ɫ

	while(DHT11_Init())//��ʪ�ȴ������ĳ�ʼ��
	{
		LCD_ShowString(30,0,200,16,16,"DHT11 Error!");
		delay_ms(500);					
		LCD_ShowString(30,0,200,16,16,"Please Check! ");
		delay_ms(500);	
		delay_ms(300);
	}
	

	
	W25QXX_Init();		//��ʼ��W25Q128		
	my_mem_init(SRAMIN);//��ʼ���ڲ��ڴ��
	while(SD_Init())	//���SD��
	{
		LCD_ShowString(30,10,200,16,16,(u8 *)"SD Card Error!");
		delay_ms(500);					
		LCD_ShowString(30,10,200,16,16,(u8 *)"Please Check! ");
		delay_ms(500);
	}
	exfuns_init();		//Ϊfatfs��ر��������ڴ�
	f_mount(fs[0],"0:",1);			//����SD��
	f_mount(fs[1],"1:",1); 			//����FLASH.	
	

	show_sdcard_info();	
	
#if 0  
	//�м̽ڵ㲻�ɼ�����������
	//����DHT11���е���

	mq135_init();		//mq135��ʼ��

	//ģ����
	MPU_Init();			//��ʼ��MPU6050
	while(mpu_dmp_init())
	{
		LCD_ShowString(30,20,200,16,16,"MPU6050 Error");
		delay_ms(500);
		LCD_Fill(30,130,239,130+16,WHITE);
		delay_ms(500);
	}  
#endif

	//��ʾ�豸ID
	lora_addrh = (My_LoRa_CFG.addr >> 8) & 0xff;
    lora_addrl = My_LoRa_CFG.addr & 0xff;
	sprintf(node_message_1,"Node ID:6");
	LCD_ShowString(30,80,200,24,24,(u8 *)node_message_1);
	//sprintf(node_message_2,"CHN:%d  ADDR=%02x%02x",My_LoRa_CFG.chn,lora_addrh, lora_addrl);
	sprintf(node_message_2,"CHN:%d  ADDR=%04x",My_LoRa_CFG.chn,My_LoRa_CFG.addr);
	LCD_ShowString(30,110,250,24,24,(u8 *)node_message_2);
	
	//RTC��ʼ��
	RTC_Init();

	OSInit(&err);		//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();//�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII
	while(1);
}

//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	
	//�����¼���־�飬���б�־λ��ֵΪ0
	OSFlagCreate(&g_flag_grp,		"g_flag_grp",0,&err);
	
	//�����ź�������ֵΪ0����һ����Դ
	OSSemCreate(&g_sem_led,"g_sem_led",0,&err);
	OSSemCreate(&g_sem_beep,"g_sem_beep",0,&err);	
	
	//����������
	OSMutexCreate(&g_mutex_printf,	"g_mutex_printf",&err);	
	OSMutexCreate(&g_mutex_oled,	"g_mutex_oled",&err);
	OSMutexCreate(&g_mutex_lcd,		"g_mutex_olcd",&err);
	
	OSMutexCreate(&g_mutex_NODE,		"g_mutex_NODE",&err);
	
	
	//������Ϣ���У�����usart2������TDLAS
	OSQCreate(&g_queue_usart1,"g_queue_usart1",16,&err);
	OSQCreate(&g_queue_usart2,"g_queue_usart2",16,&err);
	OSQCreate(&g_queue_usart3,"g_queue_usart3",16,&err);
	OSQCreate(&g_queue_usart4,"g_queue_usart4",16,&err);
	OSQCreate(&g_queue_usart5,"g_queue_usart5",16,&err);
	
	//������Ϣ���У�����dht11������lora
	OSQCreate(&g_queue_dht11_to_lora,"g_queue_dht11_to_lora",16,&err);
	OSQCreate(&g_queue_dht11_to_txt,"g_queue_dht11_to_txt",16,&err);
	
	OSQCreate(&g_queue_TDLAS_to_lora,"g_queue_TDLAS_to_lora",16,&err);
	OSQCreate(&g_queue_TDLAS_to_txt,"g_queue_TDLAS_to_txt",16,&err);
	
	OSQCreate(&g_queue_MQ135_to_lora,"g_queue_MQ135_to_lora",16,&err);
	OSQCreate(&g_queue_MQ135_to_txt,"g_queue_MQ135_to_txt",16,&err);
	
	OSQCreate(&g_queue_MQ4_to_lora,"g_queue_MQ4_to_lora",16,&err);
	OSQCreate(&g_queue_MQ4_to_txt,"g_queue_MQ4_to_txt",16,&err);
	
	dgb_printf_safe("start_task task running\r\n");
	
	//2.����DHT11����
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
	
	//3.����TDLAS����
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
	
	//4.����mq135����
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
				 
	//5.����mq4����
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
		
	//7.����SAVE����
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
				 
	//8.����LORA����
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
	
	//9.����RTC����
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
	
	//10.����LED0����
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
				 
	//11.����LED1����
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
	
	//12.����BEEP����
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
	
	//13.����KEY����
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
	
	//14.����MPU6050����
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
				 
	//15.����TASK_STA����
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
				 
	//16.���������������
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
	
	//17.�����ϴ�����
	OSTaskCreate((OS_TCB 	* )&UPLOADTaskTCB,		
				 (CPU_CHAR	* )"upload_task", 		
                 (OS_TASK_PTR )upload_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )UPLOAD_TASK_PRIO,     	
                 (CPU_STK   * )&UPLOAD_TASK_STK[0],	
                 (CPU_STK_SIZE)UPLOAD_STK_SIZE/10,	
                 (CPU_STK_SIZE)UPLOAD_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);					 
				 

	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�����ٽ���
}


//����2 DHT11��ʪ�Ȳɼ� 
void DHT11_task(void *p_arg)
{
	OS_ERR err;
	
	//DHT11 ��ʪ��
	uint8_t dht11_data[5] = {0};
	char buf[16] = {0};
	uint8_t temp_buf[16];
	uint8_t humi_buf[16];
	//����
	dgb_printf_safe("DHT11 task running\r\n");
	
	//LCD_ShowString(30,110,200,24,24,(u8 *)"DHT11 OK");
	POINT_COLOR=BLUE;//��������Ϊ��ɫ 
	
	while(1)
	{
		//��ȡ��ʪ��ֵ							  
		DHT11_Read_Data(dht11_data);	
		
		//��ϴ���
		sprintf((char *)buf,"T:%02d.%dC H:%02d.%d%%",dht11_data[2],dht11_data[3],dht11_data[0],dht11_data[1]);
		sprintf((char *)temp_buf,"Temp:%02d.%dC",dht11_data[2],dht11_data[3]);
		sprintf((char *)humi_buf,"Humi:%02d.%d%%",dht11_data[0],dht11_data[1]);
		
		//��ֵ�ṹ��
		OSMutexPend(&g_mutex_NODE,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		sprintf((char *)node_1.temperature,"%02d.%d",dht11_data[2],dht11_data[3]);
		sprintf((char *)node_1.humidity,"%02d.%d",dht11_data[0],dht11_data[1]);
		OSMutexPost(&g_mutex_NODE,OS_OPT_POST_NONE,&err);
		
		//LCD��ʾ
		OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LCD_ShowString(30,150,100,24,24,temp_buf);
		LCD_ShowString(30,180,100,24,24,humi_buf);
		OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);		
		
		//����ר��
		//������usart1,���е���
		//dgb_printf_safe("%s\r\n",buf);
		
#if 0	
		//��Ϣ����
		//������Ϣ��LORA����
		OSQPost((OS_Q*		)&g_queue_dht11_to_lora,		
				(void*		)dht11_data,
				(OS_MSG_SIZE)sizeof(dht11_data),
				(OS_OPT		)OS_OPT_POST_FIFO,
				(OS_ERR*	)&err);
				
		//���͸�txt����
		//OSQPost(&g_queue_dht11_to_txt,
				//(void *)dht11_data,
				//sizeof(dht11_data),
				//OS_OPT_POST_FIFO,
				//&err);
#endif

		//��ʱ�����������
		delay_ms(1000);
	}
}


//����3TDLAS����Ũ�Ȳɼ������ڽ���Ũ�����ݣ������� oled��ʾ	����ͨ����Ϣ���з������ݵ��̴߳洢��ת������
void TDLAS_task(void *p_arg)
{
	OS_ERR err;	
	
	int i = 0;
	int flag = 0;
	
	//��Ϣ���н��ս��
	uint8_t *TDLAS_res=NULL;
	OS_MSG_SIZE TDLAS_size;
	char TDLAS[20] = {0};
	
	//����ģ��Ũ��
	int concen = 0;
	
	//ң��ģ��Ũ��
	int gq_val = 0;
	char gq_str[2] = {0};
	int nd_val = 0;
	char nd_str[5] = {0};
	//�������
	char Fix_result[20] = {0};
	char telemetry_result_light[20] = "Light: 00";
	char telemetry_result_CH4[20] = "CH4(TDLAS): 00000";
	
	dgb_printf_safe("TDLAS task running\r\n");
	
	while(1)
	{	
		//�ȴ���Ϣ����
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
		
		//ת��TDLAS��ֵ�����ϳ�����ַ���
		if(USART2_RX_STA&0x8000)
		{                                           
			int len=USART2_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���
			for(i = 0;i < len;i++)
			{
				TDLAS[i] = USART2_RX_BUF[i];
			}

			//dgb_printf_safe("TDLAS:%s\r\n",USART2_RX_BUF);
			//dgb_printf_safe("TDLAS:%s\r\n",TDLAS);
			USART2_RX_STA = 0;
		}	
#if 1		
		//ң��ģ����Ҫ��ȡ�ַ���
		//����ǿ�Ȳ���ת��
		strncpy(gq_str, TDLAS+8, 2);
		//dgb_printf_safe(" %s \r\n",gq_str);
		gq_val = atoi(gq_str);
		//dgb_printf_safe("Light intensity : %d \r\n",gq_val);
		//sprintf(telemetry_result_light,"Light: %d",gq_val);
		telemetry_result_light[7] = gq_str[0];
		telemetry_result_light[8] = gq_str[1];
		
		//Ũ�Ȳ���ת��
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


		//��LCD����ʾ
		OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		LCD_ShowString(30,210,200,24,24,(u8 *)telemetry_result_light);
		LCD_ShowString(30,240,200,24,24,(u8 *)telemetry_result_CH4);
		OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
		
#endif

#if 0		
		OSMutexPend(&g_mutex_NODE,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		strcpy(node_1.CH4concentration,TDLAS);
		OSMutexPost(&g_mutex_NODE,OS_OPT_POST_NONE,&err);
		
		//����ģ��ֱ��ת������
		concen = atoi(TDLAS);
		sprintf(Fix_result,"TDLAS: %d ppm",concen);	

		//��OLED����ʾ
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,4,(uint8_t *)Fix_result,20);
		OSMutexPost(&g_mutex_oled,OS_OPT_NONE,&err);
#endif

		//��ʱ�����������
		//delay_ms(1000);
	}
}
	
//����4  mq135 �����������ϵ���� ����Ũ�Ȳɼ�	
void MQ135_task(void *parg)
{
	OS_ERR err;

	//mq135ת��ֵ
	float MQ135_ADC_ConvertedValue_Local;
	float AIR_Quality;
	float AIR_Quality2;
	uint8_t air_1_buf[16] = {0};
	uint8_t air_2_buf[16] = {0};
	
	dgb_printf_safe("MQ135 task running\r\n");
	
	while(1)
	{
		
#if 0 
		//�ѵ�ƽ��ģ���ź�ת������ֵ  ��ʽ��ת���� = ������ ADCֵ /4096 * 3.3
		MQ135_ADC_ConvertedValue_Local = (float)MQ135_ADC_ConvertedValue/4096*3.3;
		
		//�����������ֵ��ת����ʽ ��ʽ��ADC��ֵ * 3300 /4095
		AIR_Quality = ((float)MQ135_ADC_ConvertedValue_Local * VREF)/MAX_CONVERTED_VALUE;
		
		//������һ��
		//AIR_Quality2 = AIR_Quality/1000;
		
		//��ϴ���
		sprintf((char *)air_1_buf,"AIR_1:%2.3f",AIR_Quality);
		sprintf((char *)air_2_buf,"CH4:%2.3f ppm ",AIR_Quality);


	 	//OLED��ʾŨ��
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,5,air_2_buf,16);
		OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
#endif
		
		//��ʱ�����������
		delay_ms(1000);
	}
}
	
//����5 mq4 ���� ����Ũ�Ȳɼ�	
void MQ4_task(void *parg)
{
	OS_ERR err;
	
	/*************************
	MQ4 �� MQ135 ���𲻴����ʹ����ͬ��ת���ڣ�
	����Ȼ�������£�ʵ��AOUT�˵ĵ�ѹΪ0.0.725V��
	����⵽��Ȼ��ʱ����ѹ����0.1V��ʵ�ʼ�⵽������Ũ������200ppm
	*******************************************/

	//mq135ת��ֵ
	int MQ135_ADC_ConvertedValue_Local;
	int CH4_ppm;
	int Voltage;
	char MQ4[20] = {0};
	
	dgb_printf_safe("MQ4 task running\r\n");
	
	while(1)
	{
		//�ѵ�ƽ��ģ���ź�ת������ֵ  ��ʽ��ת���� = ������ ADCֵ /4096 * 3.3
		Voltage = MQ135_ADC_ConvertedValue/4096*3.3;
		
//����Ȼ��������,ʵ��AOUT�˵�ѹΪ1.29V,����⵽��Ȼ��ʱ,ÿ����0.1V,ʵ�ʱ�����������200ppm
		CH4_ppm = (Voltage - 1.29) / 0.1 * 200;
	
		//��ϴ���
		sprintf((char *)MQ4,"CH4:%2.3f ppm ",CH4_ppm);

		sprintf((char *)node_1.CH4concentration,"%2.3f",CH4_ppm);
		
		//OLED��ʾŨ��
		OSMutexPend(&g_mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		OLED_ShowString(0,4,(uint8_t *)MQ4,16);
		OSMutexPost(&g_mutex_oled,OS_OPT_POST_NONE,&err);
	
		//��ʱ�����������
		delay_ms(2000);
	}
}

//����6 ���Ź� 
void IWG_task(void *parg)
{
	
}
	
//����7 �ȴ�����ں˶��� ��Ϣ���н������� ��txt��ʽ����SD��
void SAVE_task(void *parg)
{
	OS_ERR err;
	OS_FLAGS flags=0;
	
	FIL fnew;			//�ļ�����
	u8 res=0;	 	//�ļ��������
	u8 state=0;	 	//�ļ��������
	UINT fwnum;			//�ļ��ɹ���д����
	
	u32 sd_size;
	u32 total,free;
	u8 *save_buf=0;
	u8 t=0;	
	
	//��������
	u8 ReadBuffer[200] = {0};
	//д������
	u8 WriteBuffer[] = 
	"NO.101 222 Air:0.123ppm CH4:5000ppm Temp:26.4�� Humi:45% 2020/11/17 16:53";
	//�ļ���
	u8 bmp_name[] = "0:Fatfs.txt";

	//dgb_printf_safe("SD card task running\r\n");	
	//���ڴ�ӡSD�������Ϣ
	show_sdcard_info();	
//	//��ʾSD������
//	OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
//	LCD_ShowString(30,10,200,16,16,(u8 *)"SD Card OK    ");
//	LCD_ShowString(30,30,200,16,16,(u8 *)"SD Card Size:     MB");
//	LCD_ShowNum(30+13*8,30,SDCardInfo.CardCapacity>>20,5,16);
//	OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	

//	//dgb_printf_safe("\r\n���������ļ�ϵͳд����\r\n");
//	//�Ƚ��ļ������Ƿ����
//	
//	state = f_open(&fnew, (const char *)bmp_name, FA_CREATE_ALWAYS|FA_WRITE|FA_READ);   // ���ļ�
//	if(state != FR_OK) // ��ʧ��
//	{
//		//dgb_printf_safe("���ļ�ʧ��\r\n");
//		state = f_open(&fnew,(const char *)bmp_name, FA_CREATE_NEW|FA_WRITE|FA_READ);  // ����
//		if(state != FR_OK) // ����ʧ��
//		{
//			//dgb_printf_safe("�����ļ�ʧ��\r\n");
//			while(1)
//			{
//				LCD_ShowString(30,240,200,16,16,(u8 *)"create file fail");
//				delay_ms(1000);
//			}
//		}
//	}
//	else{
//		//dgb_printf_safe("��/�����ļ��ɹ�\r\n");
//	}
//	//�ļ���λ����λ���ļ�ĩβ
//	state = f_lseek(&fnew,f_size(&fnew)-1);
//	state = f_write(&fnew,WriteBuffer,sizeof(WriteBuffer),&fwnum);	
//	if(state == FR_OK)
//	{
//		//dgb_printf_safe("���ļ�д������Ϊ��\r\n%s\r\n",WriteBuffer);
//		//�ļ���λ����λ���ļ�ĩβ
//		state = f_lseek(&fnew,f_size(&fnew)-1);
//		if(state == FR_OK)
//		{
//			f_printf(&fnew,"\n�������һ��\n");
//			state = f_lseek(&fnew,0);
//			state = f_read(&fnew,ReadBuffer,f_size(&fnew),&fwnum);
//			if(res == FR_OK)
//			{
//				//dgb_printf_safe("�ļ���ȡ�ɹ����������ֽ����ݣ�%d\r\n",fwnum);
//				//dgb_printf_safe("���ļ���ȡ������Ϊ��\r\n%s\r\n",ReadBuffer);
//				delay_ms(1000);
//			}		
//		}
//		f_close(&fnew);
//	}
//	else{
//		//dgb_printf_safe("!!�ļ�д��ʧ��(%d)\r\n",res);
//		delay_ms(1000);
//	}

	while(1)
	{
		//dgb_printf_safe("SAVE   SAVE   SAVE\r\n");
		//��ʱ�����������
		delay_ms(1000);
	}	
}


//����8 LORAת�� �ȴ�����ں˶��� ��Ϣ���н������� usart3 
//�������ݷ�������λ��
void Send_Node(NODE *p,u8 len)     
{
    static u8 date,i;	   
	for(i=0;i<len;i++)
	{
		date= *(((u8*) &p->device_id)+i);    
		USART_SendData(USART3,date);   
		while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET); 
	}
}

void LORA_task(void *p_arg)
{
	OS_ERR err; 
	//��Ϣ���н��ս��
	int i=0;	
	uint8_t *LORA_res=NULL;
	OS_MSG_SIZE LORA_size;
	char LORA[100] = {0};
	
	dgb_printf_safe("LORA task running\r\n");
	
	while(1)
	{
		
		//�ȴ���Ϣ����
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
		
		//���ݽ���
		
		
		if(USART3_RX_STA&0x8000)
		{                                           
			int len=USART3_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���
			for(i = 0;i < len;i++)
			{
				LORA[i] = USART3_RX_BUF[i];
			}
			
			//�����ṹ��
			memcpy(&node_temp,&USART3_RX_BUF,len);
			//ʶ��ڵ�
			//id 1~4 Ϊ������ڵ�
			OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);		
			if(!strncmp(node_temp.device_id,"1",1)){
				memcpy(&node_1,&node_temp,len);
				dgb_printf_safe("node_1  id:  %s\r\n",node_1.device_id);
				dgb_printf_safe("node_1  humi:  %s\r\n",node_1.humidity);
				dgb_printf_safe("node_1  temp:  %s\r\n",node_1.temperature);			
				LCD_ShowString(30,250,108,24,24,"NODE ID: ");
				LCD_ShowString(138,250,300,24,24,(char *)node_1.device_id);
				LCD_ShowString(150+12,250,200,24,24,"TEMP:    C");
				LCD_ShowString(210+12,250,300,24,24,(char *)node_1.temperature);
				LCD_ShowString(270+24,250,200,24,24,"HUMI:    %");
				LCD_ShowString(320+24,250,300,24,24,(char *)node_1.humidity);
				LCD_ShowString(30,275,300,24,24,"CH4:      ppm");
				LCD_ShowString(90,275,300,24,24,(char *)node_1.CH4concentration);
			}
			if(!strncmp(node_temp.device_id,"2",1)){
				memcpy(&node_2,&node_temp,len);
				dgb_printf_safe("node_2  id:  %s\r\n",node_2.device_id);
				dgb_printf_safe("node_2  humi:  %s\r\n",node_2.humidity);
				dgb_printf_safe("node_2  temp:  %s\r\n",node_2.temperature);			
				LCD_ShowString(30,300,108,24,24,"NODE ID: ");
				LCD_ShowString(138,300,300,24,24,(char *)node_2.device_id);
				LCD_ShowString(150+12,300,200,24,24,"TEMP:    C");
				LCD_ShowString(210+12,250+50,300,24,24,(char *)node_2.temperature);
				LCD_ShowString(270+24,250+50,200,24,24,"HUMI:    %");
				LCD_ShowString(320+24,250+50,300,24,24,(char *)node_2.humidity);
				LCD_ShowString(30,275+50,300,24,24,"CH4:     ppm");
				LCD_ShowString(90,275+50,300,24,24,(char *)node_2.CH4concentration);
			}			
			if(!strncmp(node_temp.device_id,"3",1)){
				memcpy(&node_3,&node_temp,len);
				dgb_printf_safe("node_3  id:  %s\r\n",node_3.device_id);
				dgb_printf_safe("node_3  humi:  %s\r\n",node_3.humidity);
				dgb_printf_safe("node_3  temp:  %s\r\n",node_3.temperature);			
				LCD_ShowString(30,250+100,108,24,24,"NODE ID: ");
				LCD_ShowString(138,250+100,300,24,24,(char *)node_3.device_id);
				LCD_ShowString(150+12,250+100,200,24,24,"TEMP:    C");
				LCD_ShowString(210+12,250+100,300,24,24,(char *)node_3.temperature);
				LCD_ShowString(270+24,250+100,200,24,24,"HUMI:    %");
				LCD_ShowString(320+24,250+100,300,24,24,(char *)node_3.humidity);
				LCD_ShowString(30,275+100,300,24,24,"CH4:     ppm");
				LCD_ShowString(90,275+100,300,24,24,(char *)node_3.CH4concentration);
			}
			if(!strncmp(node_temp.device_id,"4",1)){
				memcpy(&node_4,&node_temp,len);
				dgb_printf_safe("node_4  id:  %s\r\n",node_4.device_id);
				dgb_printf_safe("node_4  humi:  %s\r\n",node_4.humidity);
				dgb_printf_safe("node_4  temp:  %s\r\n",node_4.temperature);			
				LCD_ShowString(30,250+150,108,24,24,"NODE ID: ");
				LCD_ShowString(138,250+150,300,24,24,(char *)node_4.device_id);
				LCD_ShowString(150+12,250+150,200,24,24,"TEMP:    C");
				LCD_ShowString(210+12,250+150,300,24,24,(char *)node_4.temperature);
				LCD_ShowString(270+24,250+150,200,24,24,"HUMI:    %");
				LCD_ShowString(320+24,250+150,300,24,24,(char *)node_4.humidity);
				LCD_ShowString(30,275+150,300,24,24,"CH4:     ppm");
				LCD_ShowString(90,275+150,300,24,24,(char *)node_4.CH4concentration);
			}
			
			//id == 5 Ϊң��ڵ�
			if(!strncmp(node_temp.device_id,"5",1)){
				memcpy(&node_5,&node_temp,len);
				dgb_printf_safe("node_5  id:  %s\r\n",node_5.device_id);
				dgb_printf_safe("node_5  humi:  %s\r\n",node_5.humidity);
				dgb_printf_safe("node_5  temp:  %s\r\n",node_5.temperature);			
				LCD_ShowString(30,250+200,108,24,24,"NODE ID: ");
				LCD_ShowString(138,250+200,300,24,24,(char *)node_5.device_id);
				LCD_ShowString(150+12,250+200,200,24,24,"TEMP:    C");
				LCD_ShowString(210+12,250+200,300,24,24,(char *)node_5.temperature);
				LCD_ShowString(270+24,250+200,200,24,24,"HUMI:    %");
				LCD_ShowString(320+24,250+200,300,24,24,(char *)node_5.humidity);
				LCD_ShowString(30,275+200,300,24,24,"LIGHT:   ");
				LCD_ShowString(102,275+200,300,24,24,(char *)node_5.light);
				LCD_ShowString(138,275+200,300,24,24,"CH4:     ppm");
				LCD_ShowString(198,275+200,300,24,24,(char *)node_5.CH4concentration);				
				LCD_ShowString(30,300+200,300,24,24,"PITCH:     C");
				LCD_ShowString(30+72,300+200,300,24,24,(char *)node_5.Pitch);				
				LCD_ShowString(30+144+12,300+200,300,24,24,"ROLL:     C");
				LCD_ShowString(30+144+60+12,300+200,300,24,24,(char *)node_5.Roll);
				LCD_ShowString(30+144+144+24,300+200,300,24,24,"YAW:     C");
				LCD_ShowString(30+144+144+48+24,300+200,300,24,24,(char *)node_5.Yaw);				
			}
			OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
			
			USART3_RX_STA = 0;
		}			
	}
}
	
//����9 rtcʱ����ʾ	������ oled��ʾ	
void RTC_task(void *parg)
{
	OS_ERR err;
	u8 t = 0;	
	char date_time[20] = {0};
	dgb_printf_safe("RTC task running\r\n");
	
	
//OLED��ʾʱ��		
#if 0		
	
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
	
	
//LCD��ʾʱ��		
#if 1	
	//��ʾʱ����
	POINT_COLOR=BLUE;
	OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
	LCD_ShowString(360,20,200,16,16,"    -  -  ");	   
	LCD_ShowString(360,40,200,16,16,"  :  :  ");	
	OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);		
	
	while(1)
	{
		if(t!=calendar.sec)
		{
			t=calendar.sec;
			OSMutexPend(&g_mutex_lcd,0,OS_OPT_PEND_BLOCKING,NULL,&err);
			LCD_ShowNum(360,20,calendar.w_year,4,16);									  
			LCD_ShowNum(400,20,calendar.w_month,2,16);									  
			LCD_ShowNum(424,20,calendar.w_date,2,16);	 
			switch(calendar.week)
			{
				case 0:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Sunday   ");
					break;
				case 1:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Monday   ");
					break;
				case 2:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Tuesday  ");
					break;
				case 3:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Wednesday");
					break;
				case 4:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Thursday ");
					break;
				case 5:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Friday   ");
					break;
				case 6:
					LCD_ShowString(360,60,200,16,16,(u8 *)"Saturday ");
					break;  
			}
			LCD_ShowNum(360,40,calendar.hour,2,16);									  
			LCD_ShowNum(384,40,calendar.min,2,16);									  
			LCD_ShowNum(408,40,calendar.sec,2,16);
			OSMutexPost(&g_mutex_lcd,OS_OPT_NONE,&err);	
		}	
		delay_ms(1000);
	}
#endif	
	

}
	
//����10.ϵͳ������ʾ led0������
void led0_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		LED0=0;
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		LED0=1;
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms
	}
}

//����11.�ⱨ������ led1������
// LED1 = 1 ʱ����������
// LED1 = 0 ʱ�������ر�
void led1_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	int concen = 0;
	while(1)
	{
		OSMutexPend(&g_mutex_NODE,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		concen = atoi(node_1.CH4concentration);
		OSMutexPost(&g_mutex_NODE,OS_OPT_POST_NONE,&err);
		
		//dgb_printf_safe("concen: %d \r\n",concen);
		if(concen > 60)
		{	
			LED1 = 1;
			delay_ms(1000);
			LED1 = 0;	
			delay_ms(2000);
		}
		else{
			LED1 = 1;	
		}
		
		delay_ms(2000);
	}
}


//����12.������������
// BEEP = 1 ʱ����������
// BEEP = 0 ʱ�������ر�
void BEEP_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	int concen = 0;
			
	dgb_printf_safe("BEEP task running\r\n");

	while(1)
	{	
		OSMutexPend(&g_mutex_NODE,0,OS_OPT_PEND_BLOCKING,NULL,&err);	
		concen = atoi(node_1.CH4concentration);
		OSMutexPost(&g_mutex_NODE,OS_OPT_POST_NONE,&err);
		
		//dgb_printf_safe("concen: %d \r\n",concen);
		if(concen > 60)
		{	
			BEEP = 0;
			delay_ms(1000);	
			BEEP = 1;
			delay_ms(1000);
		}
		else{
			BEEP = 0;	
		}
		
		delay_ms(1000);
	}
}
	

//����13.��������
void KEY_task(void *p_arg)
{
	OS_ERR err;

	OS_FLAGS flags=0;
	
	//dgb_printf_safe("KEY task running\r\n");
	
	while(1)
	{
		//һֱ�����ȴ��¼���־��1���ȴ��ɹ��󣬽���Ӧ��0
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
		
		//WK_UP����
		if(flags & FLAG_GRP_WK_UP_DOWN){	
			//��ֹEXTI0�����ж�
			NVIC_DisableIRQ(EXTI0_IRQn);
			if(WK_UP == 1){
				delay_ms(20);//����
				if(WK_UP == 1){
					//dgb_printf_safe("WK_UP happend\r\n");
					//�ȴ�����WK_UP�ͷ�
					while(WK_UP == 1){
						delay_ms(1);
					}						
					
					BEEP = !BEEP;
				}
			}	
			//����EXTI0�����ж�
			NVIC_EnableIRQ(EXTI0_IRQn);	
			//���EXTI0�жϱ�־λ
			EXTI_ClearITPendingBit(EXTI_Line0);			
		}
		
		//KEY2����
		if(flags & FLAG_GRP_KEY2_DOWN){	
			if(KEY2 == 0){
				delay_ms(20);//����
				if(KEY2 == 0){
					//dgb_printf_safe("key2 happend\r\n");
					//�ȴ�����2�ͷ�
					while(KEY2==0){
						delay_ms(1);
					}	
					
					LED0 = !LED0;
				}
			}	
			//���EXTI2�жϱ�־λ
			EXTI_ClearITPendingBit(EXTI_Line2);			
		}
		
		//KEY1����
		if(flags & FLAG_GRP_KEY1_DOWN){	
			if(KEY1 == 0){
				delay_ms(20);//����
				if(KEY1 == 0){
					//dgb_printf_safe("key1 happend\r\n");
					//�ȴ�����1�ͷ�
					while(KEY1==0){
						delay_ms(1);
					}	
					//������Ϣ
//					OSSemPost(&g_json,OS_OPT_POST_1,&err);//�����ź���
				}
			}	
			//���EXTI3�жϱ�־λ
			EXTI_ClearITPendingBit(EXTI_Line3);			
		}
		
//		//KEY0����
//		if(flags & FLAG_GRP_KEY0_DOWN){	
//			if(KEY0 == 0){
//				delay_ms(20);//����
//				if(KEY0 == 0){
//					dgb_printf_safe("key0 happend\r\n");
//					//�ȴ�����0�ͷ�
//					while(KEY0==0){
//						delay_ms(1);
//					}		
//					
//					LED1 = !LED1;
//					LED0 = !LED0;
//				}
//			}	
//			//���EXTI4�жϱ�־λ
//			EXTI_ClearITPendingBit(EXTI_Line4);			
//		}
		
		delay_ms(1000);
	}
}
	

//����14 MPU6050
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void mpu6050_task(void *p_arg)
{
	OS_ERR err; 
	CPU_SR_ALLOC();
	int res = 0;
	u8 t=0;			//Ĭ�Ͽ����ϱ�
	u8 key;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�	
	
	dgb_printf_safe("MPU6050 task running\r\n");
//	LCD_ShowString(30,300,200,16,16," Temp:    . C");	
// 	LCD_ShowString(30,320,200,16,16,"Pitch:    . C");	
// 	LCD_ShowString(30,340,200,16,16," Roll:    . C");	 
// 	LCD_ShowString(30,360,200,16,16," Yaw :    . C");	
	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			res = MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			printf("res_1: %d \r\n",res);
			res =MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			printf("res_2: %d \r\n",res);
			if(temp<0)
			{
				LCD_ShowChar(30+48,300,'-',16,0);		//��ʾ����
				temp=-temp;		//תΪ����
			}
			else LCD_ShowChar(30+48,300,' ',16,0);		//ȥ������ 
			LCD_ShowNum(30+48+8,300,temp/100,3,16);		//��ʾ��������	    
			LCD_ShowNum(30+48+40,300,temp%10,1,16);		//��ʾС������ 
			
			temp=pitch*10;
			if(temp<0)
			{
				LCD_ShowChar(30+48,320,'-',16,0);		//��ʾ����
				temp=-temp;		//תΪ����
			}
			else LCD_ShowChar(30+48,320,' ',16,0);		//ȥ������ 
			LCD_ShowNum(30+48+8,320,temp/10,3,16);		//��ʾ��������	    
			LCD_ShowNum(30+48+40,320,temp%10,1,16);		//��ʾС������ 
			
			temp=roll*10;
			if(temp<0)
			{
				LCD_ShowChar(30+48,340,'-',16,0);		//��ʾ����
				temp=-temp;		//תΪ����
			}
			else LCD_ShowChar(30+48,340,' ',16,0);		//ȥ������ 
			LCD_ShowNum(30+48+8,340,temp/10,3,16);		//��ʾ��������	    
			LCD_ShowNum(30+48+40,340,temp%10,1,16);		//��ʾС������ 
			
			temp=yaw*10;
			if(temp<0)
			{
				LCD_ShowChar(30+48,360,'-',16,0);		//��ʾ����
				temp=-temp;		//תΪ����
			}
			else LCD_ShowChar(30+48,360,' ',16,0);		//ȥ������ 
			LCD_ShowNum(30+48+8,360,temp/10,3,16);		//��ʾ��������	    
			LCD_ShowNum(30+48+40,360,temp%10,1,16);		//��ʾС������  
			
		}
		delay_ms(2000);			//��ʱ500ms
		
	}
}



//����15.ϵͳ�ڴ�ռ�ü���
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

//����16.�����������
void float_task(void *p_arg)
{
	OS_ERR err; 
	CPU_SR_ALLOC();
	static float float_num=0.01;
	char node_message[16] = {0};
	while(1)
	{
		float_num+=0.01f;
		
		dgb_printf_safe("float_num��ֵΪ: %.4f\r\n",float_num);

		LoRa_SendData();
				
		delay_ms(6000);			//��ʱ500ms
		
	}
}



//����17 �ȴ���λ�������ϴ����
//��ѯ�����ڵ��ϴ�����λ��
void upload_task(void *p_arg)
{
	OS_ERR err; 
	CPU_SR_ALLOC();

	//��Ϣ���н�����λ������
	uint8_t *check_res=NULL;
	OS_MSG_SIZE CHECK_size;
	char CHECK[20] = {0};
	
	while(1)
	{
		//�ȴ���Ϣ����
		check_res = OSQPend((OS_Q*			)&g_queue_usart1,
							(OS_TICK		)0,
							(OS_OPT			)OS_OPT_PEND_BLOCKING,
							(OS_MSG_SIZE*	)&CHECK_size,
							(CPU_TS*		)NULL,
							(OS_ERR*		)&err);
		
		if(err != OS_ERR_NONE)
		{
			dgb_printf_safe("[TDLAS_task_usart2][OSQPend]Error Code = %d\r\n",err);		
		}
		
		//��������
		if(USART_RX_STA&0x8000)
		{                                           
			int len=USART_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���
			for(int i = 0;i < len;i++)
			{
				CHECK[i] = USART_RX_BUF[i];
			}

			//dgb_printf_safe("TDLAS:%s\r\n",USART2_RX_BUF);
			//dgb_printf_safe("CHECK:%s\r\n",CHECK);
			USART2_RX_STA = 0;
		}

		
		if(!strncmp(CHECK,"check",5))
		{
			printf("11111");
			json_test(&node_1);
//			json_test(&node_2);
//			json_test(&node_3);
//			json_test(&node_4);
			json_test_1(&node_5);
		}
	}
}

