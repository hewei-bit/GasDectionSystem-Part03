#include "mq135.h"


volatile u16 MQ135_ADC_ConvertedValue; 		//�������� ��ȡmq135��ֵ

//����ADC DMA GPIO
void mq135_init()
{
	//GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	//��DMA��GPIOCʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOC,ENABLE);
	
	//����GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			//��������
	GPIO_Init(GPIOC,&GPIO_InitStructure);					//GPIO��ʼ��
	
	//����DMA
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;			//ADC1��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&MQ135_ADC_ConvertedValue;	//�ڴ��ַ(Ҫ����ı�����ַ��ָ��)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					//����(���ڴ浽����)
	DMA_InitStructure.DMA_BufferSize = 1;								//�������ݵĴ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;			//�ڴ��ַ�̶�
	DMA_InitStructure.DMA_PeripheralDataSize = 				
	DMA_MemoryDataSize_HalfWord;										//�������ݵ�λ
	DMA_InitStructure.DMA_MemoryDataSize = 
	DMA_MemoryDataSize_HalfWord;										//�ڴ����ݵ�λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;						//DMAģʽ��ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;					//���ȼ�����
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;						//��ֹ�ڴ浽�ڴ�Ĵ���
	
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);							//����DMA1��ͨ��
	
	DMA_Cmd(DMA1_Channel1,ENABLE);										//DMAʹ��
	
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ȫ��
	//����ADC
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//��ֹɨ�跽ʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//��������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//Ҫת����ͨ����Ŀ
	ADC_Init(ADC1,&ADC_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);									//����ADCʱ�ӣ�ΪPCLK��8��Ƶ��9MHz
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, 
	ADC_SampleTime_55Cycles5);											//����ADC1_Channel1Ϊ55.5����������
	
	ADC_DMACmd(ADC1, ENABLE);		/* ʹ�� ADC1 DMA */
	ADC_Cmd(ADC1, ENABLE);			/* ʹ�� ADC1 */
	
	ADC_ResetCalibration(ADC1);		/* ʹ�ܸ�λУ׼�Ĵ��� */   
	while(ADC_GetResetCalibrationStatus(ADC1));/* �ȴ���λУ׼�Ĵ������ */
	
	ADC_StartCalibration(ADC1);		/* ��ʼ ADC1 У׼ */
	while(ADC_GetCalibrationStatus(ADC1));/* �ȴ�У׼��� */
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);/* ��ʼ ADC1 Software Conversion */ 

	
}



