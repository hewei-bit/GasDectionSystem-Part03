#include "dht11.h"
#include "delay.h"
  
//��λDHT11
void DHT11_Rst(void)	   
{                 
	DHT11_IO_OUT(); 	//SET OUTPUT
    DHT11_DQ_OUT=0; 	//����DQ
    delay_ms(20);    	//��������18ms
    DHT11_DQ_OUT=1; 	//DQ=1 
	delay_us(30);     	//��������20~40us
}
//�ȴ�DHT11�Ļ�Ӧ
//����1:δ��⵽DHT11�Ĵ���
//����0:����
u8 DHT11_Check(void) 	   
{   
	u8 retry=0;
	DHT11_IO_IN();//SET INPUT	 
    while (DHT11_DQ_IN&&retry<100)//DHT11������40~80us
	{
		retry++;
		delay_us(1);
	};	 
	if(retry>=100)return 1;
	else retry=0;
    while (!DHT11_DQ_IN&&retry<100)//DHT11���ͺ���ٴ�����40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;	    
	return 0;
}
//��DHT11��ȡһ��λ
//����ֵ��1/0
u8 DHT11_Read_Bit(void) 			 
{
 	u8 retry=0;
	while(DHT11_DQ_IN&&retry<100)//�ȴ���Ϊ�͵�ƽ
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!DHT11_DQ_IN&&retry<100)//�ȴ���ߵ�ƽ
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);//�ȴ�40us
	if(DHT11_DQ_IN)return 1;
	else return 0;		   
}
//��DHT11��ȡһ���ֽ�
//����ֵ������������
u8 DHT11_Read_Byte(void)    
{        
    u8 i,dat;
    dat=0;
	for (i=0;i<8;i++) 
	{
   		dat<<=1; 
	    dat|=DHT11_Read_Bit();
    }						    
    return dat;
}
//��DHT11��ȡһ������
//temp:�¶�ֵ(��Χ:0~50��)
//humi:ʪ��ֵ(��Χ:20%~90%)
//����ֵ��0,����;1,��ȡʧ��
int32_t DHT11_Read_Data(uint8_t *pdht_data)    
{        
 	u8 buf[5];
	
	u8 i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//��ȡ40λ����
		{
			buf[i]=DHT11_Read_Byte();
		}
		for(i=0;i<4;i++)//��ȡ40λ����
		{
			pdht_data[i]=buf[i];
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			return 0;
		}
	}
	else return 1;
		    
}

//��ʼ��DHT11��IO�� DQ ͬʱ���DHT11�Ĵ���
//����1:������
//����0:����    	 
u8 DHT11_Init(void)
{	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);	//ʹ��PG�˿�ʱ��
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				//PG11�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);				 	//��ʼ��IO��
 	GPIO_SetBits(GPIOG,GPIO_Pin_11);						//PG11 �����
			    
	DHT11_Rst();  //��λDHT11
	return DHT11_Check();//�ȴ�DHT11�Ļ�Ӧ
} 

//static GPIO_InitTypeDef		GPIO_InitStructure;

int32_t dht11_read(uint8_t *pdht_data)
{
	uint32_t t=0;
	
	uint8_t d;
	
	int32_t i=0;
	int32_t j=0;
	uint32_t check_sum=0;
	
	//��֤����Ϊ���ģʽ
	DHT11_IO_OUT(); 	
	
	PGout(11)=0;
	delay_ms(20);
	
	PGout(11)=1;	
	delay_us(30);
	
	//��֤����Ϊ����ģʽ
	DHT11_IO_IN();

	//�ȴ�DHT11��Ӧ���ȴ��͵�ƽ����
	t=0;
	while(PGin(11))
	{
		t++;
		delay_us(1);
		if(t >= 4000)
			return -1;
	}
	
	//���͵�ƽ����100us
	t=0;
	while(PGin(11)==0)
	{
		t++;
		delay_us(1);
		if(t >= 100)
			return -2;
	}
	
	
	//���ߵ�ƽ����100us
	t=0;
	while(PGin(11))
	{
		t++;
		delay_us(1);
		if(t >= 100)
			return -3;
	}
	
	//��������5���ֽ�
	for(j=0; j<5; j++)
	{
		d = 0;
		//���8��bit���ݵĽ��գ���λ����
		for(i=7; i>=0; i--)
		{
			//�ȴ��͵�ƽ�������
			t=0;
			while(PGin(11)==0)
			{	
				t++;
				delay_us(1);
				if(t >= 100)
					return -4;
			}	
			
			delay_us(40);
			
			if(PGin(11))
			{
				d|=1<<i;
				//�ȴ�����1�ĸߵ�ƽʱ��������
				t=0;
				while(PGin(11))
				{
					t++;
					delay_us(1);
					if(t >= 100)
						return -5;
				}			
			
			}
		}	
		pdht_data[j] = d;
	}
	
	//ͨ�ŵĽ���
	delay_us(100);
	
	//����У���
	check_sum=pdht_data[0]+pdht_data[1]+pdht_data[2]+pdht_data[3];
	
	check_sum = check_sum & 0xFF;
	if(check_sum != pdht_data[4])
		return -6;
	
	return 0;
}





