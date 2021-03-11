#include "sys.h"
#include "uart4.h"
#include "delay.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos ʹ��
#endif

u8 UART4_RX_BUF[UART4_MAX_RECV_LEN]; //���ջ���,���UART4_MAX_RECV_LEN���ֽ�.
u8 UART4_TX_BUF[UART4_MAX_SEND_LEN]; //���ͻ���,���UART4_MAX_SEND_LEN�ֽ�

vu16 UART4_RX_STA = 0;

void uart4_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //ʹ��UART4ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	

    //USART4�˿�����GPIOC11��GPIOC10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //�ٶ�50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);               	// ��ʼ��GPIOC10   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);        			//��ʼ��GPIOC11  

	//Usart4 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                           //����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
	
	//USART4 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                                     //������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //�շ�ģʽ
    
	
	USART_Init(UART4, &USART_InitStructure);                                        //��ʼ������4
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //�����ж�
	USART_Cmd(UART4, ENABLE); //ʹ�ܴ���4
	USART_ClearFlag(UART4,USART_FLAG_TC);
}

void UART4_IRQHandler(void) //����1�жϷ������
{
    u8 Res;
    OS_ERR err;

    int len = 0;
    int t = 0;

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
    {
        Res = USART_ReceiveData(UART4); //(USART1->DR);	//��ȡ���յ�������

        if ((UART4_RX_STA & 0x8000) == 0) //����δ���
        {
            if (UART4_RX_STA & 0x4000) //���յ���0x0d
            {
                if (Res != 0x0a)
                    UART4_RX_STA = 0; //���մ���,���¿�ʼ
                else
                    UART4_RX_STA |= 0x8000; //���������
            }
            else //��û�յ�0X0D
            {
                if (Res == 0x0d)
                    UART4_RX_STA |= 0x4000;
                else
                {
                    UART4_RX_BUF[UART4_RX_STA & 0X3FFF] = Res;
                    UART4_RX_STA++;
                    if (UART4_RX_STA > (UART4_MAX_RECV_LEN - 1))
                        UART4_RX_STA = 0; //�������ݴ���,���¿�ʼ����
                }
            }
        }
    }
#if 0  //������Ϣ����	
	if(UART4_RX_STA&0x8000)
	{
		len=UART4_RX_STA&0x3FFF;//�õ��˴ν������ݵĳ���
		printf("TDLAS:%s\r\n",UART4_RX_BUF);
	

		OSQPost((OS_Q*		)&g_queue_usart1,
				(void *     )UART4_RX_BUF,
				(OS_MSG_SIZE)len,
				(OS_OPT		)OS_OPT_POST_FIFO,
				(OS_ERR*	)&err);
		if(err != OS_ERR_NONE)
		{
			printf("[UART4_IRQHandler]OSQPost error code %d\r\n",err);
		}
	}
#endif	
}



void u4_printf(char *fmt, ...)
{
    u16 i, j;
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)UART4_TX_BUF, fmt, ap);
    va_end(ap);
    i = strlen((const char *)UART4_TX_BUF); //�˴η������ݵĳ���
    for (j = 0; j < i; j++)                 //ѭ����������
    {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);   	//�ȴ��ϴδ������
        USART_SendData(UART4, (uint8_t)UART4_TX_BUF[j]); 				//�������ݵ�����3
    }
}
