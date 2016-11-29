/*****************************************************************************************************
�ļ���  ��battery.c

�ļ���������ز�����غ���

������  ��

����ʱ�䣺

������ʷ��

*****************************************************************************************************/
#include "stm32f10x.h"
#include "Battery.h"
#include "LED.h"

vu16 Bat_AD_Val = 0; //ADCֱ�Ӳɼ���������
vu16 Bat_AD_Calc = 0;
vu16 Bat_AD_Buf[Bat_AD_Len]={0x00}; //Battery ADC �������ݣ������˲���

//��ص�����������
//const u16 BatteryCalDat[5] = {2111, 2143, 2175, 2271, 2367};
const u16 BatteryCalDat[5] = {2063, 2125, 2175, 2271, 2337};	 //{2063, 2125, 2175, 2271, 2367};	 {2063, 2135, 2190, 2271, 2367};   {2096, 2173, 2289, 2474, 2650}

void DMA_Configuration(void);
void ADC_Configuration(void);
void Batt_ReadAD(void);
u8 Is_Charger_IN(void);

u8 BattVal, BattState = BATT_STATE_FULL;	//������
/*****************************************************************************************************
������  ��Batt_Init

��    �ܣ���س�ʼ������Ҫ�Ǻ�ADC��صĳ�ʼ����

��ڲ�������

���ڲ�������

����ֵ  ����
*****************************************************************************************************/
void Batt_Init(void)
{
u8 i;

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	/*Enable ADC1 clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable the DMA Interrupt  ADC */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//PC1 : ADC_IN 9 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	DMA_Configuration();   //ֱ���ڴ����
   	ADC_Configuration();

	for(i = 0; i < Bat_AD_Len; i++)
	{
		delay_ms(10);
		Batt_ReadAD();
	}

	//�����͵�������
	if(0xff == Batt_GetValue())
	{
		if(!Is_Charger_IN())
		{
			LTC3555_SendCmd(0xfe,0x03);	//�رյ�Դ
		}
	}

}
/*******************************************************************************
* Function Name  : void ADC_Configuration(void)
* Description    : Configures the ADC.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void ADC_Configuration(void)
{ 
	ADC_InitTypeDef ADC_InitStructure;

	ADC_InitStructure.ADC_Mode 				= 	ADC_Mode_Independent;  // ��������ģʽ
	ADC_InitStructure.ADC_ScanConvMode 		= 	ENABLE;		//ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = 	ENABLE;  //����ת��
	ADC_InitStructure.ADC_ExternalTrigConv 	= 	ADC_ExternalTrigConv_None; //�ⲿ������ֹ
	ADC_InitStructure.ADC_DataAlign 		= 	ADC_DataAlign_Right; //�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel 		= 	1;  //����ת����ͨ����

	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channel configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5); //��ص�ѹ
			
	ADC_Cmd(ADC1, ENABLE);						/* Enable ADC1 */ 
	ADC_DMACmd(ADC1, ENABLE);					/* Enable ADC1 DMA */    
	ADC_ResetCalibration(ADC1);					/* Enable ADC1 reset calibaration register */  
	while(ADC_GetResetCalibrationStatus(ADC1));	/* Check the end of ADC1 reset calibration register */
	ADC_StartCalibration(ADC1);					/* Start ADC1 calibaration */  
	while(ADC_GetCalibrationStatus(ADC1));		/* Check the end of ADC1 calibration */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		/* Start ADC1 Software Conversion */ 
}
/*******************************************************************************
* Function Name  : void DMA_Configuration(void)
* Description    : Configures the DMA.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);

	DMA_InitStructure.DMA_PeripheralBaseAddr = 	ADC1_DR_Address;	  			//STM_AD��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr 	= 	(u32)&Bat_AD_Val;		  		//ADC_RCVTab;//�ڴ��ַ
	DMA_InitStructure.DMA_DIR 				= 	DMA_DIR_PeripheralSRC;	 		//���䷽����
	DMA_InitStructure.DMA_BufferSize 		= 	1;								//��������С
	DMA_InitStructure.DMA_PeripheralInc 	= 	DMA_PeripheralInc_Disable;		//����DMA���������ģʽ�رգ�һ������
	DMA_InitStructure.DMA_MemoryInc 		= 	DMA_MemoryInc_Enable;			// ����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = 	DMA_PeripheralDataSize_HalfWord;//���������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize 	= 	DMA_MemoryDataSize_HalfWord;	//�ڴ������ֳ�
	DMA_InitStructure.DMA_Mode 				= 	DMA_Mode_Circular;				//����DMA�Ĵ���ģʽ���������ϵ�ѭ��ģʽ
	DMA_InitStructure.DMA_Priority 			= 	DMA_Priority_High;				//����DMA�����ȼ���
	DMA_InitStructure.DMA_M2M 				= 	DMA_M2M_Disable;				//����DMA��2��memory�еı����������
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* DMA IT enable */ 
//	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //ʹ��DMA��������ж�	  
	DMA_Cmd(DMA1_Channel1, ENABLE);/* Enable DMA channel1 */
}

void Batt_ReadAD(void)
{
	static u8 cnt = 0;	
	u16 sum = 0x00,max =0x00 , min = 0xffff;
	u8 i = 0, j = 4;
		
	Bat_AD_Buf[cnt++] = Bat_AD_Val;		// volatile u16 Bat_AD_Buf[10]={0x00}; //Battery ADC �������ݣ������˲���
	cnt %= Bat_AD_Len;

	if(cnt != 0) return;

	//ȥ�����С��ƽ��	
	for ( i = 0 ; i < Bat_AD_Len ; i++ )
	{
		sum += Bat_AD_Buf[i];

		if ( Bat_AD_Buf[i] > max )
		{
			max	= Bat_AD_Buf[i];
		}
		if ( Bat_AD_Buf[i] < min )
		{
			min = Bat_AD_Buf[i];
		}
	}
	sum -= min;	
	sum -= max;
	sum /= (Bat_AD_Len-2);	//ƽ��ֵ

	Bat_AD_Calc = sum;
}


/*****************************************************************************************************
������  ��Batt_GetValue

��    �ܣ���ȡ��ǰ��ص���

��ڲ�������

���ڲ�������

����ֵ  ��u8,��ȡ���ĵ�ǰ����ֵ	 0������	1��һ�����		2����������		3���������		4���ĸ����
*****************************************************************************************************/
u8 Batt_GetValue(void)
{
	/*if (Bat_AD_Calc > 2450 )
	{
		return 0xFF;
	}*/

	//��̶��Ķ���ֵ���бȽ�
	if(Bat_AD_Calc > BatteryCalDat[4])		return 4;
	else if(Bat_AD_Calc > BatteryCalDat[3])	return 3;
	else if(Bat_AD_Calc > BatteryCalDat[2])	return 2;
	else if(Bat_AD_Calc > BatteryCalDat[1])	return 1;
	else if(Bat_AD_Calc > BatteryCalDat[0])	return 0;
	else 								return 0xff;
}

/*****************************************************************************************************
������  ��Batt_GetState

��    �ܣ���ȡ��ǰ��صĹ���״̬

��ڲ�������

���ڲ�������

����ֵ  ��u8������״̬��		0�����		1���ŵ�		2������		3���޵��
*****************************************************************************************************/
u8 Batt_GetState(void)
{
//	if (BATT_NOT_EXIST)
//	{
//		return BATT_STATE_NOBATT;
//	}
	 if ( Is_Charger_IN() && !BATT_STAT_GET )   //������������ ͬʱ ���״̬�ܽ�ָʾ�ڳ��
	{
		ChargerLed_Cfg(ENABLE);
		DisChargerLed_Cfg(DISABLE);
		return BATT_STATE_CHARGING;
	}																		 
	else if ( Is_Charger_IN() && BATT_STAT_GET ) 	//����
	{
		ChargerLed_Cfg(DISABLE);
		DisChargerLed_Cfg(ENABLE);
		return  BATT_STATE_FULL;
	}
	else if ( !Is_Charger_IN() )	//�ŵ�
	{
		return BATT_STATE_DISCHARGE;
	}

	//��ص�NTC����
	//��ر�����

	return 0;
}




























