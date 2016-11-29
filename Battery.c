/*****************************************************************************************************
文件名  ：battery.c

文件描述：电池操作相关函数

创建人  ：

创建时间：

更改历史：

*****************************************************************************************************/
#include "stm32f10x.h"
#include "Battery.h"
#include "LED.h"

vu16 Bat_AD_Val = 0; //ADC直接采集到的数据
vu16 Bat_AD_Calc = 0;
vu16 Bat_AD_Buf[Bat_AD_Len]={0x00}; //Battery ADC 缓存数据，用于滤波等

//电池电量定标数据
//const u16 BatteryCalDat[5] = {2111, 2143, 2175, 2271, 2367};
const u16 BatteryCalDat[5] = {2063, 2125, 2175, 2271, 2337};	 //{2063, 2125, 2175, 2271, 2367};	 {2063, 2135, 2190, 2271, 2367};   {2096, 2173, 2289, 2474, 2650}

void DMA_Configuration(void);
void ADC_Configuration(void);
void Batt_ReadAD(void);
u8 Is_Charger_IN(void);

u8 BattVal, BattState = BATT_STATE_FULL;	//调试用
/*****************************************************************************************************
函数名  ：Batt_Init

功    能：电池初始化（主要是和ADC相关的初始化）

入口参数：无

出口参数：无

返回值  ：无
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

	DMA_Configuration();   //直接内存管理
   	ADC_Configuration();

	for(i = 0; i < Bat_AD_Len; i++)
	{
		delay_ms(10);
		Batt_ReadAD();
	}

	//开机低电量保护
	if(0xff == Batt_GetValue())
	{
		if(!Is_Charger_IN())
		{
			LTC3555_SendCmd(0xfe,0x03);	//关闭电源
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

	ADC_InitStructure.ADC_Mode 				= 	ADC_Mode_Independent;  // 独立工作模式
	ADC_InitStructure.ADC_ScanConvMode 		= 	ENABLE;		//扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = 	ENABLE;  //连续转换
	ADC_InitStructure.ADC_ExternalTrigConv 	= 	ADC_ExternalTrigConv_None; //外部触发禁止
	ADC_InitStructure.ADC_DataAlign 		= 	ADC_DataAlign_Right; //数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel 		= 	1;  //用于转换的通道数

	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channel configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5); //电池电压
			
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

	DMA_InitStructure.DMA_PeripheralBaseAddr = 	ADC1_DR_Address;	  			//STM_AD地址
	DMA_InitStructure.DMA_MemoryBaseAddr 	= 	(u32)&Bat_AD_Val;		  		//ADC_RCVTab;//内存地址
	DMA_InitStructure.DMA_DIR 				= 	DMA_DIR_PeripheralSRC;	 		//传输方向单向
	DMA_InitStructure.DMA_BufferSize 		= 	1;								//缓冲区大小
	DMA_InitStructure.DMA_PeripheralInc 	= 	DMA_PeripheralInc_Disable;		//设置DMA的外设递增模式关闭，一个外设
	DMA_InitStructure.DMA_MemoryInc 		= 	DMA_MemoryInc_Enable;			// 设置DMA的内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize = 	DMA_PeripheralDataSize_HalfWord;//外设数据字长
	DMA_InitStructure.DMA_MemoryDataSize 	= 	DMA_MemoryDataSize_HalfWord;	//内存数据字长
	DMA_InitStructure.DMA_Mode 				= 	DMA_Mode_Circular;				//设置DMA的传输模式：连续不断的循环模式
	DMA_InitStructure.DMA_Priority 			= 	DMA_Priority_High;				//设置DMA的优先级别
	DMA_InitStructure.DMA_M2M 				= 	DMA_M2M_Disable;				//设置DMA的2个memory中的变量互相访问
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* DMA IT enable */ 
//	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //使能DMA传输完成中断	  
	DMA_Cmd(DMA1_Channel1, ENABLE);/* Enable DMA channel1 */
}

void Batt_ReadAD(void)
{
	static u8 cnt = 0;	
	u16 sum = 0x00,max =0x00 , min = 0xffff;
	u8 i = 0, j = 4;
		
	Bat_AD_Buf[cnt++] = Bat_AD_Val;		// volatile u16 Bat_AD_Buf[10]={0x00}; //Battery ADC 缓存数据，用于滤波等
	cnt %= Bat_AD_Len;

	if(cnt != 0) return;

	//去最大，最小求平均	
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
	sum /= (Bat_AD_Len-2);	//平均值

	Bat_AD_Calc = sum;
}


/*****************************************************************************************************
函数名  ：Batt_GetValue

功    能：读取当前电池电量

入口参数：无

出口参数：无

返回值  ：u8,读取到的当前电量值	 0：报警	1：一格电量		2：两个电量		3：三格电量		4：四格电量
*****************************************************************************************************/
u8 Batt_GetValue(void)
{
	/*if (Bat_AD_Calc > 2450 )
	{
		return 0xFF;
	}*/

	//与固定的定标值进行比较
	if(Bat_AD_Calc > BatteryCalDat[4])		return 4;
	else if(Bat_AD_Calc > BatteryCalDat[3])	return 3;
	else if(Bat_AD_Calc > BatteryCalDat[2])	return 2;
	else if(Bat_AD_Calc > BatteryCalDat[1])	return 1;
	else if(Bat_AD_Calc > BatteryCalDat[0])	return 0;
	else 								return 0xff;
}

/*****************************************************************************************************
函数名  ：Batt_GetState

功    能：获取当前电池的工作状态

入口参数：无

出口参数：无

返回值  ：u8，工作状态。		0：充电		1：放电		2：充满		3：无电池
*****************************************************************************************************/
u8 Batt_GetState(void)
{
//	if (BATT_NOT_EXIST)
//	{
//		return BATT_STATE_NOBATT;
//	}
	 if ( Is_Charger_IN() && !BATT_STAT_GET )   //有适配器接入 同时 电池状态管脚指示在充电
	{
		ChargerLed_Cfg(ENABLE);
		DisChargerLed_Cfg(DISABLE);
		return BATT_STATE_CHARGING;
	}																		 
	else if ( Is_Charger_IN() && BATT_STAT_GET ) 	//充满
	{
		ChargerLed_Cfg(DISABLE);
		DisChargerLed_Cfg(ENABLE);
		return  BATT_STATE_FULL;
	}
	else if ( !Is_Charger_IN() )	//放电
	{
		return BATT_STATE_DISCHARGE;
	}

	//电池的NTC换掉
	//电池本身坏掉

	return 0;
}




























