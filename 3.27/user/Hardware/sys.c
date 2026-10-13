#include "stm32f10x.h"

void RCC_Configuration(void){
	ErrorStatus HSEStartUPStatus;
	//RCC寄存器初始化
	RCC_DeInit();
	//确保LSE关闭，释放PC14/PC15为GPIO(不使用RTC)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_LSEConfig(RCC_LSE_OFF);
	//使能外部高速晶振HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//等待外部高速晶振使能完毕，完毕后给HSEStartUPStatus返回success
	HSEStartUPStatus = RCC_WaitForHSEStartUp();//RCC_WaitForHSEStartUp()是等待外部高速晶振使能完毕函数
	if(HSEStartUPStatus == SUCCESS){
		//设置PLL时钟源和倍频系数，时钟源给一个固定频率，影响运行速率，PLL可以把这个频率抬高，也可以调低
		//第一个参数Divx,x就是频率除以多少，第二个参数PLLMul_x,x就是频率乘以多少
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
		//设置AHB时钟（HCLK）,在RCC中的寄存器进行配置，也叫AHB预分频器，Divx中的x就是总线频率除以几，上一步已经设置总线频率为8*9=72MHz了。
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		//设置AHB低速时钟（PCLK1），让其频率除以2，也就是APB1时钟 = 36MHz
		//APB1时钟通常控制TIM2~TIM7,TIM12~TIM14，I2C1,I2C2,SPI2,SPI3,CAN1,CAN2,USB等等
		RCC_PCLK1Config(RCC_HCLK_Div2);
		//设置AHB高速时钟（PCLK2）,让其频率除以1，也就是APB2时钟 = 72MHz
		//APB2时钟通常控制所有的GPIO口，AFIO,TIM1,TIM8
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//配置flash存储器的等待周期，及cpu等待几个周期之后去读取flash里面的程序
		//0—24MHz时，取Latency_0；不用等   
		//24—48MHz时，取Latency_1；1个周期   
		//48~72MHz时，取Latency_2；2个周期   
		FLASH_SetLatency(FLASH_Latency_2);
		//给flash存储器创造一个缓冲器，还没有存进flash的程序现金缓冲区，这样当这一条程序被读完了之后可以立马读下一条
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		//使能PLL
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //等待PLL输出稳定   
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //选择SYSCLK时钟源为PLL，也就是72MHz
		while(RCC_GetSYSCLKSource() != 0x08); //等待PLL成为SYSCLK时钟源,上一步刚刚切换系统时钟是在SW位（switch）上切换的，需要等待切换完成，0x08是PLL时钟的SWS位（switch status）,0x04是HSE,0x00是HSI
	} 	
}