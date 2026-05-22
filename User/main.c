#include <ti/eeprom/emulation_type_a/eeprom_emulation_type_a.h> 
#include "ti_msp_dl_config.h"
#include "bsp.h"

#define wave_data_len 250*4*3  //该范围内用8.7k的adc采集，2khz的波26次采集/周期波形,则可以采集至少4*3个周期波形，两个周期波形用来计算参数，另外两个周期波形用来验证波形
#define adc_frequence 24540 //adc的采集频率  
#define wave_error 2 //允许的函数比较误差，设置为2%
#define wave_point_error 5  //采集最大值之间最少间隔
#define wave_dis 80 //波形判断时用
#define wave_vale_big 90 //判断波形的最大值时用
#define wave_vale_small 110 //判断波形的最小值时用
#define wave_period_record_len 12
#define wave_duty_record_len 12

float wave_duty_record_arr[wave_duty_record_len];
uint16_t wave_period_record_arr[wave_period_record_len];
uint8_t wave_show=0;//显示波形

float wave_period=0;//波的周期
float wave_frequence=0;//波的频率
float wave_offset=0;//波得偏置
float wave_amp=0;//波的振幅

float wave_amp_x=0; 
float wave_offset_x=0;

float wave_vpp=0;//波的峰峰值
float wave_vrms=0;//波的有效值
float wave_square_duty=0;//矩形波的占空比

uint16_t wave_max=0,wave_min=0; 
uint16_t wave_data_left=0,wave_data_right=0;//wave_data 的左右值

uint16_t wave_data_max[4*3]={0};//记录wave_data内的四组最大值
uint16_t wave_data_max_count=0;
uint16_t wave_data_max_index[4*3]={0};//记录对应数据的下标

uint16_t wave_data_min[4*3]={0};//记录wave_data内的四组最小值
uint16_t wave_data_min_count=0;
uint16_t wave_data_min_index[4*3]={0};//记录对应数据的下标

uint8_t maybe_square=0;//测量方波时防止信号抖动导致最大值或最小值数组提前塞满

uint16_t wave_data[wave_data_len];//存储采集的adc数据
uint16_t wave_data_count=0;//记录采集的数据个数
uint8_t wave_kind_sign=0;//未识别出波形
uint8_t adc_load=0;//adc数据采集成功标志

float my_sin_x[wave_data_len];
uint16_t my_sin_x_count=0;


uint8_t my_start=0;
uint8_t my_big_index_one=0;
uint8_t my_big_index_two=0;
uint8_t my_small_index=0;
uint16_t period_t=0;
uint16_t my_big=0;
uint16_t my_small=0;

void wave_kind(uint8_t* wave_sign);

int main()
{ 
	 SYSCFG_DL_init();
	 OLED_Init();
	 OLED_ShowString(0,0,"Waiting...");
   
	 NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
   DL_ADC12_enableInterrupt(ADC12_0_INST,DL_ADC12_IIDX_MEM0_RESULT_LOADED);
	 
   for(int k=0;k<wave_data_len;k++)	
	 {
	   my_sin_x[k]=(sin(4*3.14159/wave_data_len*k)+1.2)/3.3*4096;
	 }
	 
	 while(1)
	 {
		 	DL_ADC12_startConversion(ADC12_0_INST);
	    while(!adc_load)
      {
//				OLED_Clear();
//				OLED_ShowString(0,0,"Waiting...");
			}
		  adc_load=0;//重置
			
			if(DL_GPIO_readPins(SW_SW2_PORT,SW_SW2_PIN))
			{
			  wave_show=0;
			}
			else
			{
			  wave_show=1;//允许显示波形
			}
			
//		if(wave_show)
			{ 
				if(wave_data_count<wave_data_len)
				{
					wave_data[wave_data_count++]=DL_ADC12_getMemResult(ADC12_0_INST,DL_ADC12_MEM_IDX_0);//加载数据
				}
				else
				{
				   for(int i=0;i<wave_data_len-1;i++)
					 {
					    wave_data[i]=wave_data[i+1];
						  my_sin_x[i]=my_sin_x[i+1];
					 }
					 wave_data[wave_data_len-1]=DL_ADC12_getMemResult(ADC12_0_INST,DL_ADC12_MEM_IDX_0);//加载数据
					 my_sin_x[wave_data_len-1]=(sin(4*3.14159/wave_data_len*my_sin_x_count++)+1.2)/3.3*4096;
					 if(my_sin_x_count>=wave_data_len)
					 {
						 my_sin_x_count=0;
					 }
					 OLED_Clear();
					 
					 for(int i=0;i<128;i++)
					 {
						 OLED_DrawPoint(i,36,1);
					 }
					 for(int j=0;j<64;j++)
					 {
					   OLED_DrawPoint(63,j,1);
					 }
					 
					 uint16_t wave_aver=0;
					 my_big=wave_data[0];
           my_small=wave_data[0];
					 
           for(int i=1;i<wave_data_len;i++)
           {
					   my_big=(my_big>wave_data[i])?my_big:wave_data[i];
						 my_small=(my_small<wave_data[i])?my_small:wave_data[i];
					 }
					 wave_aver=0.5*(my_big+my_small);
					 
					 u8 y=0;
					 
					 for(int k=0;k<128;k++)
					 {
						 y=36-(wave_data[k]-wave_aver)*1.0/4096.0*3.3*20;
					   OLED_DrawPoint(k,y,1);  
//					   OLED_DrawPoint(k,36+(my_sin_x[k]/4096*3.3-1.2)*12,1);  
					 }
					 delay_cycles(sysosc*0.02);
				}
				DL_ADC12_disableConversions(ADC12_0_INST);
//      delay_cycles(sysosc*0.002);
        DL_ADC12_enableConversions(ADC12_0_INST);
			}
//		else if(!wave_show)//采集结束，对采集的数据进行处理
			
			if(!wave_show&&(wave_data_count>=wave_data_len))//采集结束，对采集的数据进行处理
			{
				
//				 for(int i=0;i<wave_data_len;i++)
//				 {
//				   wave_data[i]=my_sin_x[i];
//				 }
				 
				 wave_data_max_count=0;
				 wave_data_min_count=0;
			   wave_data_count=0;//重置
				 
				 wave_max=wave_data[1];
				 wave_min=wave_data[1];
				
				 uint16_t max_val=wave_data[0];//记录最大值
				 uint16_t min_val=wave_data[0];//记录最小值
				
				 for(int i=1;i<wave_data_len-1;i++)//获取周期波形的4*3组最大最小值
				 {
					  wave_data_left=wave_data[i-1];
					  wave_data_right=wave_data[i+1];
					   	 
					  if((wave_data_left<=wave_data[i])&&(wave_data_right<=wave_data[i]))//局部最大值
						{		 			
							  if(wave_data_max_count&&(wave_data[i]-max_val)*1.0/max_val*100>wave_error)//防止采集抖动出现次高,若变化幅度大于error%，则认为是次高，需要更新
						  	{
									max_val=(wave_data[i]>max_val)?wave_data[i]:max_val;//更新已经记录到的最大值	
									wave_data_max[0]=wave_data[i];//若有3%的误差，说明前面的高点为假高点，舍去，需要重新采集
									wave_data_max_index[0]=i;
									wave_data_max_count=1;
								}
								else if(wave_data[i]*1.0/max_val*100>=wave_vale_big)//不是次高,而是新的最大值,同时排除全局较小值
								{
								  wave_data_max[wave_data_max_count]=wave_data[i];
 									wave_data_max_index[wave_data_max_count]=i; 
									if((wave_data_max_count<4*3)&&(!wave_data_max_count)||(wave_data_max_count>=1&&((wave_data_max_index[wave_data_max_count]-wave_data_max_index[wave_data_max_count-1])>wave_point_error)))//新的最大值有效的前提
									{
										wave_data_max_count++;
									}
								}
						}
						else if((wave_data_left>=wave_data[i])&&(wave_data_right>=wave_data[i]))//局部最小值
						{
							  if(wave_data_min_count&&(-wave_data[i]+min_val)*1.0/min_val*100>wave_error)//防止采集抖动出现次低,若变化幅度大于error%，则认为是次低，需要更新
						  	{
									min_val=(wave_data[i]<min_val)?wave_data[i]:min_val;//更新已经记录到的最小值	
									wave_data_min[0]=wave_data[i];//若有3%的误差，说明前面的低点为假低点，舍去，需要重新采集
									wave_data_min_index[0]=i;
									wave_data_min_count=1;
								} 
								else if(wave_data[i]*1.0/min_val*100<=wave_vale_small)//不是次低，而是新的最小值，同时排除全局较大值
								{
								  wave_data_min[wave_data_min_count]=wave_data[i];
									wave_data_min_index[wave_data_min_count]=i;  
									if((wave_data_min_count<4*3)&&(!wave_data_min_count)||(wave_data_min_count>=1&&((wave_data_min_index[wave_data_min_count]-wave_data_min_index[wave_data_min_count-1])>wave_point_error)))//新的最小值有效的前提
									{
										wave_data_min_count++;
									}
								}
						}
				 }
				 
         				 

				 
			   wave_period=0;//重置周期为0
				 wave_frequence=0;//波的频率
				 wave_offset=0;//波的偏置
				
				 wave_vpp=0;//波的峰峰值
         wave_vrms=0;//波的有效值
	
				 
				 for(int j=0;j<2;j++)
				 {
					 for(int i=0;i<4*3;i++)
				  {
						if(!j)
				    {
							wave_offset+=wave_data_min[i];
						  wave_vpp-=wave_data_min[i];
						}
						else
						{
							wave_offset+=wave_data_max[i];
							wave_vpp+=wave_data_max[i];
						}
				  }
				 }
				 wave_offset=wave_offset/(4.0*3)/2.0;//波的直流偏置
				 wave_vpp=wave_vpp/(4.0*3);//波的峰峰值
				
				 wave_amp=0;
				 for(int p=0;p<4*3;p++)
				 {
					 wave_amp+=wave_data_max[p];
				 }
				 wave_amp/=(4.0*3);
				 wave_amp-=wave_offset;//波的振幅
				 wave_offset_x=wave_offset;//记录偏移值
				 wave_offset=wave_offset/4096*3.3;//波的直流偏置
				 
				 wave_amp_x=wave_amp;//整数形式的波振幅
				 
				 wave_amp=wave_amp/4096*3.3;//波的振幅
				 wave_vpp=wave_vpp/4096*3.3;//波的峰峰值
				 
				 
		 my_big=0;
		 my_small=0;
		
		my_big=wave_data[0];
		my_small=wave_data[0];
		
		for(int i=1;i<wave_data_len;i++)
    {
			my_big=(my_big>wave_data[i])?my_big:wave_data[i];//找到最大最小值
			my_small=(my_small<wave_data[i])?my_small:wave_data[i];//找到最大最小值
		}		
		
		 my_start=0;
		 my_big_index_one=0;
		 my_big_index_two=0;
		 my_small_index=0;
		 period_t=0;
			
		for(int r=0;r<wave_period_record_len;r++)
		{
		  wave_period_record_arr[r]=0;
		}
		
		
		for(int r=0;r<wave_period_record_len;r++)
		{
			for(int i=8+period_t;i<wave_data_len;i++)
			{
				if(wave_data[i]*1.0/my_big*100>=wave_vale_big)//与最大值误差在3以内，注意先测的最大值
				{
					if(my_start&&my_small_index)//已经记录完第一个最大值和最小值
					{
						my_big_index_two=i;
						wave_duty_record_arr[r]=(my_small_index-my_big_index_one)*1.0/(my_big_index_two-my_big_index_one);
						wave_period_record_arr[r]=my_big_index_two-my_big_index_one;//存储周期
						
						wave_data_max_index[r]=my_big_index_one;//记录下标
						wave_data_min_index[r]=my_small_index;
						
						period_t+=wave_period_record_arr[r];
						
						my_start=0;
						my_big_index_one=0;
						my_big_index_two=0;
						my_small_index=0;
						
						break;
					}
					my_start=1;//找到第一个最大值下标
				}
				
				if(my_start)
				{
					if(!my_big_index_one)
					my_big_index_one=i;//记录第一个最大值下标
					
					if((wave_data[i]*1.0/my_small*100<=wave_vale_small)&&((i-my_big_index_one)>=wave_point_error)&&(!my_small_index))
					{
						 my_small_index=i;//记录第一个最小值下标
					}
				}
			}
    }

		
		float wave_frequence_base=0,wave_period_base=0,wave_duty_x=0;
		
//	wave_period_base=0.5*(my_big_index_two-my_big_index_one)*1.0/adc_frequence;//为匹配实验凑合0.5的因子
		
		wave_period_base=0;
		uint16_t x_max=wave_period_record_arr[0],x_min=wave_period_record_arr[0];
		
		
		for(int i=1;i<wave_period_record_len;i++)
		{
			x_max=(x_max>wave_period_record_arr[i])?x_max:wave_period_record_arr[i];
			x_min=(x_min<wave_period_record_arr[i])?x_min:wave_period_record_arr[i];
		}
		
		uint8_t x_mid=0;//标志是否有值处于最大最小之间
		uint8_t x_min_count=0,x_count_max=0;//没有中间项，则统计最大最小各自个数
		
		for(int i=1;i<wave_period_record_len;i++)
		{
			if((wave_period_record_arr[i]>x_min&&wave_period_record_arr[i]<x_max))//中间项判断
			{
			   x_mid=1;
				 break;
			}
			
			if(wave_period_record_arr[i]==x_min)
			{
			   x_min_count++;
			}
			else if(wave_period_record_arr[i]==x_max)
			{
			   x_count_max++;
			}
		}	
		
		uint8_t n_count=0;
		
		for(int i=0;i<wave_period_record_len;i++)
		{
			if(x_mid&&(wave_period_record_arr[i]!=x_max)&&(wave_period_record_arr[i]!=x_min))//有中间值，可去除最大最小值
			{
				wave_duty_x+=wave_duty_record_arr[i];
				wave_period_base+=wave_period_record_arr[i];
				n_count++;
			}
			else //没有中间项
			{
				if((x_min_count>x_count_max)&&(wave_period_record_arr[i]==x_min))//保留最小值
				{
				   wave_duty_x+=wave_duty_record_arr[i];
				   wave_period_base+=wave_period_record_arr[i];
				   n_count++;
				}
				else if((x_min_count<x_count_max)&&(wave_period_record_arr[i]==x_max))//保留最大值
				{
				   wave_duty_x+=wave_duty_record_arr[i];
				   wave_period_base+=wave_period_record_arr[i];
				   n_count++;  
				}
			}
		}
		
		if(n_count>0)
		{
			wave_period_base=0.5*wave_period_base*1.0/n_count*(1.0/adc_frequence);
	    wave_square_duty=wave_duty_x*1.0/n_count;//得到占空比
		}
		else
		{
		  wave_period_base=0;
	    wave_square_duty=0;//得到占空比
		}
		
		if(wave_period_base>0)
		wave_frequence_base=1.0/wave_period_base;
		else
		wave_frequence_base=0;
		
		wave_period=wave_period_base;
		wave_frequence=wave_frequence_base;
					 				 				 
//					 for(int k=0;k<4*3/2.0;k++)
//				   {
//				    wave_period+=wave_data_min_index[k+6];
//						wave_period-=wave_data_min_index[k];
//						 
//						wave_period+=wave_data_max_index[k+6];
//						wave_period-=wave_data_max_index[k+6];
//				   }
//					 wave_period=wave_period/72*(1.0/adc_frequence);//波的周期
//					 if(wave_period>0)
//					 wave_frequence=1.0/wave_period;//波的频率
//					 else
//					 wave_frequence=0;
			 
		
  		//波种类判断
			wave_kind(&wave_kind_sign);//判断波形
				 
			switch(wave_kind_sign)//根据不同的波形计算有效值
			{
			  case 1://正弦波有效值处理
				{

					wave_vrms= wave_amp/1.414+wave_offset;
				  break;
				}
				case 2://三角波有效值处理
			  {
					float wave_k=0;
					wave_k=wave_amp*1.0/(wave_period/4.0);//波形斜率
					wave_vrms=sqrt(4.0/(3*wave_k*wave_period)*(pow(wave_offset+wave_k*wave_period/4.0,3)-pow(wave_offset,3)));
				  break;
				}
				case 3://矩形波有效值处理
				{
					wave_vrms=sqrt(pow(wave_amp+wave_offset,2)*wave_square_duty+pow(-wave_amp+wave_offset,2)*(1-wave_square_duty));
					break;
				}
				default:
				{
					wave_vrms=0;
					break;
				}
			}			
			
			OLED_Clear();
			switch(wave_kind_sign)
			{
				case 1:
				{
					OLED_ShowString(0,0,"Sine:");
					
					break;
				}
				case 2:
				{
				  OLED_ShowString(0,0,"Triangle:");
					break;
				}
				case 3:
				{ 
					OLED_ShowString(0,0,"Square:");
					 break;
				}
				default:
				{
					OLED_ShowString(0,0,"unknow:");
					break;
				}
			}
			
			if(DL_GPIO_readPins(SW_SW1_PORT,SW_SW1_PIN))
			{
				OLED_ShowString(0,2,"T:");
				OLED_show_float(0+8*2,2,wave_period*1e3,3);
				OLED_ShowString(0+8*2+8+8*3+8*3,2,"ms");
				
				OLED_ShowString(0,4,"F:");
				if(wave_frequence)
				{
					OLED_show_float(0+8*2,4,wave_frequence,1);
				  OLED_ShowString(0+8*10+8*3+8,4,"hz");
				}
				else
				{
					OLED_ShowString(0+8*2,4,"error");//频率为0
				}
				
				OLED_ShowString(0,6,"offset:");
				OLED_show_float(0+8*7+8,6,wave_offset,3);
				OLED_ShowString(0+8*7+8+8*3+8,6,"v");
				
			}
			else
			{
				if(wave_kind_sign!=3)
				{
					OLED_ShowString(0,2,"amp:");
					OLED_show_float(0+8*4,2,wave_amp,3);
					OLED_ShowString(0+8*10+8+8*3+8,2,"v");
					
					OLED_ShowString(0,4,"vpp:");
					OLED_show_float(0+8*4,4,wave_vpp,3);
					OLED_ShowString(0+8*10+8+8*3+8,4,"v");
					
					OLED_ShowString(0,6,"vrms:");
					OLED_show_float(0+8*5,6,wave_vrms,3);
					OLED_ShowString(0+8*10+8+8*3+8,6,"v");
				}
				if(wave_kind_sign==3)//矩形波显示占空比
				{
					OLED_ShowString(0,0,"amp:");
					OLED_show_float(0+8*4,0,wave_amp,3);
					OLED_ShowString(0+8*10+8+8*3+8,0,"v");
					
					OLED_ShowString(0,2,"vpp:");
					OLED_show_float(0+8*4,2,wave_vpp,3);
					OLED_ShowString(0+8*10+8+8*3+8,2,"v");
					
					OLED_ShowString(0,4,"vrms:");
					OLED_show_float(0+8*5,4,wave_vrms,3);
					OLED_ShowString(0+8*10+8+8*3+8,4,"v");
					
				   OLED_ShowString(0,6,"duty:");
				   OLED_show_float(0+8*5,6,wave_square_duty,3);//保留两位小数
				}
				
			}
			
			}
//			delay_cycles(sysosc);
	 }
}	

void wave_kind(uint8_t* wave_sign)//判断波形
{
  //判断为正弦波,编号为1
	//判断为三角波，编号为2
	//判断为矩形波，编号为3
	
	wave_kind_sign=0;//重置
	
	uint16_t data_index_min=0;
	uint16_t data_index_max=0;
	uint16_t data_index_mid=0;
	
	uint8_t sin_ok=0;//没判断对一次就+1
	uint8_t tri_ok=0;
	uint8_t square_ok=0;
	uint8_t unknow_ok=0;
	
	float line_k=0;//变化斜率
	uint16_t wave_sin_maybe=0;//斜率预测时，可能性增加
	uint16_t wave_sin_maybe_x=0;
	float wave_line_predict_value=0;//依据斜率预测值
	float w_error=0;//预测记录误差   

	for(int m=2;m<4*3-3;m++)//遍历下标数组一部分
	{
		
	data_index_min=wave_data_max_index[m];//注意先采集的最大值
	data_index_max=wave_data_min_index[m];//圈定最大和最小值分析
		
	data_index_mid=(data_index_min+data_index_max)/2.0;
	
	wave_offset_x=(wave_data[data_index_min]+wave_data[data_index_max])*0.5;
	wave_amp_x=(wave_data[data_index_min]-wave_data[data_index_max])*0.5;
		
	line_k=(wave_data[data_index_min]-wave_data[data_index_max])/(data_index_max-data_index_min);//变化斜率
	line_k=(line_k>0)?line_k:(-line_k);
	
	float wave_distance=wave_data[data_index_min]-wave_data[data_index_max-5];//最大值与最小值的差距
	wave_distance=(wave_distance>0)?wave_distance:(-wave_distance);
	
	float wave_new_distance=0;
			
	if(wave_distance*1.0/(wave_amp_x*2)*100<10)//可能是矩形波
	{
	  for(int i=data_index_min+2;i<data_index_mid;i++)//遍历1/4区间数据
		{
		   wave_new_distance=wave_data[i]-wave_data[data_index_mid];
			 wave_new_distance=(wave_new_distance>0)?wave_new_distance:(-wave_new_distance);
  		 if(wave_new_distance*1.0/(wave_amp_x)*100>wave_dis)//峰峰值比较,变化小，为矩形波
			 {
				 unknow_ok++;
			   break;
			 }	
		}
		square_ok++;
	}	

else if((wave_distance*1.0/(wave_amp_x*2)*100>=30))//正弦波或者三角波
{
	
//		float line_error=0;//累加估计误差
//		float sin_error=0;//累加估计误差
//		
//		float line_predict=0;
//		float sin_predict=0;
//		
//		for(int i=data_index_min+2;i<data_index_mid;i++)//遍历1/4区间数据，自变量为下标
//		{
//		  line_predict=wave_data[data_index_min]-line_k*(i-data_index_min); 
//			line_error+=(line_predict-wave_data[i])*(line_predict-wave_data[i]);//直线误差的平方
//			
//			sin_predict=wave_amp_x*sin((data_index_mid-i)*1.0/(data_index_mid-data_index_min))+wave_offset_x;//正弦误差的平方
//			sin_error+=(sin_predict-wave_data[i])*(sin_predict-wave_data[i]);
//		}
//		
//		if(sin_error<line_error)//正弦拟合得更好,正弦波
//		{
//		   sin_ok++;
//		}
//		else if(sin_error>line_error)//直线拟合得更好，三角波
//		{
//		   tri_ok++;
//		}


     my_start=0;
		 my_big_index_one=0;
		 my_big_index_two=0;
		 my_small_index=0;
		 period_t=0;

     uint16_t er=0;
		 
			for(int i=8;i<wave_data_len;i++)
			{
				er=-wave_data[i]+my_big;
		    
				if(er<=5)//与最大值误差在3以内，注意先测的最大值
				{
					if(my_start&&my_small_index)//已经记录完第一个最大值和最小值
					{
						my_big_index_two=i;	
						break;
					}
					my_start=1;//找到第一个最大值下标
				}
				
				if(my_start)
				{
					if(!my_big_index_one)
					my_big_index_one=i;//记录第一个最大值下标
					
					er=wave_data[i]-my_small;
					if(er<=5&&((i-my_big_index_one)>=wave_point_error)&&(!my_small_index))
					{
						 my_small_index=i;//记录第一个最小值下标
					}
				}
			}
     
			float x_k=0;
      x_k=(my_big-my_small)*1.0/(my_small_index-my_big_index_one);//斜率标准


     float k1=wave_data[my_big_index_one]-wave_data[my_big_index_one+1];//计算顶点斜率
		 k1=(k1>0)?k1:(-k1);
		 
		 float k2=wave_data[my_big_index_one]-wave_data[my_big_index_one-1];
		 k2=(k2>0)?k2:(-k2);
		 
		 if(k1>k2)
		 {
			 float temp=0;
			 temp=k1;
			 k1=k2;//确保k1为小斜率
			 k2=temp;//确保k2为大斜率
		 }
		 
		 
			if((k2*1.0/x_k*100)>60)//在顶点处斜率来判断
     {
		    tri_ok++;
		 }
		 else if((k1*1.0/x_k*100)<20)
		 { 
		    sin_ok++;
		 } 
}

}
	
	if((sin_ok>tri_ok)&&(sin_ok>square_ok)&&(sin_ok>unknow_ok))
	{
	  wave_kind_sign=1;
	}
	else if((tri_ok>sin_ok)&&(tri_ok>square_ok)&&(tri_ok>unknow_ok))
	{
	  wave_kind_sign=2;
	}
	else if((square_ok>tri_ok)&&(square_ok>sin_ok)&&(square_ok>unknow_ok))
	{
	  wave_kind_sign=3;		
	}
	else 
	{
	  wave_kind_sign=0;
	}
		
}


void ADC12_0_INST_IRQHandler(void)
{
   switch(DL_ADC12_getPendingInterrupt(ADC12_0_INST))
	 {
	   case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
		 {
			 adc_load=1;
			 DL_GPIO_togglePins(LED_LED1_PORT,LED_LED1_PIN);
			 break;
		 }
		 default:break;
	 }
  
}


