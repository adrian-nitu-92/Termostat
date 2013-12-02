#include "stm32f10x_conf.h"
#include "extern.h"
#include "sys_time.h"
#include "main.h"

s32 sys_timeout;
s32 sys_timeout2;
s32 sys_time=0;
s32 sys_time2=0;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;

s32 icnt,tcnt;
s32 timeout;
s32 alive_time;
u16 ReadCount(void);

void DelayUs(s32 valu){
    int i;
    timeout = valu * 2000;
    tcnt = 0;
    icnt = ReadCount();
    for(i = 0; i< timeout; i++)
        {
        }
    icnt = ReadCount() - icnt;


}

u16 ReadCount(void){
    return TIM1->CNT;
}

void InitSysTick(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Period = 3999; // 1ms tick        
    TIM_TimeBaseStructure.TIM_Prescaler = 17; // -- 2MHz --      
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
    sys_time = 0;
    TIM_Cmd(TIM3, ENABLE);
}


void TimeoutSet(s32 timeout){
    sys_timeout = sys_time+timeout;
}

void TimeoutSet2(s32 timeout2){
    sys_timeout2 = sys_time2+timeout2;
}


u8 TimeoutExp(void){
    if (sys_time >= sys_timeout)
        return 1;
    return 0;
}

u8 TimeoutExp2(void){
    if (sys_time2 >= sys_timeout2)
        return 1;
    return 0;
}
void Sys_Alive(void){
    if(((sys_time%1000)==0) && (alive_time != sys_time)){
        GPIO_WriteBit(GPIOB,GPIO_Pin_12,!GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_12));
        alive_time = sys_time;
    }
}

void wasteTime(int in){
    TimeoutSet(in);
    while(TimeoutExp()==0);
}

void delay(int n){
int k;
   k=n;
   while( k >0)
	{   
   k=k-1;
   }
}