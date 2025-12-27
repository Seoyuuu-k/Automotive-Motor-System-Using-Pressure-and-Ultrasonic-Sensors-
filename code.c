//Automotive Motor System Using Pressure and Ultrasonic Sensors 

#include "stm32f767xx.h" 
#include "OK-STM767.h" 
#include "OK-STM767_large.h" 
 
volatile unsigned char distance_flag;       // 거리에 따른 인터럽트 
   unsigned int PWM;            // pulse width 
      float distance ; // 거리측정 
         
         
/* ----- 인터럽트 처리 프로그램 ----------------------------------------------- */ 
 
void SysTick_Handler(void)         /* SysTick interrupt function */ 
{ // 시스틱 타이머 인터럽트로 주기적으로 거리값 확인하여 플레그 값 변화시켜줌   
    if(distance< 15 ) {  
    distance_flag=0; } 
  else if(distance>=15 ){ 
    distance_flag=1; } 
} 
 
 
int main(void) 
{ 
   unsigned short result1; //ADC 압력값 
   unsigned short result2 ; //ADC 압력값 크기 늘리기 
 
 unsigned char key; 
 
 
  Initialize_MCU();            // initialize MCU and kit 
  Delay_ms(50);               // wait for system stabilization 
  Initialize_LCD();            // + initialize text LCD module  
  Initialize_TFT_LCD();            // initialize TFT-LCD module 
       
  Beep(); 
 
  SysTick->LOAD  = 1349999;         // 27MHz/(1349999+1) = 20Hz // 인터럽트 주파수 20HZ이다   
  SysTick->VAL   = 0;            // clear SysTick Counter 
  SysTick->CTRL  = 0x00000003;         // 216MHz/8 = 27MHz, enable SysTick and interrupt 
 
   
   PWM = 10;               // initialize PWM = 10 (duty ratio 1%) 
 
 
    GPIOE->MODER &= 0x0000FFFF;  // GPIOE 8~14 설정을 위해 비트 클리어  
    GPIOE->MODER |= 0x18850000;   
   /* 
   9 - trigger - output 
   10 - eco - input 
   11 - timer(PWM) - alternate funtion mode  
    13 - timer(PWM) - alternate funtion mode 
    14 - VCC - output 
   */ 

    RCC->APB1ENR |= 0x00000001;  // enable TIM2 clock 
    TIM2->PSC = 54 - 1;  // PSC 설정 (1us) 
    TIM2->CR1 = 0x00000001;  // counter enable 
    TIM2->CNT = 0;  // 카운터 초기화 
     
    GPIOE->AFR[1] &= 0xFF0F0FFF;         // PE13, PE11을 AF1로 사용하여 타이머 사용 
    GPIOE->AFR[1] |= 0x00101000; 
   
    GPIOE->ODR |= 0x00004000;  // PE14 - VCC 
 
    RCC->APB2ENR |= 0x00000001;         // enable TIM1 clock 
 
    TIM1->PSC = 107;            // 108MHz/(107+1) = 1MHz ,프리스케일 설정 
    TIM1->ARR = 999;            // 1MHz / (999+1) = 1kHz , ARR 값 설정 
    TIM1->CCR2 = PWM;          // 초기화에선 둘다 duty ratio 1% 들어가도록 설정해서 멈춰 있도록 
    TIM1->CCR3 = PWM; 
    TIM1->CNT = 0;            // clear counter 
 
   
    TIM1->CCMR1 = 0x00006C00; // 채널 2 , 외부트리거입력 사용X, PWM모드1, Output Compare 2 preload enable , Output Compare 2 fast 
enable,  CC2 channel is configured as outpu  
    TIM1->CCMR2 = 0x0000006C; // 채널 3 
   
  
    TIM1->CCER = 0x00000110;         // CC2,3E = 1(enable OC2,3 output) 
    TIM1->BDTR = 0x00008000;         // MOE = 1 
    TIM1->CR1 = 0x0005;            // edge-aligned, up-counter, enable TIM1 
 
 
    GPIOA->MODER |= 0x0000C000; // 7포트 아날로그 모드로 변환 
    RCC->APB2ENR |= 0x00000100; // ADC클락 활성화 
    ADC->CCR = 0x00000000;      // ADCCLK = 54MHz/2 = 27MHz 
    ADC1->SMPR2 = 0x00040000;       // sampling time of channe7=15 cycle 
    ADC1->CR1 = 0x00000000;         // 12-bit resolution 
    ADC1->CR2 = 0x00000001;         // right alignment, single conversion, ADON = 1 
    ADC1->SQR1 = 0x00000000; //total regular channel number = 1  
 
     
    while (1) 
    { 
         
 key = Key_input();         // key input 
  
 
 
      if((key == KEY1) && (PWM < 999))   // if KEY1 , 전진 
      { 
        while(1){   
 
        GPIOE->BSRR |= 0x00000200;  // trigger High 
        Delay_us(12);  // trigger 지속 

            GPIOE->BSRR |= 0x02000000;  // trigger low 
 
        while (!(GPIOE->IDR & 0x0400));  // eco high 기다리기  
 
        TIM2->CNT = 0;  // 타이머 카운터 초기화 
 
        while (GPIOE->IDR & 0x0400);  // eco low 대기  
 
        distance = (TIM2->CNT) / 5800.;  // 음속 약 340m/s, cm로 변환 
      
 
        Delay_ms(200);  // 200ms 대기 
 
        TFT_clear_screen();  // TFT LCD 화면 클리어 
 
 
TFT_xy(11, 16);  // TFT LCD의 (11, 16) 위치로 커서 이동 
        TFT_color(Cyan, Black);  // 글자 색은 Cyan, 배경색은 Black으로 설정 
        TFT_signed_float((float)distance_flag, 6, 1);  // 거리 플레그를 출력 (6자리, 소수점 아래 1자리) 
         
        TFT_xy(11, 19);  // TFT LCD의 (10, 28) 위치로 커서 이동 
        TFT_color(Cyan, Black);  // 글자 색은 Cyan, 배경색은 Black으로 설정 
        TFT_signed_float((float)distance, 6, 1);  // 측정한 거리를 출력 (6자리, 소수점 아래 1자리) 
 
 
           
          ADC1->SQR3 = 0x00000007;   // + channel 7 
        ADC1->CR2 |= 0x40000000;         // start conversion by software 
        while (!(ADC1->SR & 0x00000002));      // wait for end of conversion 
        result1 = ADC1->DR; // adc센싱값 
 
        result2 = result1 * 3 ; // PWM으로 사용하기 위해 adc센싱값 값키워줌 
         
         
           if(distance_flag==1) { // 정방향 전진 
           
          PWM = 200 + result2 ; 
          TIM1->CCR2  = PWM; 
          TIM1->CCR3  = 0 ; 
   } 
            
        else if(distance_flag==0) { // 멈춤 
           
          TIM1->CCR2  = 0 ; 
          TIM1->CCR3  = 0 ; 
          } 
 
           Delay_ms(200);  
            
            key = Key_input();    
         if( key == KEY2 ) break; // 후진기어쪽으로 한칸 당김 
          
        } 
         
          TFT_clear_screen();  // TFT LCD 화면 클리어 
 
        } 
       
       
       
      else if((key == KEY2) && (PWM < 999))   // if KEY2 , 후진 
        {  
           while(1){   
           
          ADC1->SQR3 = 0x00000007;   // + channel 7 
        ADC1->CR2 |= 0x40000000;         // start conversion by software 
        while (!(ADC1->SR & 0x00000002));      // wait for end of conversion 
        result1 = ADC1->DR; 
 
        result2 = result1 * 3 ; // 값키워줌 
         
          PWM = 200 + result2 ; 
          TIM1->CCR2  = 0; 
          TIM1->CCR3  = PWM; 
          Delay_ms(200); 
           
           key = Key_input();    
           if( key == KEY1 ) break; //전진기어쪽으로 한칸 당김 
       } 
   }      
} 
}