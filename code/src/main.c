/*********************************************************************
**
**                           Main.c
**
**
**********************************************************************/
/*********************************************************************
   Last committed:     2015-10-07
   Last changed by:    Mikael Sundin
   Last changed date:  2015-10-07
   ID:                 main.c

**********************************************************************/
#include "stm32f0xx_conf.h"

//number of ADC channels
#define ADC_COUNT           8

//number of samples to collect from every channel
#define ADC_NBR_OF_SAMPLES   8


volatile uint16_t adc_data[ADC_COUNT*ADC_NBR_OF_SAMPLES];

void init_adc(void)
{
    int i;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    ADC_StructInit(&ADC_InitStructure);
    GPIO_StructInit(&GPIO_InitStructure);

    ADC_DeInit(ADC1);
    DMA_DeInit(DMA1_Channel1);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_data[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = (ADC_COUNT*ADC_NBR_OF_SAMPLES);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //transfer complete interrupt

    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);


    /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC_InitStructure);

    //configure PA0-PA7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    for(i=0;i<ADC_COUNT;i++){
        ADC_ChannelConfig(ADC1, 1<<i , ADC_SampleTime_7_5Cycles);
        GPIO_InitStructure.GPIO_Pin = 1<<i;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    ADC_GetCalibrationFactor(ADC1);
    ADC_Cmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)){};

    //turn on DMA after calibration, else we got a extra DMA request
    ADC_DMACmd(ADC1, ENABLE);
}

void init_dcc_pin(){
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //PB1
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
}

//start a conversion on trigger
void EXTI0_1_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line1);
        ADC_StartOfConversion(ADC1);
    }
}

void DMA1_Channel1_IRQHandler(void){
  if (DMA_GetITStatus(DMA1_IT_TC1)){
    DMA_ClearITPendingBit(DMA1_IT_TC1);

  }

}

/**
 * Try to read one byte from usart
 * @return non zero on receive
 */
char read_usart(){
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET){
        return USART_ReceiveData(USART1);
    }else{
        return 0;
    }



}

void write_usart(char ch){
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){};
    USART_SendData(USART1, ch);
}

void writeline_usart(char* msg){
    while(*msg != 0){
        write_usart(*msg++);
    }
    write_usart('\n');
}

void init_usart(){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init (USART1, &USART_InitStructure);
    USART_Cmd (USART1, ENABLE);

    //configure alternative function for PA9, PA10 to UART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    //TX pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void nvic_init(){
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01; //0-3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01; //0-3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void init_hal(){
    //enable clocking of peripherals
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG | RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    init_adc();
    init_dcc_pin();
    init_usart();
}

uint16_t get_array_min(uint16_t *bfr, uint8_t offset, uint8_t count){
    uint16_t val = UINT16_MAX;

    while(count--){
        val = *bfr <= val ? *bfr : val;
        bfr += offset;
    }

    return val;
}

uint16_t get_array_max(uint16_t *bfr, uint8_t offset, uint8_t count){
    uint16_t val = 0;

    while(count--){
        val = *bfr >= val ? *bfr : val;
        bfr += offset;
    }

    return val;
}

uint16_t adc_min[ADC_COUNT];
uint16_t adc_max[ADC_COUNT];
int32_t adc_diff[ADC_COUNT];

int main(void)
{
    int32_t diff;
    int i;
    char ch;

    SystemInit();
    nvic_init();
    init_hal();

    writeline_usart("Online\n");

    while(1)
    {
        for(i=0;i<ADC_COUNT;i++){
            adc_min[i] = get_array_min((uint16_t*)&adc_data[i], ADC_COUNT, ADC_NBR_OF_SAMPLES);
            adc_max[i] = get_array_max((uint16_t*)&adc_data[i], ADC_COUNT, ADC_NBR_OF_SAMPLES);

            //lowpass iir filter
            diff = (adc_max[i] - adc_min[i]);
            adc_diff[i] = (adc_diff[i]*31 + diff)/32;
        }

        ch = read_usart();
        if(ch>0){
            write_usart(ch);
        }

        //TODO:calculate if we got a train on the track and output to uart
    }
}
