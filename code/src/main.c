/*********************************************************************
**
**                           Main.c
**
**
**********************************************************************/
/*********************************************************************
   Last committed:     2015-10-07
   Last changed by:    Mikael Sundin
   Last changed date:  2015-10-23
   ID:                 main.c
   TODO:               Clean-up and split into different files.

**********************************************************************/
#include "stm32f0xx_conf.h"
#include <stdint.h>
#include <stdbool.h>
#include "uart.h"
#include "array_fn.h"

#define SOFTWARE_NAME       "DCC Block detector"
#define SOFTWARE_VERSION    "1.0"


//hard-coded threshold for now, seems to work good and detect over 5mA.
#define DETECTOR_THRESHOLD  4

//number of ADC channels
#define ADC_COUNT           8

//number of samples to collect from every channel
#define ADC_NBR_OF_SAMPLES   8

//private functions
static void init_hal(void);
static void nvic_init(void);
static void init_dcc_pin(void);
static void init_adc(void);

const char *build_str = "#" SOFTWARE_NAME " Version: " SOFTWARE_VERSION " " __DATE__ " " __TIME__;
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

void init_dcc_pin(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //PB1 used to sync to dcc signal.
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

//start a conversion on DCC trigger
void EXTI0_1_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line1);
        ADC_StartOfConversion(ADC1);
    }
}

void nvic_init(void){
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01; //0-3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void init_hal(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG | RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    init_adc();
    init_dcc_pin();
    init_usart();
}



volatile bool update_flag=false;

void SysTick_Handler(void)  {
    static uint32_t cnt=0;

    if(cnt++ >= 250){
        cnt = 0;
        update_flag = true;
    }
}

//seems that we need ~5mA to detect anything on the track
int main(void)
{
    int i;
    uint8_t current_state=0;
    uint8_t last_state=0;
    int32_t tmp_diff;
    uint16_t adc_min[ADC_COUNT];
    uint16_t adc_max[ADC_COUNT];
    int32_t adc_diff[ADC_COUNT];

    SystemInit();
    nvic_init();
    init_hal();
    SysTick_Config(SystemCoreClock / 1000); //1ms interrupt

    //print software version
    print_ln((char*)build_str);

    while(1)
    {
        current_state = 0;
        for(i=0;i<ADC_COUNT;i++){
            adc_min[i] = get_array16_min((uint16_t*)&adc_data[i], ADC_COUNT, ADC_NBR_OF_SAMPLES);
            adc_max[i] = get_array16_max((uint16_t*)&adc_data[i], ADC_COUNT, ADC_NBR_OF_SAMPLES);

            //lowpass iir filter on difference.
            tmp_diff = (adc_max[i] - adc_min[i]);
            adc_diff[i] = (adc_diff[i]*31 + tmp_diff)/32;

            //convert array to bitmask
            if(adc_diff[i] >= DETECTOR_THRESHOLD){
                current_state |= 1<<i;
            }

        }

        //periodically check current state
        if(update_flag){
            update_flag = 0;
            if(current_state != last_state){
                last_state = current_state;
                print_hex_ln(current_state);
            }
        }
    }
}
