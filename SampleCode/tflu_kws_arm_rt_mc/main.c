/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an keyword spotting inference realtime V2. with TFLITE 
 *           matching I2S demo with PDMA function connected with audio codec.
 *           This example can change different model, for example DNN & DS_CNN
 *           in Model.h #define.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <vector>
#include <iostream>
using namespace std;


#include "KWS/kws.h"
#include "BufAttributes.h"

extern "C" {

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"

#include <arm_math.h>


#define NAU8822     1


static DMA_DESC_T DMA_TXDESC[2], DMA_RXDESC[2];

//PDMA1
static uint32_t audio_io_buffer1[BUFF_LEN*2];	
static uint32_t audio_buffer[BUFF_LEN];	

int16_t* AUDIO_BUFFER_IN;
int16_t* AUDIO_BUFFER_OUT;



//KWS
KWS *kws;
static volatile uint8_t s_uCount = 0;
static volatile uint8_t s_u8pdmaIRQReIn = 0;

static volatile uint8_t s_u8TxIdx = 0, s_u8RxIdx = 0;
static volatile uint8_t s_u8CopyData = 0;

void PDMA0_IRQHandler(void);
void SYS_Init(void);
void PDMA_Init(void);
void I2C2_Init(void);

#if NAU8822
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data);
void NAU8822_Setup(void);
#else
uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len);
uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat);
void NAU88L25_Reset(void);
void NAU88L25_Setup(void);
#endif

// 
volatile uint8_t u8Count=0;

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);
	
	  //printf("Start PDMA_IRQ, status=0x%x!!\n", u32Status);
		
    if (u32Status & 0x2) {
        if (PDMA_GET_TD_STS(PDMA0) & 0x2) {          /* channel 1 done */
            /* Copy RX data to TX buffer */
					  if(PDMA0->CURSCAT[1] == (uint32_t)&DMA_RXDESC[0])
					  {
                for (int i=0;i<BUFF_LEN;i++) {
									audio_buffer[i] = audio_io_buffer1[i];
                }
						}
					  else if(PDMA0->CURSCAT[1] == (uint32_t)&DMA_RXDESC[1])
					  {
                for (int i=0;i<BUFF_LEN;i++) {
									audio_buffer[i] = audio_io_buffer1[BUFF_LEN*1 + i*1];
                }
						}
        s_u8CopyData = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);	
						
        }
		
        if (PDMA_GET_TD_STS(PDMA0) & 0x4) {          /* channel 2 done */
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }
    }
		else
		{
			PDMA_CLR_ALIGN_FLAG(PDMA0, PDMA_ABTSTS_ABTIF1_Msk);
		}
 
}




#if NAU8822

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C2                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data)
{
    I2C_START(I2C2);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, (uint8_t)((u8Addr << 1) | (u16Data >> 8)));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, (uint8_t)(u16Data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_STOP(I2C2);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

#ifdef INPUT_IS_LIN   /* Input source is LIN */
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K */
    I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   /* Input source is MIC */
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
		
		I2C_WriteNAU8822(6,  0x001);   /* Divide by 1, FS and BCLK are driven as outputs, 16K */ // NAU8822 as Master
		I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    
		//I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K */
    //I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
		
		//I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K */
    //I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */

    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
}


#endif

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
		
		
		/*---------------------------------------------------------------------------------------------------------*/
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
		
}

void PDMA_Init1(void)
{
	
    /* Rx description */
    DMA_RXDESC[0].ctl = ((BUFF_LEN*2-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_16|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_RXDESC[0].src = (uint32_t)&I2S0->RXFIFO;
    DMA_RXDESC[0].dest = (uint32_t)(&audio_io_buffer1);
    DMA_RXDESC[0].offset = (uint32_t)&DMA_RXDESC[1] - (PDMA0->SCATBA);

    DMA_RXDESC[1].ctl = ((BUFF_LEN*2-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_16|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_RXDESC[1].src = (uint32_t)&I2S0->RXFIFO;
    DMA_RXDESC[1].dest = (uint32_t)(&audio_io_buffer1[BUFF_LEN*1]);
    DMA_RXDESC[1].offset = (uint32_t)&DMA_RXDESC[0] - (PDMA0->SCATBA);

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(PDMA0, 0x1 << 1);

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA0, 1, PDMA_I2S0_RX, 1, (uint32_t)&DMA_RXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA0, 1, 0);

    NVIC_EnableIRQ(PDMA0_IRQn);
}



/* Init I2C interface */
void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 100k */
    I2C_Open(I2C2, 100000);

    /* Get I2C2 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C2));
}


/****************************************************************************
 * InferenceJob
 ****************************************************************************/
/*const char outputClass[12][8] = {
        "Silence",
        "Unknown",
        "yes",
        "no",
        "up",
        "down",
        "left",
        "right",
        "on",
        "off",
        "stop",
        "go"};

const std::vector<std::string> outputClass_ans = {
            "yes",
            "no",
            "up",
            "down",
            "left",
            "right",
            "on",
            "off",
            "stop",
            "go"}; 
	*/					

const char outputClass[12][8] = {
        "Silence",
        "Unknown",
        "one",
        "two",
        "three",
        "four",
        "five",
        "six",
        "seven",
        "eight",
        "nine",
        "zero"};

const std::vector<std::string> outputClass_ans = {
            "one",
            "two",
            "three",
            "four",
            "five",
            "six",
            "seven",
            "eight",
            "nine",
            "zero"};  

/*
const char outputClass[12][8] = {
        "Silence",
        "Unknown",
        "one",
        "two",
        "three",
        "four",
	      "s_r"
        };

const std::vector<std::string> outputClass_ans = {
            "one",
            "two",
            "three",
            "four",
           };						
*/





/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                keyword spotting inference offline with tflite         |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with audio codec.\n");

    /* Init I2C2 to access codec */
    I2C2_Init();

#if (!NAU8822)
    /* Reset NAU88L25 codec */
    NAU88L25_Reset();
#endif

#ifdef INPUT_IS_LIN
    /* Open I2S0 interface and set to slave mode, stereo channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);
#else
    /* Open I2S0 interface and set to slave mode, mono channel, I2S format */
		
    I2S_Open(I2S0, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S);
#endif

    /* Set PD3 low to enable phone jack on NuMaker board. */
    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk);
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* Select source from HXT(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);

    /* Set MCLK and enable MCLK */
		//I2S_EnableMCLK(I2S0, 4096000);
    I2S_EnableMCLK(I2S0, 12000000);

#ifndef INPUT_IS_LIN
    /* NAU8822 will store data in left channel */
    I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_LEFT);
#endif

#if NAU8822
    /* Initialize NAU8822 codec */
    NAU8822_Setup();
#else
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif



    printf("\nThis sample code run keyword spotting inference\n");
    printf("KWS simple example; build timestamp: %s:%s\n", __DATE__, __TIME__);

	  	
		int recordingWin = 49;  //DSCNN, no need
		int averagingWindowLen = 1;
		int detectionThreshold = 40;
		
		/*
		int recordingWin = 25; //DNN, no need
		int averagingWindowLen = 1;
		int detectionThreshold = 20;
		*/
		
		KWS kws(averagingWindowLen, audio_buffer);

		

		PDMA_Init1();

    /* Enable I2S Rx function */
    I2S_ENABLE_RXDMA(I2S0);
    I2S_ENABLE_RX(I2S0);

    /* Enable I2S Tx function */
    I2S_ENABLE_TXDMA(I2S0);
    I2S_ENABLE_TX(I2S0);
		

    while(1)
    {
			   
        if(s_u8CopyData)
        {
					    //printf("=================================\n");
					    //printf("Extracting features.. \r\n");
              
							kws.ExtractFeatures();  // Extract MFCC features.
		    
              //printf("Classifying..\r\n");
              kws.Classify();  // Classify the extracted features.
		          
              int maxIndex = kws.GetTopClass(kws.output);
		          
              //printf("Detected %s (%d%%)\r\n", outputClass[maxIndex],
              //    (static_cast<int>(kws.output[maxIndex]*100)));
					
					    //printf("***  Averaging predictions.\r\n");
              kws.AveragePredictions();
					    int maxIndex_av = kws.GetTopClass(kws.averagedOutput);
					
					    if(kws.averagedOutput[maxIndex_av]*100 >= detectionThreshold) {
								//printf("**** Classified: %s (%d%%)\r\n", outputClass[maxIndex_av],
                //  (static_cast<int>(kws.averagedOutput[maxIndex_av]*100)));
							
								if(std::find(outputClass_ans.begin(), outputClass_ans.end(), outputClass[maxIndex_av]) != outputClass_ans.end()){
							      printf("Trigger %s\r\n", outputClass[maxIndex_av]);	
									  printf("**** Classified: %s (%d%%)\r\n", outputClass[maxIndex_av], (static_cast<int>(kws.averagedOutput[maxIndex_av]*100))); 
							      //PDMA_Close(PDMA0);
									  //TIMER_Delay(TIMER0, 1000000);
							  } 
							}
					    
							u8Count++;
					    s_u8CopyData = 0;
					    s_u8pdmaIRQReIn = 1; 
					    
							if(u8Count >= (kws.numFrames-1))
								u8Count = 0;
							
							
        } //s_u8CopyData
    } //while
}

}
