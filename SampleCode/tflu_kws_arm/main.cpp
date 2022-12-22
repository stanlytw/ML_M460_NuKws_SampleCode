/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an keyword spotting inference offline with TFLITE v1.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include <vector>
using namespace std;

//#include <arm_math.h>
#include "KWS/kws.h"
#include "raw/go_2.h"
#include "BufAttributes.h"


#define NAU8822     1

static uint32_t s_au32PcmRxBuff[2][BUFF_LEN] = {{0}};
//static uint32_t s_au32PcmTxBuff[2][BUFF_LEN] = {{0}};
static DMA_DESC_T DMA_TXDESC[2], DMA_RXDESC[2];

//KWS
KWS *kws;

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

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x2)
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x4)              /* channel 2 done */
        {
            /* Copy RX data to TX buffer */
            s_u8CopyData = 1;
            s_u8RxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }

        if(PDMA_GET_TD_STS(PDMA0) & 0x2)              /* channel 1 done */
        {
            s_u8TxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);
        }
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}


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
    printf("  NOTE: .\n");

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
    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S);
#endif

    /* Set PD3 low to enable phone jack on NuMaker board. */
    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk);
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* Select source from HXT(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

#ifndef INPUT_IS_LIN
    /* NAU8822 will store data in left channel */
    I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_LEFT);
#endif



   printf("\nThis sample code run keyword spotting inference offline\n");
	
	const char outputClass[12][8] = {
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

    int16_t audioBuffer[] ALIGNMENT_ATTRIBUTE = WAVE_DATA;
   const uint32_t audioBufferElements = sizeof(audioBuffer) / sizeof(int16_t);

   printf("KWS simple example; build timestamp: %s:%s\n", __DATE__, __TIME__);

   printf("Initialising KWS object. Wav data has %du elements\r\n",
       audioBufferElements);
			
   KWS kws(audioBuffer, audioBufferElements);
	//kws = new KWS(audioBuffer,audioBufferElements);		

   printf("Extracting features.. \r\n");
   kws.ExtractFeatures();  // Extract MFCC features.

   printf("Classifying..\r\n");
   kws.Classify();  // Classify the extracted features.

   int maxIndex = kws.GetTopClass(kws.output);

   printf("Detected %s (%d%%)\r\n", outputClass[maxIndex],
       (static_cast<int>(kws.output[maxIndex]*100)));

    

    while(1)
    {
        if(s_u8CopyData)
        {
    //        memcpy(&s_au32PcmTxBuff[s_u8TxIdx ^ 1], &s_au32PcmRxBuff[s_u8RxIdx], BUFF_LEN * 4);
        }
    }
}
