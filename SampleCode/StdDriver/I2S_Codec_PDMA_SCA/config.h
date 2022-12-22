/**************************************************************************//**
 * @file     config.h
 * @version  V3.00
 * @brief    I2S driver sample configuration header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define sample_r   16000  //sample rate
#define sample_bits  32   //sample bits, uint32 = 4 bytes
#define ch  1  //channels
#define ms 100 //wanted ms

#define BUFF_NUM        4
#define BUFF_LEN        1024
#define PDMA_TEST_LENGTH  64000   // sample rate = 16000, 16000 * 4(s) = 64k
#define BUFF_HALF_LEN   (BUFF_LEN/2)

/* Use LIN as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

extern void PDMA_ResetTxSGTable(uint8_t id);
extern void PDMA_ResetRxSGTable(uint8_t id);
extern void PDMA_Init(void);
#endif
