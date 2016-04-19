/****************************************************************************
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.5 $
 *
 * DATED:              $Date: 2007/12/19 16:05:33 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             Scott Nichols
 *
 * DESCRIPTION:
 * Handles DAC and ADC samples
 *
 * CHANGE HISTORY:
 *
 * $Log: audioManager.h,v $
 * Revision 1.5  2007/12/19 16:05:33  snich
 * Added channel black listing
 *
 * Revision 1.4  2007/08/09 10:33:03  rmm
 * std Jennic disclaimer added to the header
 *
 * Revision 1.3  2007/08/03 09:04:24  snich
 * moved files from library into main source
 *
 * Revision 1.1  2007/07/25 16:03:09  snich
 * *** empty log message ***
 *
 * Revision 1.1  2007/07/25 11:13:44  snich
 * Rewrite
 *
 *
 *
 * LAST MODIFIED BY:   $Author: snich $
 *                     $Modtime: $
 *
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2005, 2006. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - all rights reserved.
 *
 * Condition to use: (refer to detailed conditions in Japanese)
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 * 利用条件:
 *   - 本ソースコードは、別途ソースコードライセンス記述が無い限り東京コスモス電機が著作権を
 *     保有しています。
 *   - 本ソースコードは、無保証・無サポートです。本ソースコードや生成物を用いたいかなる損害
 *     についても東京コスモス電機は保証致しません。不具合等の報告は歓迎いたします。
 *   - 本ソースコードは、東京コスモス電機が販売する TWE シリーズ上で実行する前提で公開
 *     しています。他のマイコン等への移植・流用は一部であっても出来ません。
 *
 ****************************************************************************/

#ifndef  AUDIOMANAGER_INCLUDED
#define  AUDIOMANAGER_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "config.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* サンプルのバッファーサイズ */
#define MAX_SAMPLE_BUFFER_SIZE 256

/* Calculate tick timer period based on required sampling frequency */
#define TICK_TIMER_INT                  ((((XTAL_FREQ_HZ / (256 * SAMPLE_FREQ_HZ)) + 1)* 256) - 1)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct {
	bool_t bInpLPF;
	bool_t bOutLPF;
	uint16 u16SampleBufferSize;
} tsAM_Conf;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
void     vAM_Init(tsAM_Conf *);
void     vAM_SetSampleRate(uint16 u16SampleFreq);
int16*   pi16AM_GetFreeDataBlock(void);
uint8 u8AM_CountFreeDataBlock(void);
void     vAM_DataBlockLoaded(int16 *pi16Buffer);
int16*   pi16AM_GetData(void);
void     vAM_StartStopSampling(bool_t bStart);
void     vAM_SetVolumeLevel(uint8 volume);
void     vAM_TestToneOn(bool_t testToneOn);
uint32   u32AM_TimeUntilDataReady(void);
uint16   u16AM_GetSampleBufferSize();

void vAM_ISR(uint32 u32d, uint32 u32m);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

