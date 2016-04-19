/****************************************************************************
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.8 $
 *
 * DATED:              $Date: 2008/01/04 11:11:39 $
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
 * $Log: audioManager.c,v $
 * Revision 1.8  2008/01/04 11:11:39  snich
 * Fixed FA bug when using sample rates other than 8K and 16K
 *
 * Revision 1.7  2007/12/19 16:05:33  snich
 * Added channel black listing
 *
 * Revision 1.6  2007/12/05 09:38:23  ahuss
 * Added Unix OEL stuff.
 *
 * Revision 1.5  2007/11/27 14:41:57  rmm
 * no message
 *
 * Revision 1.4  2007/08/09 10:32:37  rmm
 * std Jennic disclaimer added to the header
 *
 * Revision 1.3  2007/08/03 09:04:24  snich
 * moved files from library into main source
 *
 * Revision 1.4  2007/08/01 16:07:57  snich
 * added option to repeat last packet if both DAC buffers empty
 *
 * Revision 1.3  2007/08/01 13:22:58  snich
 * *** empty log message ***
 *
 * Revision 1.2  2007/08/01 12:48:24  snich
 * Init the audio manager state to idle in  vAM_init
 *
 * Revision 1.1  2007/07/25 16:03:08  snich
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

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "audioManager.h"
#include "audioDevice.h"
#include "testwave.h"

#include "common.h" // for DEBUG printf

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define DAC_BUFFER_SIZE (sAM_Data.u16SampleBufferSize)
#define DAC_BUFFER1_MID (&sAM_Data.ai16DacBuffer1[DAC_BUFFER_SIZE/2])
#define DAC_BUFFER2_MID (&sAM_Data.ai16DacBuffer2[DAC_BUFFER_SIZE/2])

#define DAC_BUFFER1_END (&sAM_Data.ai16DacBuffer1[DAC_BUFFER_SIZE])
#define DAC_BUFFER2_END (&sAM_Data.ai16DacBuffer2[DAC_BUFFER_SIZE])

#define ADC_BUFFER1_END (&sAM_Data.ai16AdcBuffer1[DAC_BUFFER_SIZE])
#define ADC_BUFFER2_END (&sAM_Data.ai16AdcBuffer2[DAC_BUFFER_SIZE])

#define DAC_BUFFER1_START (sAM_Data.ai16DacBuffer1)
#define DAC_BUFFER2_START (sAM_Data.ai16DacBuffer2)

#define ADC_BUFFER1_START (sAM_Data.ai16AdcBuffer1)
#define ADC_BUFFER2_START (sAM_Data.ai16AdcBuffer2)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum {
	E_AM_STATE_IDLE = 0, E_AM_STATE_ACTIVE_TEST_TONE, E_AM_STATE_ACTIVE
} teAM_STATES;

typedef struct {
	int16 *pi16NextDacEntry1;
	int16 *pi16NextDacEntry2;
	int16 *pi16NextAdcEntry1;
	int16 *pi16NextAdcEntry2;
	int16 ai16DacBuffer1[MAX_SAMPLE_BUFFER_SIZE];
	int16 ai16DacBuffer2[MAX_SAMPLE_BUFFER_SIZE];
	int16 ai16AdcBuffer1[MAX_SAMPLE_BUFFER_SIZE];
	int16 ai16AdcBuffer2[MAX_SAMPLE_BUFFER_SIZE];
	uint16 u16SampleFreq;
	uint16 u16SamplesLeftReady;
	uint8 u8CurrentDacBuffer;
	uint8 u8CurrentAdcBuffer;
	uint8 u8TestWaveSampleNbr;
	bool_t bAdcBuffer1Ready;
	bool_t bAdcBuffer2Ready;
	tsAM_Conf sConf;
	teAM_STATES eState;

	uint16 u16SampleBufferSize;
} tsAM_Data;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static bool_t bPutADCSample(int16 i16Sample);
static int16 i16GetDACSample();
static bool_t bGetCurrentDACSample(int16 *pi16Sample);
//PRIVATE bool_t  bEmptyDACSampleBuffer();
static void vBuffersInit(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
static tsAM_Data sAM_Data;
static uint32 u32XTAL_FREQ_HZ;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vAM_Init
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_Init(tsAM_Conf *pConf) {
	u32XTAL_FREQ_HZ = 16000000UL;

	vBuffersInit();
	sAM_Data.sConf = *pConf;

	sAM_Data.eState = E_AM_STATE_IDLE;
	sAM_Data.u16SampleFreq = AUDIO_DEV_SAMPLE_FREQ;
	sAM_Data.u16SampleBufferSize = pConf->u16SampleBufferSize ? pConf->u16SampleBufferSize : MAX_SAMPLE_BUFFER_SIZE;
}

/****************************************************************************
 *
 * NAME: vAM_SetSampleRate
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_SetSampleRate(uint16 u16SampleFreq) {
	sAM_Data.u16SampleFreq = u16SampleFreq;

	// TODO AUDIO DEVICE への設定

	vBuffersInit();
}

/****************************************************************************
 *
 * NAME: pi16AM_NewDataBlock
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
int16* pi16AM_GetFreeDataBlock(void) {
	int16 *pi16Buffer = NULL;

	if (sAM_Data.pi16NextDacEntry2 >= DAC_BUFFER2_END)
	{
		pi16Buffer = DAC_BUFFER2_START;
	}
	else if (sAM_Data.pi16NextDacEntry1 >= DAC_BUFFER1_END)
	{
		pi16Buffer = DAC_BUFFER1_START;
	}

	return pi16Buffer;
}

uint8 u8AM_CountFreeDataBlock(void) {
	uint8 u8ct = 0;

	if (sAM_Data.pi16NextDacEntry2 >= DAC_BUFFER2_END)
	{
		u8ct++;
	}
	if (sAM_Data.pi16NextDacEntry1 >= DAC_BUFFER1_END)
	{
		u8ct++;
	}

	return u8ct;
}
/****************************************************************************
 *
 * NAME: vAM_DataBlockLoaded
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_DataBlockLoaded(int16 *pi16Buffer) {
	if (pi16Buffer == DAC_BUFFER1_START) {
		sAM_Data.pi16NextDacEntry1 = DAC_BUFFER1_START;
	} else {
		sAM_Data.pi16NextDacEntry2 = DAC_BUFFER2_START;
	}
}

/****************************************************************************
 *
 * NAME: pi16AM_GetData
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
int16* pi16AM_GetData(void) {
	int16 *pi16Buffer = NULL;

	if (sAM_Data.bAdcBuffer1Ready == TRUE) {
		sAM_Data.bAdcBuffer1Ready = FALSE;
		pi16Buffer = ADC_BUFFER1_START;
	} else if (sAM_Data.bAdcBuffer2Ready == TRUE) {
		sAM_Data.bAdcBuffer2Ready = FALSE;
		pi16Buffer = ADC_BUFFER2_START;
	}

	return pi16Buffer;
}

/****************************************************************************
 *
 * NAME: vAM_StartStopSampling
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_StartStopSampling(bool_t bStart) {
	if (bStart == TRUE) {
		sAM_Data.eState = E_AM_STATE_ACTIVE;
	} else {
		sAM_Data.eState = E_AM_STATE_IDLE;
	}
}

/****************************************************************************
 *
 * NAME: vAM_SetVolumeLevel
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_SetVolumeLevel(uint8 volume) {
}

/****************************************************************************
 *
 * NAME: vAM_TestToneOn
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_TestToneOn(bool_t testToneOn) {
	if (testToneOn == TRUE) {
		sAM_Data.eState = E_AM_STATE_ACTIVE_TEST_TONE;
	} else {
		sAM_Data.eState = E_AM_STATE_ACTIVE;
	}
}

/****************************************************************************
 *
 * NAME: u16AM_TimeUntilDataReady
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
uint32 u32AM_TimeUntilDataReady(void) {
	uint16 usLeft = 0;

	if (sAM_Data.bAdcBuffer1Ready == FALSE
			&& sAM_Data.bAdcBuffer2Ready == FALSE) {
		usLeft = (((uint32) sAM_Data.u16SamplesLeftReady) * 1000000)
				/ sAM_Data.u16SampleFreq;
	}

	return usLeft;
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vBuffersInit
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
static void vBuffersInit(void) {
	sAM_Data.u8CurrentDacBuffer = 1;
	sAM_Data.pi16NextDacEntry1 = DAC_BUFFER1_END;
	sAM_Data.pi16NextDacEntry2 = DAC_BUFFER2_END;

	sAM_Data.u8CurrentAdcBuffer = 1;
	sAM_Data.pi16NextAdcEntry1 = ADC_BUFFER1_START;
	sAM_Data.pi16NextAdcEntry2 = ADC_BUFFER1_START;

	sAM_Data.u8TestWaveSampleNbr = 0;

	sAM_Data.bAdcBuffer1Ready = FALSE;
	sAM_Data.bAdcBuffer2Ready = FALSE;

	sAM_Data.u16SamplesLeftReady = sAM_Data.u16SampleBufferSize;
}

/****************************************************************************
 *
 * NAME: vAM_ISR
 *
 * DESCRIPTION:
 * Tick timer interrupt service routine. Outputs value to DAC.
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void vAM_ISR(uint32 u32d, uint32 u32m) {
	int16 i16ADCSample = 0;
	uint8 u8MaxTestWaveSamples;

	static uint16 u16Delay = MAX_SAMPLE_BUFFER_SIZE / 2;
	uint8 u8FreeBlk = u8AM_CountFreeDataBlock();

	// サンプルバッファの予備バッファ処理
	// 最初のサンプルを得てから少しだけ待ってサンプルの利用を開始する
	if (u16Delay) {
		if (u8FreeBlk == 1) { // １ブロック埋まったらカウントダウン開始
			u16Delay--;
		} else if (u8FreeBlk == 0) { // ２ブロック埋まった、一定サンプル読み飛ばして開始
			int i;
			for (i = 0; i < sAM_Data.u16SampleBufferSize / 2; i++) {
				i16GetDACSample();
			}
			u16Delay = 0;
		}
	} else {
		if (u8FreeBlk == 2) { // バッファが空になったらウェイトを設定する
			u16Delay = sAM_Data.u16SampleBufferSize / 2;
			S_PUTCHAR('B');
		}
	}

	// サンプルバッファから取り出し
	int16 i16Samp = u16Delay ? 0 : i16GetDACSample();
	if (sAM_Data.sConf.bOutLPF) {
		static int16 i16Prev;

		int16 i16New = (i16Prev + i16Samp) >> 1;
		i16Prev = i16Samp;
		i16Samp = i16New;
	}

	switch (sAM_Data.eState) {
	case E_AM_STATE_IDLE:
		break;

	case E_AM_STATE_ACTIVE_TEST_TONE:
		if (sAM_Data.u16SampleFreq < 16000) {
			u8MaxTestWaveSamples = TEST_WAVE_FORM_SAMPLES_8KHZ;
			i16ADCSample = ai16TestWaveForm8KHz[sAM_Data.u8TestWaveSampleNbr];
		} else if (sAM_Data.u16SampleFreq == 16000) {
			u8MaxTestWaveSamples = TEST_WAVE_FORM_SAMPLES_16KHZ;
			i16ADCSample = ai16TestWaveForm16KHz[sAM_Data.u8TestWaveSampleNbr];
		} else if (sAM_Data.u16SampleFreq == 24000) {
			u8MaxTestWaveSamples = TEST_WAVE_FORM_SAMPLES_24KHZ;
			i16ADCSample = ai16TestWaveForm24KHz[sAM_Data.u8TestWaveSampleNbr];
		} else {
			u8MaxTestWaveSamples = TEST_WAVE_FORM_SAMPLES_32KHZ;
			i16ADCSample = ai16TestWaveForm32KHz[sAM_Data.u8TestWaveSampleNbr];
		}
		if (++sAM_Data.u8TestWaveSampleNbr >= u8MaxTestWaveSamples) {
			sAM_Data.u8TestWaveSampleNbr = 0;
		}

		/* just ignore the sample coming from the codec */
		i16AD_ReadWrite(i16Samp); // サンプルの出力（入力は無視）
		bPutADCSample(i16ADCSample); // テストトーンのサンプルを格納する
		break;

	case E_AM_STATE_ACTIVE:
		i16ADCSample = i16AD_ReadWrite(i16Samp); // サンプルの入出力

		// 直前のサンプルとの差を取る
		if (sAM_Data.sConf.bInpLPF) {
			static int16 i16Prev;

			int16 i16New = (i16Prev + i16ADCSample) >> 1;
			i16Prev = i16ADCSample;
			i16ADCSample = i16New;
		}
		bPutADCSample(i16ADCSample); // 入力デバイスからの値を格納する
		break;
	}
}

/****************************************************************************
 *
 * NAME: bPutADCSample
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
static bool_t bPutADCSample(int16 i16Sample) {
	bool_t bStatus = FALSE;

	if (sAM_Data.u8CurrentAdcBuffer == 1) {
		if (sAM_Data.pi16NextAdcEntry1 < ADC_BUFFER1_END)
		{
			*sAM_Data.pi16NextAdcEntry1 = i16Sample;
			sAM_Data.pi16NextAdcEntry1++;
			sAM_Data.u16SamplesLeftReady--;
			bStatus = TRUE;

			if (sAM_Data.pi16NextAdcEntry1 == ADC_BUFFER1_END)
			{
				sAM_Data.u8CurrentAdcBuffer = 2;
				sAM_Data.bAdcBuffer1Ready = TRUE;
				sAM_Data.bAdcBuffer2Ready = FALSE;
				sAM_Data.pi16NextAdcEntry1 = ADC_BUFFER1_START;
				sAM_Data.u16SamplesLeftReady = sAM_Data.u16SampleBufferSize;
			}
		}
	}
	else
	{
		if (sAM_Data.pi16NextAdcEntry2 < ADC_BUFFER2_END)
		{
			*sAM_Data.pi16NextAdcEntry2 = i16Sample;
			sAM_Data.pi16NextAdcEntry2++;
			sAM_Data.u16SamplesLeftReady--;
			bStatus = TRUE;

			if (sAM_Data.pi16NextAdcEntry2 == ADC_BUFFER2_END)
			{
				sAM_Data.u8CurrentAdcBuffer = 1;
				sAM_Data.bAdcBuffer1Ready = FALSE;
				sAM_Data.bAdcBuffer2Ready = TRUE;
				sAM_Data.pi16NextAdcEntry2 = ADC_BUFFER2_START;
				sAM_Data.u16SamplesLeftReady = sAM_Data.u16SampleBufferSize;
			}
		}
	}
	return bStatus;
}

/****************************************************************************
 *
 * NAME: i16GetDACSample
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
static int16 i16GetDACSample() {
	static int16 i16Sample = 0;

	if (bGetCurrentDACSample(&i16Sample) == FALSE) {

		/* no data so swap buffers */
		if (sAM_Data.u8CurrentDacBuffer == 1) {
			sAM_Data.u8CurrentDacBuffer = 2;
		} else {
			sAM_Data.u8CurrentDacBuffer = 1;
		}

		if (bGetCurrentDACSample(&i16Sample) == FALSE) {

#ifdef REPEAT_PACKET_DAC_BUFFER_EMPTY
			/* no data so swap back again */
			if(sAM_Data.u8CurrentDacBuffer == 1)
			{
				sAM_Data.u8CurrentDacBuffer = 2;
				sAM_Data.pi16NextDacEntry2 = DAC_BUFFER2_START;
			}
			else
			{
				sAM_Data.u8CurrentDacBuffer = 1;
				sAM_Data.pi16NextDacEntry1 = DAC_BUFFER1_START;
			}
#endif
		}
	}

	return i16Sample;
}

/****************************************************************************
 *
 * NAME: bGetCurrentDACSample
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
static bool_t bGetCurrentDACSample(int16 *pi16Sample) {
	bool_t bStatus = FALSE;

	if (sAM_Data.u8CurrentDacBuffer == 1) {
		if (sAM_Data.pi16NextDacEntry1 < DAC_BUFFER1_END)
		{
			*pi16Sample = *sAM_Data.pi16NextDacEntry1;
			sAM_Data.pi16NextDacEntry1++;
			bStatus = TRUE;
		}
	}
	else
	{
		if (sAM_Data.pi16NextDacEntry2 < DAC_BUFFER2_END )
		{
			*pi16Sample = *sAM_Data.pi16NextDacEntry2;
			sAM_Data.pi16NextDacEntry2++;
			bStatus = TRUE;
		}
	}

	return bStatus;
}

#if 0 // unused
static bool_t bEmptyDACSampleBuffer() {
	bool_t bStatus = TRUE;

	if (sAM_Data.u8CurrentDacBuffer == 1)
	{
		if (sAM_Data.pi16NextDacEntry1 < DAC_BUFFER1_END )
		{
			bStatus = FALSE;
		}
	}
	else
	{
		if (sAM_Data.pi16NextDacEntry2 < DAC_BUFFER2_END )
		{
			bStatus = FALSE;
		}
	}

	return bStatus;
}
#endif

/****************************************************************************
 *
 * NAME: u16AM_GetSampleBufferSize
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
uint16 u16AM_GetSampleBufferSize() {
	return sAM_Data.u16SampleBufferSize;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
