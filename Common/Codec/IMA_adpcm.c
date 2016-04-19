/***********************************************************
 Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
 Netherlands.

 All Rights Reserved

 Permission to use, copy, modify, and distribute this software and its
 documentation for any purpose and without fee is hereby granted,
 provided that the above copyright notice appear in all copies and that
 both that copyright notice and this permission notice appear in
 supporting documentation, and that the names of Stichting Mathematisch
 Centrum or CWI not be used in advertising or publicity pertaining to
 distribution of the software without specific, written prior permission.
 ******************************************************************/

/***********************************************************
 Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
 Netherlands.

 All Rights Reserved

 Permission to use, copy, modify, and distribute this software and its
 documentation for any purpose and without fee is hereby granted,
 provided that the above copyright notice appear in all copies and that
 both that copyright notice and this permission notice appear in
 supporting documentation, and that the names of Stichting Mathematisch
 Centrum or CWI not be used in advertising or publicity pertaining to
 distribution of the software without specific, written prior permission.
 ******************************************************************/

/*
 ** Intel/DVI ADPCM coder/decoder.
 **
 ** The algorithm for this coder was taken from the IMA Compatability Project
 ** proceedings, Vol 2, Number 2; May 1992.
 **
 ** Version 1.2, 18-Dec-92.
 **
 */

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

#include <string.h>

#include "codec.h"
#include "IMA_adpcm.h"

#ifndef __STDC__
#define signed
#endif

/* Intel ADPCM step variation table */
static int16 indexTable[16] =
{ -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8, };

static int16 stepsizeTable[89] =
{ 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
		50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209,
		230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
		876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499,
		2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
		8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350,
		22385, 24623, 27086, 29794, 32767 };

/**
 * 初期化
 * @param psCodec
 * @param state
 */
void CODEC_IMA_Adpcm_vInit(tsCODEC *psCodec, tsImaAdpcmState *state) {
	psCodec->vState = state;
	memset(state, 0, sizeof(tsImaAdpcmState));

	psCodec->fp_u16Decoder = CODEC_IMA_Adpcm_u16Decoder;
	psCodec->fp_u16Encoder = CODEC_IMA_Adpcm_u16Encoder;
	psCodec->fp_u8SetStatus = CODEC_IMA_Adpcm_u8SetStatus;
	psCodec->fp_pu8GetStatus = CODEC_IMA_Adpcm_pu8GetStatus;
}

/**
 *
 * @param psCodec
 * @param pu8statinfo
 * @return
 */
uint8 CODEC_IMA_Adpcm_u8SetStatus(tsCODEC *psCodec, uint8 *pu8statinfo) {
	tsImaAdpcmState *state = psCodec->vState;
	*state = *(tsImaAdpcmState *)pu8statinfo; // 状態配列のコピー

	return sizeof(tsImaAdpcmState);
}

/**
 *
 * @param psCodec
 * @param pu8statinfo
 * @return
 */
uint8* CODEC_IMA_Adpcm_pu8GetStatus(tsCODEC *psCodec, uint8 *pu8len) {
	*pu8len = sizeof(tsImaAdpcmState);
	return psCodec->vState;
}

/**
 * IMA ADPCM エンコーダ
 *
 * @param indata 入力データ列(16bit)
 * @param outdata 出力バイト列
 * @param len 入力データ数
 * @param vState 状態
 * @return 出力バイト数
 */
uint16 CODEC_IMA_Adpcm_u16Encoder(tsCODEC *psCodec, int16 *indata, uint8 *outdata, int16 len)
{
	int16 *inp; /* Input buffer pointer */
	uint8 *outp; /* output buffer pointer */
	int16 val; /* Current input sample value */
	int16 sign; /* Current adpcm sign bit */
	int16 delta; /* Current adpcm output value */
	int32 diff; /* Difference between val and valprev */
	int16 step; /* Stepsize */
	int32 valprev; /* Predicted output value */
	int16 vpdiff; /* Current change to valpred */
	int16 index; /* Current step change index */

	outp = outdata;
	inp = indata;

	tsImaAdpcmState *state = psCodec->vState;

	valprev = state->valprev;
	index = state->index;
	step = stepsizeTable[index];


	for (; len > 0; len--)
	{
		val = *inp++;

		val >>= 3;

		/* Step 1 - compute difference with previous value */
		diff = val - valprev;
		sign = (diff < 0) ? 8 : 0;
		if (sign)
			diff = (-diff);

		/* Step 2 - Divide and clamp */
		/* Note:
		 ** This code *approximately* computes:
		 **    delta = diff*4/step;
		 **    vpdiff = (delta+0.5)*step/4;
		 ** but in shift step bits are dropped. The net result of this is
		 ** that even if you have fast mul/div hardware you cannot put it to
		 ** good use since the fixup would be too expensive.
		 */
		delta = 0;
		vpdiff = (step >> 3);

		if (diff >= step)
		{
			delta += 4;
			diff -= step;
			vpdiff += step;
		}
		step >>= 1;
		if (diff >= step)
		{
			delta += 2;
			diff -= step;
			vpdiff += step;
		}
		step >>= 1;
		if (diff >= step)
		{
			delta += 1;
			vpdiff += step;
		}

		/* Step 3 - Update previous value */
		if (sign)
			valprev -= vpdiff;
		else
			valprev += vpdiff;

		/* Step 4 - Clamp previous value to 16 bits */
		if (valprev > 32767)
			valprev = 32767;
		else if (valprev < -32768)
			valprev = -32768;

		/* Step 5 - Assemble value, update index and step values */
		delta |= sign;

		index += indexTable[delta];
		if (index < 0)
			index = 0;
		if (index > 88)
			index = 88;
		step = stepsizeTable[index];

		/* Step 6 - Output value */
		if (len & 1) {
			*outp = (delta & 0x0f) | *outp;
			outp++;
		} else {
			*outp = ((delta << 4) & 0xf0);
		}

		state->valprev = valprev;
		state->index = index;
	}

	return outp - outdata;
}

/**
 * IMA ADPCM デコーダ
 * @param indata 入力バイト列
 * @param outdata 出力データ列 (16bit)
 * @param len サンプル数(バッファ長ではないのに注意！)
 * @param vState ADPCM状態
 * @return 出力データバイト数
 */
uint16 CODEC_IMA_Adpcm_u16Decoder(tsCODEC *psCodec, uint8 *indata, int16 *outdata, int16 len)
{
	uint8 *inp; /* Input buffer pointer */
	int16 *outp; /* output buffer pointer */
	int16 sign; /* Current adpcm sign bit */
	int16 delta; /* Current adpcm output value */
	int16 step; /* Stepsize */
	int32 valprev; /* Predicted value */
	int32 vpdiff; /* Current change to valpred */
	int16 index; /* Current step change index */

	outp = outdata;
	inp = indata;

	tsImaAdpcmState *state = psCodec->vState;

	valprev = state->valprev;
	index = state->index;
	step = stepsizeTable[index];

	for (; len > 0; len--)
	{
		/* Step 1 - get the delta value and compute next index */
		if (len & 1) {
			delta = *inp & 0x0f;
			inp++;
		} else {
			delta = ((*inp) >> 4) & 0x0f;
		}

		/* Step 2 - Find new index value (for later) */
		index += indexTable[delta];
		if (index < 0)
			index = 0;
		if (index > 88)
			index = 88;

		/* Step 3 - Separate sign and magnitude */
		sign = delta & 8;
		delta = delta & 7;

		/* Step 4 - Compute difference and new predicted value */
		/*
		 ** Computes 'vpdiff = (delta+0.5)*step/4', but see comment
		 ** in adpcm_coder.
		 */
		vpdiff = step >> 3;
		if (delta & 4)
			vpdiff += step;
		if (delta & 2)
			vpdiff += step >> 1;
		if (delta & 1)
			vpdiff += step >> 2;

		if (sign)
			valprev -= vpdiff;
		else
			valprev += vpdiff;

		/* Step 5 - clamp output value */
		if (valprev > 32767)
			valprev = 32767;
		else if (valprev < -32768)
			valprev = -32768;

		/* Step 6 - Update step value */
		step = stepsizeTable[index];

		/* Step 7 - Output value */
		*outp++ = valprev << 3;

		state->valprev = valprev;
		state->index = index;
	}

	return len * 2; // サンプル数
}
