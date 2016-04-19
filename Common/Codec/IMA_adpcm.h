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
 **  ======== adpcm.h ========
 **
 **  Header file for adpcm.c
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

#ifndef IMA_ADPCM_H
#define IMA_ADPCM_H

#ifdef DEBUG_CMD_LINE
#include "jendefs_dup.h"
#else
#include <jendefs.h>
#endif

typedef struct
{
	int16 valprev; /* Previous output value */
	int16 index;
} tsImaAdpcmState;

void CODEC_IMA_Adpcm_vInit(tsCODEC *psCodec, tsImaAdpcmState *state);
uint8 CODEC_IMA_Adpcm_u8SetStatus(tsCODEC *psCodec, uint8 *pu8statinfo);
uint8* CODEC_IMA_Adpcm_pu8GetStatus(tsCODEC *psCodec, uint8 *pu8len);
uint16 CODEC_IMA_Adpcm_u16Encoder(tsCODEC *psCodec, int16 *indata, uint8 *outdata, int16 len);
uint16 CODEC_IMA_Adpcm_u16Decoder(tsCODEC *psCodec, uint8 *indata, int16 *outdata, int16 len);

#endif
