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

#include <jendefs.h>
#include <string.h>
#include "CodecPreBuffer.h"

bool_t bCodecData[CODEC_PRE_BUFFERS];
uint8 au8CodecData[CODEC_PRE_BUFFERS][96];

/**
 * initialize the buffer
 */
void CodecPreBuffer_vInit() {
	memset(bCodecData, 0, sizeof(bCodecData));
}

/**
 * find a buffer to put.
 * @return
 */
uint8* CodecPreBuffer_pu8Find() {
	int i;

	for (i = CODEC_PRE_BUFFERS; i > 0; i--) {
		if (bCodecData[i - 1]) {
			bCodecData[i - 1] = FALSE;
			return au8CodecData[i - 1];
		}
	}
	return NULL;
}

/**
 * add data into the buffer.
 * @param pu8buf
 * @return
 */
uint8 CodecPreBuffer_u8Add(uint8 *pu8buf, uint8 u8len) {
	int i;
	for (i = 0; i < CODEC_PRE_BUFFERS; i++) {
		if (bCodecData[i] == FALSE) {
			bCodecData[i] = TRUE;
			memcpy(au8CodecData[i], pu8buf, u8len);
			return i;
		}
	}

	return 0xFF;
}

