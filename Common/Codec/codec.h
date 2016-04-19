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

#ifndef CODEC_H_
#define CODEC_H_

#include <jendefs.h>


typedef struct _tsCODEC {
	uint8 (*fp_u8SetStatus)(struct _tsCODEC *, uint8 *);
	uint8* (*fp_pu8GetStatus)(struct _tsCODEC *, uint8 *);
	uint16 (*fp_u16Decoder)(struct _tsCODEC *, uint8 *, int16 *, int16 );
	uint16 (*fp_u16Encoder)(struct _tsCODEC *, int16 *, uint8 *, int16 );
	void *vState;
} tsCODEC;

#define CODEC_u8SetStatus(a, b) (*((a).fp_u8SetStatus))(&(a), b)
#define CODEC_pu8GetStatus(a, b) (*((a).fp_pu8GetStatus))(&(a), b)
#define CODEC_u16Encode(a, b, c, d) (*((a).fp_u16Encoder))(&(a), b, c, d)
#define CODEC_u16Decode(a, b, c, d) (*((a).fp_u16Decoder))(&(a), b, c, d)

#endif /* CODEC_H_ */
