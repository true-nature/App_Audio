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

#include "codec.h"
#include "RAW.h"

/**
 * signed 16bit を unsigned 10bit に詰める
 *
 * @param pi16Data 入力サンプル配列
 * @param u16smplen サンプル数
 * @param pu8CodedBuff 出力データ(４の倍数であること）
 * @return 出力データ長
 */
inline static uint16 pack(int16 *pi16Data, uint16 u16smplen, uint8 *pu8CodedBuff)
{
	uint32 out_buffer = 0;
	int32 out_bits = 0;
	uint8 out_byte;

	int32 code;

	uint8 *po = pu8CodedBuff;
	int16 *pi = pi16Data;

	const uint8 bits = 10;

	while(u16smplen > 0) {
		code = ((*pi >> 6) + 512); // 10bit

		/* pack bits into byte array */
		out_buffer |= (code << out_bits);
		out_bits += bits;
		while (out_bits >= 8) {
			out_byte = out_buffer & 0xff;
			out_bits -= 8;
			out_buffer >>= 8;
			*po++ = out_byte;
		}

		u16smplen--;
		pi++;
	}

	return (po - pu8CodedBuff);
}

inline static uint16 pack8(int16 *pi16Data, uint16 u16smplen, uint8 *pu8CodedBuff)
{
	int32 code;

	uint8 *po = pu8CodedBuff;
	int16 *pi = pi16Data;

	while(u16smplen > 0) {
		code = ((*pi >> 8) + 128); // unsigned 8bit に変換
		*po++ = code & 0xFF;

		u16smplen--;
		pi++;
	}

	return (po - pu8CodedBuff);
}

/**
 * unsigned 10bit を singed 16bit に展開する
 *
 * @param pu8CodedBuff 入力データ
 * @param u16smplen 展開後のサンプル数（４の倍数であること）
 * @param pi16Data 出力サンプル配列
 * @return 展開後のサンプル数
 */
inline static uint16 unpack(uint8 *pu8CodedBuff, uint16 u16smplen, int16 *pi16Data) {
	uint32 in_buffer = 0;
	int32 in_bits = 0;
	uint8 in_byte;

	int32 code;

	uint8 *pi = pu8CodedBuff;
	int16 *po = pi16Data;

	const uint8 bits = 10;

	while (u16smplen) {
		int i32s;

		/* extract packed bits */
		while (in_bits < bits) {
			in_byte = *pi++;

			in_buffer |= (in_byte << in_bits);
			in_bits += 8;
		}
		code = in_buffer & ((1 << bits) - 1);

		i32s = (code - 512) << 6; // 10bit を singed 16bit に変換

		*po = (int16)(i32s);

		in_buffer >>= bits;
		in_bits -= bits;

		u16smplen--;
		po++;
	}

	return po - pi16Data;
}

inline static uint16 unpack8(uint8 *pu8CodedBuff, uint16 u16smplen, int16 *pi16Data) {
	int32 code;
	uint8 *pi = pu8CodedBuff;
	int16 *po = pi16Data;

	while (u16smplen) {
		int i32s;

		/* extract packed bits */
		code = *pi++;

		i32s = (code - 128) << 8; // unsigned 8bit を singed 16bit に変換

		*po = (int16)(i32s);

		u16smplen--;
		po++;
	}

	return po - pi16Data;
}

void CODEC_RAW_vInit(tsCODEC *psCodec) {
	psCodec->fp_u16Encoder = CODEC_RAW_u16Encoder;
	psCodec->fp_u16Decoder = CODEC_RAW_u16Decoder;
	psCodec->fp_u8SetStatus = CODEC_RAW_u8SetStatus;
	psCodec->fp_pu8GetStatus = CODEC_RAW_pu8GetStatus;
}

void CODEC_RAW8_vInit(tsCODEC *psCodec) {
	psCodec->fp_u16Encoder = CODEC_RAW8_u16Encoder;
	psCodec->fp_u16Decoder = CODEC_RAW8_u16Decoder;
	psCodec->fp_u8SetStatus = CODEC_RAW_u8SetStatus;
	psCodec->fp_pu8GetStatus = CODEC_RAW_pu8GetStatus;
}

uint8 CODEC_RAW_u8SetStatus(tsCODEC *psCodec, uint8 *pu8statinfo) {
	return 0;
}

uint8* CODEC_RAW_pu8GetStatus(tsCODEC *psCodec, uint8 *pu8len) {
	*pu8len = 0;
	return NULL;
}

uint16 CODEC_RAW_u16Encoder(tsCODEC *psCodec, int16 *indata, uint8 *outdata, int16 len) {
	return pack(indata, len, outdata);
}

uint16 CODEC_RAW_u16Decoder(tsCODEC *psCodec, uint8 *indata, int16 *outdata, int16 len) {
	return unpack(indata, len, outdata);
}

uint16 CODEC_RAW8_u16Encoder(tsCODEC *psCodec, int16 *indata, uint8 *outdata, int16 len) {
	return pack8(indata, len, outdata);
}

uint16 CODEC_RAW8_u16Decoder(tsCODEC *psCodec, uint8 *indata, int16 *outdata, int16 len) {
	return unpack8(indata, len, outdata);
}
