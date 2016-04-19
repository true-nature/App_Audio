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

#ifndef RAW_H_
#define RAW_H_

void CODEC_RAW_vInit(tsCODEC *psCodec);
void CODEC_RAW8_vInit(tsCODEC *psCodec);
uint16 CODEC_RAW_u16Encoder(tsCODEC *psCodec, int16 *indata, uint8 *outdata, int16 len);
uint16 CODEC_RAW_u16Decoder(tsCODEC *psCodec, uint8 *indata, int16 *outdata, int16 len);
uint16 CODEC_RAW8_u16Encoder(tsCODEC *psCodec, int16 *indata, uint8 *outdata, int16 len);
uint16 CODEC_RAW8_u16Decoder(tsCODEC *psCodec, uint8 *indata, int16 *outdata, int16 len);

uint8 CODEC_RAW_u8SetStatus(tsCODEC *psCodec, uint8 *pu8statinfo);
uint8* CODEC_RAW_pu8GetStatus(tsCODEC *psCodec, uint8 *pu8len);

#endif /* RAW_H_ */
