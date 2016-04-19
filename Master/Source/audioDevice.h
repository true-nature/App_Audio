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

#ifndef AUDIODEVICE_H_
#define AUDIODEVICE_H_

#include <jendefs.h>

/**
 * PWM出力の周波数
 *   32000Hz の場合、500カウントで１周期であるため約9bitの分解能が得られる
 *   16000Hz はノイズの影響が大きいためＮＧとした
 */
#define AUDIO_DEV_PWM_FREQ 32000

/**
 * サンプル周波数
 */
#define AUDIO_DEV_SAMPLE_FREQ 8000

typedef struct {
	uint16 u16Samp_Freq;
	uint16 u16PWM_Freq;

	bool_t bDoubleSample;
	bool_t bSync;
	bool_t bSyncPassThru;
} tsAD_Conf;
/**
 * オーディオデバイスへ１サンプルを読み書きする
 * @param int16
 * @return
 */
int16  i16AD_ReadWrite(int16);

/**
 * オーディオデバイスの初期化
 */
void vAD_Init(tsAD_Conf *);

/**
 * オーディオサンプルを処理する割り込み処理部
 *
 * @param u32d
 * @param u32m
 */
extern void vAM_ISR(uint32 u32d, uint32 u32m);

#endif /* AUDIODEVICE_H_ */
