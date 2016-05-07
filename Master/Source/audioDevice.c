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
#include "utils.h"

#include <string.h>
#include <AppHardwareApi.h>

#include "common.h"
#include "audioManager.h"
#include "audioDevice.h"


/*
 * 各種定義
 */
#undef AD_DEBUG_SYNC_OUT // PORT_OUT3,4 にSYNC同期時の同期タイミングを出力

/*
 * 各種変数または宣言
 */
static tsAD_Conf sConf; // 設定情報

static tsTimerContext sTimerPWM; // 主出力
static tsTimerContext sTimerPWM_Thru; // パススルー出力
static tsTimerContext sTimerSample; // サンプリング周期単位で動作するタイマー

static int32 i32SampLast; // 直前のサンプル (簡易LPF)
static int32 i32SampGet;  // 確定したサンプル値
static uint16 u16CtThres; // PWM同期更新時の更新タイミング

static void vAD_ISR(uint32, uint32); // 周期割り込みハンドラー

#ifdef AD_DEBUG_SYNC_OUT
static uint32 u32AD_SyncCallBack(uint32 u32Stat);
static uint32 u32AD_SyncCallBackPassThru(uint32 u32Stat);
#endif

/**
 * オーディオデバイスの初期化を行う
 * @param u32XTAL_FREQ
 */
void vAD_Init(tsAD_Conf *pConf) {
	tsTimerContext *pCtx;

	// 設定データをコピー
	if (pConf) {
		sConf = *pConf;
	} else {
		memset(&sConf, 9, sizeof(sConf));
		pConf = &sConf;
	}

	// タイマーのポート割り当てを変更する
	vAHI_TimerSetLocation(E_AHI_TIMER_1, TRUE, TRUE);

	memset(&sTimerPWM, 0, sizeof(tsTimerContext));
	memset(&sTimerPWM_Thru, 0, sizeof(tsTimerContext));
	memset(&sTimerSample, 0, sizeof(tsTimerContext));

	// PWM (無線経由)
	pCtx = &sTimerPWM;
	(*pCtx).u16Hz =  (pConf && pConf->u16PWM_Freq) ? pConf->u16PWM_Freq : AUDIO_DEV_PWM_FREQ;
	(*pCtx).u8PreScale = 0;
	(*pCtx).u16duty = 512;
	(*pCtx).bPWMout = TRUE;
	(*pCtx).bDisableInt = TRUE;

	(*pCtx).u8Device = E_AHI_DEVICE_TIMER1;

	vTimerConfig(pCtx);
	vTimerStart(pCtx);
	u16CtThres = (*pCtx).u16ct_total >> 6; // トータルカウントの 1.5% 分が同期更新用のマージン

	// PWM (パススルー)
	pCtx = &sTimerPWM_Thru;
	(*pCtx).u16Hz = (pConf && pConf->u16PWM_Freq) ? pConf->u16PWM_Freq : AUDIO_DEV_PWM_FREQ;
	(*pCtx).u8PreScale = 0;
	(*pCtx).u16duty = 512;
	(*pCtx).bPWMout = TRUE;
	(*pCtx).bDisableInt = TRUE;

	(*pCtx).u8Device = E_AHI_DEVICE_TIMER2;

	vTimerConfig(pCtx);
	vTimerStart(pCtx);

	// サンプル処理用の割り込み
	pCtx = &sTimerSample;
	(*pCtx).u16Hz =  (pConf && pConf->u16Samp_Freq) ? pConf->u16Samp_Freq : AUDIO_DEV_SAMPLE_FREQ; // サンプル周期は 8000 Hz
	if (sConf.bDoubleSample) {
		// 倍サンプル処理時
		(*pCtx).u16Hz *= 2;
	}
	(*pCtx).u8PreScale = 0;
	(*pCtx).u16duty = 512;
	(*pCtx).bPWMout = TRUE;
	(*pCtx).bDisableInt = FALSE; // 割り込みは有効
	(*pCtx).pvHandler = vAD_ISR; // 割り込みハンドラ

	(*pCtx).u8Device = E_AHI_DEVICE_TIMER3;

	vTimerConfig(pCtx);
	vTimerStart(pCtx);

	// ADC を開始する
	if (!bAHI_APRegulatorEnabled()) {
		vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
						 E_AHI_AP_INT_DISABLE,     // 割り込み無し
						 E_AHI_AP_SAMPLE_4,        // 4サンプル平均
						 E_AHI_AP_CLOCKDIV_250KHZ, // 250kHz 動作
						 E_AHI_AP_INTREF);

		while(!bAHI_APRegulatorEnabled());
	}

	vAHI_AdcEnable(
			E_AHI_ADC_CONTINUOUS,   // 連続モード（値の読み出しはサンプリング周期のタイマー割り込み）
			E_AHI_AP_INPUT_RANGE_2, // 0-2.4V まで
			E_AHI_ADC_SRC_ADC_2);   // SMDでパターンを引きやすいADC2

	vAHI_AdcStartSample();  // 開始！
}

/**
 * オーディオデバイスへのサンプルの入出力を行う
 * ※ 割り込みハンドラ内から呼び出す。
 * @param int16 入力サンプル値 (16bit で正規化しておくこと)
 * @return 読みだしたサンプル値 (16bit で正規化しておくこと)
 */
int16 i16AD_ReadWrite(int16 i16SampInput) {
	// ＡＤの読み取り
	int32 i32AdcSamp = i32SampGet;

	// 出力 DUTY の変更
	int32 i32Duty = ((int32)i16SampInput + 32768L - 12*64) >> 6;
		// 0...1023 に変換した結果を -12...1011 に変換し 0<, 1000> を切り捨てる。
		// 結果、12分のDCオフセットと 1001～1024 分が利用されない空白と出来る。
		// この空白期間を、同期データ入力時のマージンとする。
	if (i32Duty < 0) i32Duty = 0;
	if (i32Duty > 1000) i32Duty = 1000;
	sTimerPWM.u16duty = (uint16)i32Duty;
	if (sConf.bSync) {
#ifdef AD_DEBUG_SYNC_OUT
		vTimerChangeDutySync(&sTimerPWM, u16CtThres, u32AD_SyncCallBack);
#else
		vTimerChangeDutySync(&sTimerPWM, u16CtThres, NULL);
#endif
	} else {
		vTimerChangeDuty(&sTimerPWM);
	}

	// パススルー出力 DUTY の変更
	sTimerPWM_Thru.u16duty = (uint16)i32AdcSamp; // ADC が 10bit なので、そのまま投入
	if (sConf.bSyncPassThru) {
#ifdef AD_DEBUG_SYNC_OUT
		vTimerChangeDutySync(&sTimerPWM_Thru, u16CtThres, u32AD_SyncCallBackPassThru);
#else
		vTimerChangeDutySync(&sTimerPWM_Thru, u16CtThres, NULL);
#endif
	} else {
		vTimerChangeDuty(&sTimerPWM_Thru);
	}

	// AD 読み取り値を int16 スケールに変換して戻す
	return (int16)((i32AdcSamp << 6) - 32736);
		// 32736の根拠： 0-1023 を 64 倍した場合 0-65472 の値域を取るため、その半値を０とした。
}

#ifdef AD_DEBUG_SYNC_OUT
/**
 * PWM への同期時のコールバック
 * PORT_OUT3 を同期タイミングで LO にする
 *
 * @param u32dev
 * @param u32bm
 */
static uint32 u32AD_SyncCallBack(uint32 u32Stat) {
	vPortSet_TrueAsLo(PORT_OUT3, !u32Stat);
	return 0;
}

/**
 * PWM への同期時のコールバック
 * PORT_OUT4 を同期タイミングで LO にする
 *
 * @param u32dev
 * @param u32bm
 */
static uint32 u32AD_SyncCallBackPassThru(uint32 u32Stat) {
	vPortSet_TrueAsLo(PORT_OUT4, !u32Stat);
	return 0;
}
#endif

/**
 * 割り込みハンドラ
 */
void vAD_ISR(uint32 u32dev, uint32 u32bm) {
	static uint32 u32ct;

	int32 i32AdcSamp = (int32)u16AHI_AdcRead(); // 10bit

	if (sConf.bDoubleSample) {
		if (u32ct & 1) {
			i32SampGet = (i32SampLast + i32AdcSamp) / 2;
			vAM_ISR(u32dev, u32bm);
		} else {
			i32SampLast = i32AdcSamp;
			i32SampGet = 0;
		}
	} else {
		i32SampGet = i32AdcSamp;
		vAM_ISR(u32dev, u32bm);
	}

	u32ct++;
}
