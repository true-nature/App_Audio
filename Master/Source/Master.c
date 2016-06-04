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
#include <string.h>
#include <AppHardwareApi.h>

#include "Master.h"

#include "ccitt8.h"
#include "Interrupt.h"

#include "Version.h"

#include "utils.h"
#include "flash.h"

#include "common.h"
#include "config.h"

// IO Read Options
#include "btnMgr.h"

// 重複チェッカ
#include "duplicate_checker.h"

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "modbus_ascii.h"
#include "input_string.h"

#include "Interactive.h"

#include "audioManager.h"
#include "audioDevice.h"
#include "CodecPreBuffer.h"

#include "codec.h"
#include "IMA_adpcm.h"
#include "RAW.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
//#define ToCoNet_USE_MOD_CHANNEL_MGR

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

// 実験的な実装
#include "Experimental.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

//!< 起床アイドルカウンタ初期値 (64FPS)
#define WAKEUP_DURATION_COUNT 2
//!< 最長アイドル時間
#define MAX_IDLE_TIME_sec 5
//!< 64FPSのイベント毎に行うアイドル状態カウントダウンの初期値
#define IDLE_COUNT_RESET_VALUE (MAX_IDLE_TIME_sec * 64)
//!< カソードコモンの2色LEDを使うのでHighで点灯
#define POSITIVE_LOGIC_LED 1

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vProcessEvCoreSub(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vProcessEvCorePairing(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vSendPairingRequest(uint32 u32State);

static void vInitHardware(int f_warm_start);

static void vSerialInit(uint32, tsUartOpt *);
void vProcessSerialCmd(tsModbusCmd *pSer);
void vSerInitMessage();

//static void vRxEvent_Transceiver(tsRxDataApp *psRx);
//static void vRxEvent_Pairing(tsRxDataApp *psRx);

static bool_t bCheckDupPacket(tsDupChk_Context *pc, uint32 u32Addr, uint8 u8Seq);

static void vReceiveAudioData(tsRxDataApp *pRx);
static void vReceivePairingData(tsRxDataApp *pRx);

static int16 i16TransmitRepeat(tsRxDataApp *pRx);
static int16 i16TransmitAudioData(uint8 *, uint8);
static int16 i16TransmitPairingRequest(uint32 u32State);

static void vProcessAudio();

static void vInitCodecRaw();
static void vInitCodecRaw8();
static void vInitCodecIMA_ADPCM();

static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern tsModbusCmd sSerCmd;
extern uint16 u16HoldUpdateScreen;
extern tsInpStr_Context sSerInpStr;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsAppData sAppData; //!< アプリケーションデータ  @ingroup MASTER

tsFILE sSerStream; //!< シリアル出力ストリーム  @ingroup MASTER
tsSerialPortSetup sSerPort; //!< シリアルポートの設定  @ingroup MASTER

tsTimerContext sTimerApp; //!< タイマー管理構造体  @ingroup MASTER

uint8 au8SerOutBuff[128]; //!< シリアルの出力書式のための暫定バッファ  @ingroup MASTER

tsDupChk_Context sDupChk; //!< 重複チェック(IO関連のデータ転送)  @ingroup MASTER

tsImaAdpcmState sIMAadpcm_state_Enc; //!< ADPCM エンコーダーの状態ベクトル  @ingroup MASTER
tsImaAdpcmState sIMAadpcm_state_Dec; //!< ADPCM デコーダーの状態ベクトル @ingroup MASTER

static uint16 sIdleCountDown;	//!< アイドル状態を監視するカウンター

//static void (*sRxEventHandler)(tsRxDataApp *psRx) = vRxEvent_Transceiver;

/****************************************************************************/
/***        FUNCTIONS                                                     ***/
/****************************************************************************/

/** @ingroup MASTER
 * アプリケーションの基本制御状態マシン。
 * - 特別な処理は無い。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			/// 始動メッセージ
			// 始動メッセージの表示
			if (!(u32evarg & EVARG_START_UP_WAKEUP_MASK)) {
				vSerInitMessage();
			}

			/// オーディオ関連の初期化
			vAM_StartStopSampling(FALSE);
			// サンプルバッファーの登録
			CodecPreBuffer_vInit();
			// オーディオバッファの初期化
			tsAM_Conf sAM_Conf;
			memset(&sAM_Conf, 0, sizeof(sAM_Conf));
			sAM_Conf.bInpLPF = IS_APPCONF_OPT_INPUT_LPF();
			sAM_Conf.bOutLPF = IS_APPCONF_OPT_OUTPUT_LPF();
			// CODEC & AM
			switch (sAppData.sFlash.sData.u8codec) {
			case 0:
			default:
				sAM_Conf.u16SampleBufferSize = 160; // 80bytes payload
				vInitCodecIMA_ADPCM();
				break;
			case 1:
				sAM_Conf.u16SampleBufferSize = 64; // 10bit/sample (80bytes = 640bits)
				vInitCodecRaw();
			case 2:
				sAM_Conf.u16SampleBufferSize = 80; // 8bit/sample
				vInitCodecRaw8();
				break;
			}
			vAM_Init(&sAM_Conf);

			if (sAppData.bWakeupByButton || IS_LOGICAL_ID_PARENT(au8IoModeTbl_To_LogicalID[sAppData.u8Mode])) {
				sIdleCountDown = IDLE_COUNT_RESET_VALUE;
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			} else {
				sIdleCountDown = WAKEUP_DURATION_COUNT;
			}
		}
		else if (eEvent == E_EVENT_APP_TICK_A) {
			// アイドル状態の監視
			if (sIdleCountDown > WAKEUP_DURATION_COUNT) {
				/// RUNNING 状態へ遷移
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			}
			else if (sIdleCountDown > 0) {
				sIdleCountDown--;
				if (sIdleCountDown == 0) {
					/// SLEEP 状態へ遷移
					ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
				}
			}
		}
		break;

	case E_STATE_RUNNING:
		if (eEvent == E_EVENT_NEW_STATE) {
			// OPAMPの電源を投入
			vPortSetHi(PORT_OUT3);
			vPortSetLo(PORT_OUT4);

			// オーディオデバイス (ADC/PWM の初期化)
			tsAD_Conf sAD_Conf;
			memset(&sAD_Conf, 0, sizeof(tsAD_Conf));
			sAD_Conf.u16PWM_Freq = sAppData.sFlash.sData.u32PWM_Hz;
			sAD_Conf.u16Samp_Freq = sAppData.sFlash.sData.u16Samp_Hz;
			sAD_Conf.bSync = (IS_APPCONF_OPT_OUTPUT_SYNC_PWM() != 0);
			sAD_Conf.bSyncPassThru = (IS_APPCONF_OPT_OUTPUT_SYNC_PWM_PASS_THRU() != 0);
			sAD_Conf.bDoubleSample = (IS_APPCONF_OPT_INPUT_DOUBLE_SAMPLE() != 0);
			vAD_Init(&sAD_Conf);

			// 開始
			vAM_StartStopSampling(TRUE);
		} else if (eEvent == E_EVENT_APP_TICK_A) {
			// 親機(設定モード)はスリープしない。子機はアイドル時に強制スリープ。
			if (IS_LOGICAL_ID_CHILD(au8IoModeTbl_To_LogicalID[sAppData.u8Mode])) {
				// アイドル状態の監視
				if (sIdleCountDown > 0) {
					sIdleCountDown--;
				} else {
					// 停止
					vAM_StartStopSampling(FALSE);
					/// SLEEP 状態へ遷移
					ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
				}
			}
		}

		break;

	case E_STATE_FINISHED:
		_C {
			if (eEvent == E_EVENT_NEW_STATE) {
				vfPrintf(&sSerStream, "!INF SLEEP %dms @%dms."LB,
						sAppData.u32SleepDur, u32TickCount_ms);
				SERIAL_vFlush(sSerStream.u8Device);
#ifdef POSITIVE_LOGIC_LED
				vPortSetLo(PORT_OUT1);
				vPortSetLo(PORT_OUT2);
#else
				vPortSetHi(PORT_OUT1);
				vPortSetHi(PORT_OUT2);
#endif
				// OPAMPの電源を遮断
				vPortSetLo(PORT_OUT3);
				vPortSetHi(PORT_OUT4);
				pEv->bKeepStateOnSetAll = FALSE;
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEPING);
			}
		}
		break;

	case E_STATE_APP_SLEEPING:
		if (eEvent == E_EVENT_NEW_STATE) {
			vSleep(sAppData.u32SleepDur, TRUE, FALSE);
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * アプリケーション制御（電源常時 ON モード）
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCoreSub(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {

	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			// RUNNING 状態へ遷移
			ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		}
		break;

	case E_STATE_RUNNING:
		if (sAppData.u8Mode == E_IO_MODE_ROUTER) {
			break; // リピーターは何もしない。
		}

		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * アプリケーション制御（AutoPairing モード）
 * - 機能概要
 *   - 起動時にランダムで処理を保留する（同時起動による送信パケットの競合回避のため）
 *   - 実行状態では E_EVENT_APP_TICK_A (64fps タイマーイベント) を起点に処理する。
  *     - 定期パケット送信後は、次回のタイミングを乱数によってブレを作る。
 *
 * - 状態一覧
 *   - E_STATE_IDLE\n
 *     起動直後に呼び出される状態で、同時起動によるパケット衝突を避けるためランダムなウェイトを置き、次の状態に遷移する。
 *   - E_STATE_APP_PAIR_SCAN\n
 *     ペアリング相手が現れるのを待つ。
 *   - E_STATE_APP_PAIR_PROPOSE\n
 *     送受信を1秒間継続し、AppIdとChannelの合意を形成する。
 *   - E_STATE_APP_PAIR_CONFIRM\n
 *     合意したAppIdとChannelにRF設定を切り替える。
 *     送受信を1秒間継続し、AppIdとChannelの合意を確認する。
 *   - E_STATE_APP_PAIR_COMPLETE\n
 *     合意したAppIdとChannelをflashに保存する。
 *   - E_STATE_APP_PAIR_FAILED\n
 *     リセットする。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCorePairing(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			vfPrintf(&sSerStream, "!INF ENTERED AUTO PAIRING MODE.@%dms"LB, u32TickCount_ms);
			vPortSetLo(PORT_OUT1);
			vPortSetLo(PORT_OUT2);
			vPortSetLo(PORT_OUT3);
			vPortSetHi(PORT_OUT4);

			sAppData.u16CtRndCt = 0;

			sAppData.u32ReqAppId = ToCoNet_u32GetSerial();  // 要求APP ID
			// 要求channel
			if (sAppData.u8PairingMode == 3) {
				// 子機追加モードの親機はchannel不変
				sAppData.u8ReqCh = sAppData.sFlash.sData.u8ch;
			} else {
				// ランダムに決定
				sAppData.u8ReqCh = ((ToCoNet_u16GetRand() & 0xF) + 11);
			}
			sAppData.u32CandidateAppId = 0;
			sAppData.u32AnotherAppId = 0;
			sAppData.u8CandidateCh = 0;
			sAppData.u16MatchCount = 0;
			sAppData.u16PeerMatched = 0;
		}

		if (eEvent == E_EVENT_TICK_TIMER) {
			if (!sAppData.u16CtRndCt) {
				sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0xFF) + 10; // 始動時にランダムで少し待つ（同時電源投入でぶつからないように）
			}

			// 始動時ランダムな待ちを置く
			if (sAppData.u16CtRndCt
					&& PRSEV_u32TickFrNewState(pEv) > sAppData.u16CtRndCt) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_SCAN);
				sAppData.u16CtRndCt = 0;
			}
		}

		break;

	case E_STATE_APP_PAIR_SCAN:
		if (eEvent == E_EVENT_APP_TICK_A  // 秒64回のタイマー割り込み
				&& (sAppData.u32CtTimer0 & 1) // 秒32回にする
				) {
			vSendPairingRequest(pEv->eState);
			const uint32 mask = 0x7F;
			const uint32 duty = 1;
			vPortSet_TrueAsLo(PORT_OUT1, (sAppData.u32CtTimer0 & mask) > duty);
		}
		// ペアリング相手が現れたら提案確認フェーズ
		if (eEvent == E_EVENT_TICK_TIMER) {
			if (0 < sAppData.u16MatchCount) {
				vfPrintf(&sSerStream, "!INF PEER EXIST.@%dms"LB, u32TickCount_ms);
				ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_PROPOSE);
			}
			else if (30000 <= PRSEV_u32TickFrNewState(pEv)) {
				// 30秒待って相手が現れなければ諦める
				ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_FAILED);
			}
		}
		break;

	case E_STATE_APP_PAIR_PROPOSE:
		if (eEvent == E_EVENT_APP_TICK_A  // 秒64回のタイマー割り込み
				&& (sAppData.u32CtTimer0 & 1) // 秒32回にする
				) {
			vSendPairingRequest(pEv->eState);
			const uint32 mask = 0x1F;
			const uint32 duty = 1;
			vPortSet_TrueAsLo(PORT_OUT1, (sAppData.u32CtTimer0 & mask) > duty);
		}
		if (1000 <= PRSEV_u32TickFrNewState(pEv)) {
			// 1秒待ってから判断
			sToCoNet_AppContext.bRxOnIdle = FALSE;
			ToCoNet_vRfConfig();	// 受信を一旦停止
			vfPrintf(&sSerStream, "!INF u16MatchCount=%d u32CandidateAppId:%08x ch:%d"LB, sAppData.u16MatchCount, sAppData.u32CandidateAppId, sAppData.u8CandidateCh);
			if (AUTO_PAIR_COUNT_MIN <= sAppData.u16MatchCount
					&& AUTO_PAIR_COUNT_MIN <= sAppData.u16PeerMatched) {
				vfPrintf(&sSerStream, "!INF TRY NEW ID/CH SETTINGS.@%dms"LB, u32TickCount_ms);
				// カウンタを一旦クリア
				sAppData.u16MatchCount = 0;
				sAppData.u16PeerMatched = 0;
				// u32AppIdはcbAppColdStart以外で変更不可なのでu8AppIdentifierだけを変更
				sAppData.u8AppIdentifier = u8CCITT8(
						(uint8*) &sAppData.u32CandidateAppId, 4);
				sToCoNet_AppContext.u8Channel = sAppData.u8CandidateCh;
				sToCoNet_AppContext.u32ChMask = (1UL << sAppData.u8CandidateCh);
				// 次の定期パケットのタイミングを仕込む
				sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0x3);
				sToCoNet_AppContext.bRxOnIdle = TRUE;
				ToCoNet_vRfConfig();	// 新たなRF設定に切り替える
				ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_CONFIRM);
			} else {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_FAILED);
			}
		}
		break;

	case E_STATE_APP_PAIR_CONFIRM:
		if (eEvent == E_EVENT_APP_TICK_A  // 秒64回のタイマー割り込み
					&& (sAppData.u32CtTimer0 & 1) // 秒32回にする
					) {
			vSendPairingRequest(pEv->eState);
			const uint32 mask = 0xF;
			const uint32 duty = 1;
			vPortSet_TrueAsLo(PORT_OUT1, (sAppData.u32CtTimer0 & mask) > duty);
		}
		else if (eEvent == E_EVENT_TICK_TIMER) {
			if (1000 <= PRSEV_u32TickFrNewState(pEv)) {
				// 1秒待ってから判断
				sToCoNet_AppContext.bRxOnIdle = FALSE;
				ToCoNet_vRfConfig();	// 受信を停止
				vfPrintf(&sSerStream, "!INF u16MatchCount=%d u32CandidateAppId:%08x ch:%d"LB, sAppData.u16MatchCount, sAppData.u32CandidateAppId, sAppData.u8CandidateCh);
				if (AUTO_PAIR_COUNT_MIN <= sAppData.u16MatchCount
						&& AUTO_PAIR_COUNT_MIN <= sAppData.u16PeerMatched) {
					ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_COMPLETE);
				} else {
					ToCoNet_Event_SetState(pEv, E_STATE_APP_PAIR_FAILED);
				}
			}
		}
		break;

	case E_STATE_APP_PAIR_COMPLETE:
		// AppId,Chを書き換えて保存
		if (eEvent == E_EVENT_NEW_STATE) {
			vfPrintf(&sSerStream, "!INF SAVE NEW ID/CH SETTINGS.@%dms"LB, u32TickCount_ms);
			tsFlash sFlash = sAppData.sFlash;
			sFlash.sData.u32appid = sAppData.u32CandidateAppId;
			sFlash.sData.u8ch = sAppData.u8CandidateCh;
			sFlash.sData.u32chmask = (1UL << sAppData.u8CandidateCh);
			sFlash.sData.u32appkey = APP_ID;
			sFlash.sData.u32ver = VERSION_U32;
			bool_t bRet = bFlash_Write(&sFlash, FLASH_SECTOR_NUMBER - 1, 0);
			V_PRINT("!INF FlashWrite %s"LB, bRet ? "Success" : "Failed");
			vWait(100000);
		}
		// no break

	case E_STATE_APP_PAIR_FAILED:
		// 諦めてリセット
		if (eEvent == E_EVENT_NEW_STATE) {
			V_PRINT("!INF RESET SYSTEM.");
			vPortSetLo(PORT_OUT1);
			vWait(1000000);
			vAHI_SwReset();
		}
		break;

	default:
		break;
	}
}

/**
 * ペアリング要求を間歇的に送信
 * @param pEv
 */
void vSendPairingRequest(uint32 u32State) {
	if (sAppData.u16CtRndCt)
		sAppData.u16CtRndCt--;
	// 定期パケット送信までのカウントダウン
	// レギュラー送信
	if (sAppData.u16CtRndCt == 0) {
		// 送信要求
		i16TransmitPairingRequest(u32State);

		// 次の定期パケットのタイミングを仕込む
		sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0x3);
	}
}

/** @ingroup MASTER
 * 電源投入時・リセット時に最初に実行される処理。本関数は２回呼び出される。初回は u32AHI_Init()前、
 * ２回目は AHI 初期化後である。
 *
 * - 各種初期化
 * - ToCoNet ネットワーク設定
 * - 設定IO読み取り
 * - 緊急フラッシュ消去処理
 * - 設定値の計算
 * - ハードウェア初期化
 * - イベントマシンの登録
 * - 本関数終了後は登録したイベントマシン、および cbToCoNet_vMain() など各種コールバック関数が
 *   呼び出される。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppColdStart(bool_t bStart) {
	if (!bStart) {
		// before AHI initialization (very first of code)

		// Module Registration
		ToCoNet_REG_MOD_ALL();
	} else {
		// メモリのクリア
		memset(&sAppData, 0x00, sizeof(sAppData));
		vConfig_UnSetAll(&sAppData.sConfig_UnSaved);

		// デフォルトのネットワーク指定値
		sToCoNet_AppContext.u8TxMacRetry = 3; // MAC再送回数（JN516x では変更できない）
		sToCoNet_AppContext.u32AppId = APP_ID; // アプリケーションID
		sToCoNet_AppContext.u8Channel = CHANNEL; // デフォルトのチャネル
		//sToCoNet_AppContext.u32ChMask = CHMASK;

		sToCoNet_AppContext.u16TickHz = 1000; // 1KHz で制御
		sToCoNet_AppContext.bRxOnIdle = TRUE;

		// フラッシュの読み出し
		sAppData.bFlashLoaded = bConfig_Load(&sAppData.sFlash);

		// フラッシュ設定値の反映
		if (sAppData.bFlashLoaded) {
			sToCoNet_AppContext.u32AppId = sAppData.sFlash.sData.u32appid;
			sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;
			int i;
			for (i = 11; i <= 26; i++) {
				if (sAppData.sFlash.sData.u32chmask & (1UL << i)) {
					sToCoNet_AppContext.u8Channel = i;
					break;
				}
			}
			sToCoNet_AppContext.u8TxPower = sAppData.sFlash.sData.u8pow; // 出力の設定
		}

		// フラッシュのロードに失敗したとき
		if (sAppData.bFlashLoaded != TRUE) {
			// 構造体にデフォルト値を格納する
			vConfig_SetDefaults(&(sAppData.sFlash.sData));
		}
		// ヘッダの１バイト識別子を AppID から計算
		sAppData.u8AppIdentifier = u8CCITT8(
				(uint8*) &sToCoNet_AppContext.u32AppId, 4); // APP ID の CRC8

		// IOより状態を読み取る (ID など)
		sAppData.u32DIO_startup = ~u32PortReadBitmap(); // この時点では全部入力ポート

		// version info
		sAppData.u32ToCoNetVersion = ToCoNet_u32GetVersion();

		// ToCoNet の制御 Tick [ms]
		sAppData.u16ToCoNetTickDelta_ms = 1000 / sToCoNet_AppContext.u16TickHz;

		// その他ハードウェアの初期化
		vInitHardware(FALSE);

		// 論理IDの設定チェック、その他設定値のチェック
		//  IO の設定を優先し、フラッシュ設定で矛盾するものについてはデフォルト値を書き直す。
		if (IS_LOGICAL_ID_CHILD(au8IoModeTbl_To_LogicalID[sAppData.u8Mode])) {
			// 子機IDはフラッシュ値が設定されていれば、これを採用
			if (sAppData.bFlashLoaded) {
				sAppData.u8AppLogicalId = sAppData.sFlash.sData.u8id;
			}

			if (!IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId )) {
				sAppData.u8AppLogicalId =
						au8IoModeTbl_To_LogicalID[sAppData.u8Mode];
			}
		}

		// 論理IDを121,122に保存した場合、親機で起動する
		if (sAppData.bFlashLoaded) {
			if (sAppData.sFlash.sData.u8id == 121) {
				sAppData.u8Mode = 1; // 親機のモード番号
				sAppData.u8AppLogicalId =
						au8IoModeTbl_To_LogicalID[sAppData.u8Mode]; // 論理IDを設定
			}
		}

		// 各モード依存の初期値の設定など
		switch (sAppData.u8Mode) {
		case E_IO_MODE_PARNET:
			sAppData.u8AppLogicalId = LOGICAL_ID_PARENT;
			break;

		case E_IO_MODE_ROUTER:
			sAppData.u8AppLogicalId = LOGICAL_ID_REPEATER;
			break;

		case E_IO_MODE_CHILD:
			break;

		default: // 未定義機能なので、SILENT モードにする。
			sAppData.u8AppLogicalId = 255;
			break;
		}

		// ショートアドレスの設定(決めうち)
		sToCoNet_AppContext.u16ShortAddress =
				SERCMD_ADDR_CONV_TO_SHORT_ADDR(sAppData.u8AppLogicalId);

		// 間欠動作のスリープ時間
		if (!sAppData.u32SleepDur) {
			if (sAppData.bFlashLoaded) {
				sAppData.u32SleepDur = sAppData.sFlash.sData.u16SleepDur_ms;
			} else {
				sAppData.u32SleepDur = MODE4_SLEEP_DUR_ms;
			}
		}

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk); // 重複チェック用

		// LED消灯
#ifdef POSITIVE_LOGIC_LED
		vPortSetLo(PORT_OUT1);
		vPortSetLo(PORT_OUT2);
#else
		vPortSetHi(PORT_OUT1);
		vPortSetHi(PORT_OUT2);
#endif
		// OPAMPの電源を遮断
		vPortSetLo(PORT_OUT3);
		vPortSetHi(PORT_OUT4);

		sAppData.bPktMon = TRUE;

		sToCoNet_AppContext.bRxOnIdle = TRUE;

		if (sAppData.u8PairingMode) {
			if (sAppData.u8PairingMode & 2) {
				sAppData.u8AppLogicalId = LOGICAL_ID_PARENT;
			} else {
				sAppData.u8AppLogicalId = LOGICAL_ID_CHILDREN;
			}
			ToCoNet_Event_Register_State_Machine(vProcessEvCorePairing); // Auto Pairingの処理
			sAppData.prPrsEv = (void*) vProcessEvCorePairing;
			sToCoNet_AppContext.u8TxPower = 0;	// 最小出力
			sToCoNet_AppContext.u32AppId = APP_ID;
			sAppData.u8AppIdentifier = u8CCITT8(
					(uint8*) &sToCoNet_AppContext.u32AppId, 4); // APP ID の CRC8
			sToCoNet_AppContext.u8Channel = CHANNEL; // pairing用に固定
			sToCoNet_AppContext.u32ChMask = CHMASK;
		} else {
			ToCoNet_Event_Register_State_Machine(vProcessEvCore);
			sAppData.prPrsEv = (void*) vProcessEvCore;
		}

		// MAC の初期化
		ToCoNet_vMacStart();

		// 主状態遷移マシンの登録
		sAppData.u8Hnd_vProcessEvCore = ToCoNet_Event_Register_State_Machine(vProcessEvCoreSub);
	}
}

/** @ingroup MASTER
 * スリープ復帰後に呼び出される関数。\n
 * 本関数も cbAppColdStart() と同様に２回呼び出され、u32AHI_Init() 前の
 * 初回呼び出しに於いて、スリープ復帰要因を判定している。u32AHI_Init() 関数は
 * これらのレジスタを初期化してしまう。
 *
 * - 変数の初期化（必要なもののみ）
 * - ハードウェアの初期化（スリープ後は基本的に再初期化が必要）
 * - イベントマシンは登録済み。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppWarmStart(bool_t bStart) {
	if (!bStart) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.
		sAppData.bWakeupByButton = FALSE;
		if (u8AHI_WakeTimerFiredStatus()) {
			;
		} else if (u32AHI_DioWakeStatus()
				& ((1UL << PORT_INPUT1) | (1UL << PORT_INPUT2)
						| (1UL << PORT_INPUT3) | (1UL << PORT_INPUT4))) {
			// woke up from DIO events
			sAppData.bWakeupByButton = TRUE;
		}
	} else {
		vInitHardware(TRUE);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk);
		sToCoNet_AppContext.bRxOnIdle = TRUE;
		// LED消灯
#ifdef POSITIVE_LOGIC_LED
		vPortSetLo(PORT_OUT1);
		vPortSetLo(PORT_OUT2);
#else
		vPortSetHi(PORT_OUT1);
		vPortSetHi(PORT_OUT2);
#endif
		// OPAMPの電源を遮断
		vPortSetLo(PORT_OUT3);
		vPortSetHi(PORT_OUT4);

		// MAC の開始
		ToCoNet_vMacStart();
	}
}

/** @ingroup MASTER
 * 本関数は ToCoNet のメインループ内で必ず１回は呼び出される。
 * ToCoNet のメインループでは、CPU DOZE 命令を発行しているため、割り込みなどが発生した時に
 * 呼び出されるが、処理が無い時には呼び出されない。
 * しかし TICK TIMER の割り込みは定期的に発生しているため、定期処理としても使用可能である。
 *
 * - シリアルの入力チェック
 */
void cbToCoNet_vMain(void) {
	vHandleSerialInput(); // シリアルポートの処理
}

/** @ingroup MASTER
 * パケットの受信完了時に呼び出されるコールバック関数。\n
 * パケットの種別によって具体的な処理関数にディスパッチしている。
 * データ種別は psRx->u8Cmd (ToCoNet のパケットヘッダに含まれます) により識別される。
 *
 * - パケット種別
 *   - TOCONET_PACKET_CMD_APP_DATA : シリアル電文パケット
 *   - TOCONET_PACKET_CMD_APP_USER_PAIRING : ペアリング要求
 *
 * @param psRx 受信パケット
 */
void cbToCoNet_vRxEvent(tsRxDataApp *psRx) {
	//uint8 *p = pRx->auData;

	DBGOUT(3, "Rx packet (cm:%02x, fr:%08x, to:%08x)"LB, psRx->u8Cmd,
			psRx->u32SrcAddr, psRx->u32DstAddr);

	switch (psRx->u8Cmd) {
	case TOCONET_PACKET_CMD_APP_DATA: // データ到着
		vReceiveAudioData(psRx);
		break;
	case TOCONET_PACKET_CMD_APP_USER_PAIRING:	// auto pairing
		if (sAppData.u8PairingMode && PRSEV_eGetStateH(sAppData.u8Hnd_vProcessEvCore) == E_STATE_RUNNING) { // 稼動状態でパケット処理をする
			vReceivePairingData(psRx);
		}
		break;
	}
}

/** @ingroup MASTER
 * 送信完了時に呼び出されるコールバック関数。
 * (本アプリでは送信完了の取り扱いはしない)
 *
 * @param u8CbId 送信時に設定したコールバックID
 * @param bStatus 送信ステータス
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	//uint8 *q = au8SerOutBuff;

	return;
}

/** @ingroup MASTER
 * ネットワーク層などのイベントが通達される。\n
 * 本アプリケーションでは特別な処理は行っていない。
 *
 * @param ev
 * @param u32evarg
 */
void cbToCoNet_vNwkEvent(teEvent ev, uint32 u32evarg) {
	switch (ev) {
	case E_EVENT_TOCONET_NWK_START:
		break;

	case E_EVENT_TOCONET_NWK_DISCONNECT:
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * ハードウェア割り込み時に呼び出される。本処理は割り込みハンドラではなく、割り込みハンドラに登録された遅延実行部による処理で、長い処理が記述可能である。
 * 本アプリケーションに於いては、ADC/DIの入力状態のチェック、64fps のタイマーイベントの処理などを行っている。
 *
 * - E_AHI_DEVICE_SYSCTRL
 *   - DI割り込みの処理を行う。これは、低レイテンシモードでの処理である。
 *
 * - E_AHI_DEVICE_TICK_TIMER : このイベントは ToCoNet 組み込みで、ToCoNet の制御周期 (sToCoNet_AppContext.u16TickHz) を
 *   実現するためのタイマーです。ユーザが TickTimer の制御を変更したりすると ToCoNet は動作しなくなります。
 *
 *   - Di入力の変化の確認。変化が有れば、sAppData.sIOData_now 構造体に結果を格納する。
 *     低レイテンシモードの時は、この判定を起点として送信を行う。
 *
 * - E_AHI_DEVICE_TIMER0 : TICK_TIMER から分周して制御周期を作っても良いのですが、TIMER_0 を使用しています。
 *   - カウンタのインクリメント処理
 *   - ADC の完了確認
 *   - パケット重複チェックアルゴリズムのタイムアウト処理
 *   - DIのカウンタ処理 (インタラクティブモードでカウンタ終了時にもパケットを送信する処理を行う）
 *   - イベントマシンに TIMER0 イベントを発行
 *   - インタラクティブモード時の画面再描画
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	switch (u32DeviceId) {
	case E_AHI_DEVICE_SYSCTRL:
		break;

	case E_AHI_DEVICE_ANALOGUE: //ADC完了時にこのイベントが発生する。
		break;

	case E_AHI_DEVICE_TICK_TIMER: //比較的頻繁な処理
		if (!sAppData.u8PairingMode) {
			// オーディオ処理を行う。
			vProcessAudio();
		}
		break;

	case E_AHI_DEVICE_TIMER0:
		// タイマーカウンタをインクリメントする (64fps なので 64カウントごとに１秒)
		sAppData.u32CtTimer0++;

		// 重複チェックのタイムアウト処理
		if ((sAppData.u32CtTimer0 & 0xF) == 0) {
			DUPCHK_bFind(&sDupChk, 0, NULL);
		}

		if (!sAppData.u8PairingMode) {
			// イベント処理部分にイベントを送信
			ToCoNet_Event_Process(E_EVENT_APP_TICK_A, 0, (void*)vProcessEvCore);
		}
		// イベント処理部分にイベントを送信
		if (sAppData.prPrsEv && (sAppData.u32CtTimer0 & 1)) {
			ToCoNet_Event_Process(E_EVENT_APP_TICK_A, 0, sAppData.prPrsEv);
		}

		// シリアル画面制御のためのカウンタ
		if (!(--u16HoldUpdateScreen)) {
			if (sSerCmd.bverbose) {
				vSerUpdateScreen();
			}
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * 割り込みハンドラ。ここでは長い処理は記述してはいけない。
 *
 * - TIMER_0\n
 *   - JN514x での DAC 出力
 * - TICK_TIMER\n
 *   - ADCの実行管理
 *   - ボタン入力判定管理
 */
PUBLIC uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	uint8 u8handled = FALSE;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_TIMER0:
		break;

	case E_AHI_DEVICE_ANALOGUE:
		break;

	case E_AHI_DEVICE_TICK_TIMER:
		// ボタンハンドラの駆動
		if (sAppData.pr_BTM_handler) {
			// ハンドラを稼働させる
			(*sAppData.pr_BTM_handler)(sAppData.u16ToCoNetTickDelta_ms);
		}
		break;

	default:
		break;
	}

	return u8handled;
}

/** @ingroup MASTER
 * ハードウェアの初期化を行う。スリープ復帰時も原則同じ初期化手続きを行う。
 *
 * - 管理構造体のメモリ領域の初期化
 * - DO出力設定
 * - DI入力設定
 * - DI割り込み設定 (低レイテンシモード)
 * - M1-M3 の読み込み
 * - UARTの初期化
 * - ADC3/4 のプルアップ停止
 * - タイマー用の未使用ポートを汎用IOに解放する宣言
 * - 秒64回のTIMER0の初期化と稼働
 * - ADC/DAC(JN514x)/PWM の初期化
 * - I2Cの初期化
 *
 * @param f_warm_start TRUE:スリープ復帰時
 */
static void vInitHardware(int f_warm_start) {
	int i;

	// メモリのクリア
	memset(&sTimerApp, 0, sizeof(tsTimerContext));

	// 出力の設定
	{
		vPortSetLo(PORT_OUT1);
		vPortAsOutput(PORT_OUT1);
		vPortSetLo(PORT_OUT2);
		vPortAsOutput(PORT_OUT2);
		// OPAMPの電源を切る
		vPortSetLo(PORT_OUT3);
		vPortAsOutput(PORT_OUT3);
		// CE#をdisableにする
		vPortSetHi(PORT_OUT4);
		vPortAsOutput(PORT_OUT4);
	}

	// 入力の設定
	for (i = 0; i < 4; i++) {
		vPortAsInput(au8PortTbl_DIn[i]);
	}

	// モード設定
	vPortAsInput(PORT_CONF1);
	vPortAsInput(PORT_CONF2);
	vPortAsInput(PORT_CONF3);
	sAppData.u8Mode = (bPortRead(PORT_CONF1) | (bPortRead(PORT_CONF2) << 1)
			| (bPortRead(PORT_CONF3) << 2));
	if (!f_warm_start) {
		sAppData.u8PairingMode = 0;
		// 電源投入時にDI1がアサートされていたらペアリングモード
		if (bPortRead(PORT_INPUT1)) {
			sAppData.u8PairingMode |= 1;
		}
		if (bPortRead(PORT_INPUT2)) {
			sAppData.u8PairingMode |= 2;
		}
	}

	// UART 設定
	{
		vPortAsInput(PORT_BAUD);

		uint32 u32baud = bPortRead(PORT_BAUD) ? UART_BAUD_SAFE : UART_BAUD;
		tsUartOpt sUartOpt;

		memset(&sUartOpt, 0, sizeof(tsUartOpt));

		// BAUD ピンが GND になっている場合、かつフラッシュの設定が有効な場合は、設定値を採用する (v1.0.3)
		if (sAppData.bFlashLoaded && bPortRead(PORT_BAUD)) {
			u32baud = sAppData.sFlash.sData.u32baud_safe;
			sUartOpt.bHwFlowEnabled = FALSE;
			sUartOpt.bParityEnabled = UART_PARITY_ENABLE;
			sUartOpt.u8ParityType = UART_PARITY_TYPE;
			sUartOpt.u8StopBit = UART_STOPBITS;

			// 設定されている場合は、設定値を採用する (v1.0.3)
			switch (sAppData.sFlash.sData.u8parity) {
			case 0:
				sUartOpt.bParityEnabled = FALSE;
				break;
			case 1:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_ODD_PARITY;
				break;
			case 2:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
				break;
			}

			vSerialInit(u32baud, &sUartOpt);
		} else {
			vSerialInit(u32baud, NULL );
		}

	}

	// ADC3/4 のピンのプルアップを廃止する
#ifdef JN516x // v1.0.3
	vPortDisablePullup(0);
	vPortDisablePullup(1);
#endif

	// モード設定ピンで Lo になっているポートはプルアップ停止
	// Lo でない場合は、プルアップ停止をするとリーク電流が発生する
	// ※ 暗電流に神経質な mode4, 7 のみ設定する。
	if (sAppData.u8Mode == 4) {
		vPortDisablePullup(PORT_CONF3);
	} else if (sAppData.u8Mode == 7) {
		vPortDisablePullup(PORT_CONF1);
		vPortDisablePullup(PORT_CONF2);
		vPortDisablePullup(PORT_CONF3);
	}

	// タイマの未使用ポートの解放（汎用ＩＯとして使用するため）
#ifdef JN516x
	vAHI_TimerFineGrainDIOControl(0x7); // bit 0,1,2 をセット (TIMER0 の各ピンを解放する, PWM1..4 は使用する)
#endif

	// 秒64回のTIMER0の初期化と稼働
	sTimerApp.u8Device = E_AHI_DEVICE_TIMER0;
	sTimerApp.u16Hz = 64;
	sTimerApp.u8PreScale = 4; // 15625ct@2^4
	vTimerConfig(&sTimerApp);
	vTimerStart(&sTimerApp);

	// button Manager (for Input)
	sAppData.sBTM_Config.bmPortMask = (1UL << PORT_INPUT1) | (1UL << PORT_INPUT2) | (1UL << PORT_INPUT3) | (1UL << PORT_INPUT4);

	sAppData.sBTM_Config.u16Tick_ms = 1;
	sAppData.sBTM_Config.u8MaxHistory = 5;
	sAppData.sBTM_Config.u8DeviceTimer = 0xFF; // TickTimer を流用する。
	sAppData.pr_BTM_handler = prBTM_InitExternal(&sAppData.sBTM_Config);
	vBTM_Enable();

}

/** @ingroup MASTER
 * RAW CODEC の初期化
 */
static void vInitCodecRaw() {
	CODEC_RAW_vInit(&sAppData.sEncode);
	CODEC_RAW_vInit(&sAppData.sDecode);
}

/** @ingroup MASTER
 * RAW CODEC の初期化
 */
static void vInitCodecRaw8() {
	CODEC_RAW8_vInit(&sAppData.sEncode);
	CODEC_RAW8_vInit(&sAppData.sDecode);
}

/** @ingroup MASTER
 * RAW CODEC の初期化
 */
static void vInitCodecIMA_ADPCM() {
	static tsImaAdpcmState sEnc, sDec;

	CODEC_IMA_Adpcm_vInit(&sAppData.sEncode, &sEnc);
	CODEC_IMA_Adpcm_vInit(&sAppData.sDecode, &sDec);
}

/** @ingroup MASTER
 * UART を初期化する
 * @param u32Baud ボーレート
 */
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[512];
	static uint8 au8SerialRxBuffer[512];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_MASTER;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	/* prepare stream for vfPrintf */
	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_MASTER;

	/* other initialization */
	INPSTR_vInit(&sSerInpStr, &sSerStream);
	memset(&sSerCmd, 0x00, sizeof(sSerCmd));

	static uint8 au8SerialBuffCmd[256];
	sSerCmd.au8data = au8SerialBuffCmd;
	sSerCmd.u16maxlen = sizeof(au8SerialBuffCmd);
}

/** @ingroup MASTER
 * 始動時メッセージの表示を行う。
 */
void vSerInitMessage() {
	vfPrintf(&sSerStream,
			LB"!INF TOCOS " APP_NAME " V%d-%02d-%d, SID=0x%08X, LID=0x%02x"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(),
			sAppData.u8AppLogicalId);
	if (sAppData.bFlashLoaded == 0) {
		vfPrintf(&sSerStream, "!INF Default config (no save info)..." LB);
	}
#if defined(JN514x)
	vfPrintf(&sSerStream, "!INF DIO --> %021b"LB, sAppData.u32DIO_startup);
#elif defined(JN516x)
	vfPrintf(&sSerStream, "!INF DIO --> %020b"LB, sAppData.u32DIO_startup);
#endif
}

/** @ingroup MASTER
 * シリアルから入力されたコマンド形式の電文を処理します。
 *
 * - 先頭バイトがアドレス指定で、0xDB 指定の場合、自モジュールに対しての指令となります。
 * - ２バイト目がコマンドで、0x80 以降を指定します。0x7F 以下は特別な処理は割り当てられていません。
 * - コマンド(0xDB向け)
 *   - SERCMD_ID_GET_MODULE_ADDRESS\n
 *     モジュールのシリアル番号を取得する
 *
 * @param pSer シリアルコマンド入力の管理構造体
 */
void vProcessSerialCmd(tsModbusCmd *pSer) {
	uint8 *p = pSer->au8data;

	uint8 u8addr; // 送信先論理アドレス
	uint8 u8cmd; // コマンド

	uint8 *p_end;
	p_end = &(pSer->au8data[pSer->u16len]); // the end points 1 byte after the data end.

	// COMMON FORMAT
	OCTET(u8addr); // [1] OCTET : 論理ID
	OCTET(u8cmd); // [1] OCTET : 要求番号

	DBGOUT(3, "* UARTCMD ln=%d cmd=%02x req=%02x %02x%0x2%02x%02x..." LB,
			pSer->u16len, u8addr, u8cmd, *p, *(p + 1), *(p + 2), *(p + 3));

	if (u8addr == SERCMD_ADDR_TO_MODULE) {
		/*
		 * モジュール自身へのコマンド (0xDB)
		 */
		switch (u8cmd) {
		case SERCMD_ID_GET_MODULE_ADDRESS:
			vModbOut_MySerial(&sSerStream);
			break;

		default:
			break;
		}
	} else {
	}
}

/** @ingroup MASTER
 * 重複パケットの判定。タイムスタンプの比較で、ごく最近であって旧いパケットを除外する。
 *
 * - 注意点
 *   - DUPCHK_vInin() の初期化を行うこと
 *   - DUPCHK_bFIND(0,NULL) を一定周期で呼び出すこと
 *
 * @param pc 管理構造体
 * @param u32Addr
 * @param u16TimeStamp
 * @return TRUE:重複している FALSE:新しいパケットが来た
 */
static bool_t bCheckDupPacket(tsDupChk_Context *pc, uint32 u32Addr,
		uint8 u8Seq) {
	uint32 u32Key;
	if (DUPCHK_bFind(pc, u32Addr, &u32Key)) {
		// 最後に受けたカウンタの値が得られるので、これより新しい
		uint8 u8Delta = (uint8)u32Key - u8Seq; // 最上位ビットは設定されない
		if (u8Delta <= 4) { // ４パケット位まで旧いものは処理しない
			// すでに処理したパケット
			return TRUE;
		}
	}

	// 新しいパケットである（時間情報を格納する）
	DUPCHK_vAdd(pc, u32Addr, u8Seq);
	return FALSE;
}


/** @ingroup MASTER
 * オーディオデータを送信します。
 *
 * @param u8buf オーディオサンプル
 * @param u8len バッファーの長さ
 * @param u8Seq シーケンス番号
 * @return
 */
static int16 i16TransmitAudioData(uint8 *pu8buf, uint8 u8len) {
	int16 i16Ret = -1;
	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));
	uint8 *q = sTx.auData;

	sAppData.u16TxFrame++;

	/// ペイロード
	// 最初のバイト
	S_OCTET(sAppData.u8AppIdentifier);

	// パケットバージョン
	S_OCTET(1);

	// 送信元のアドレス
	S_OCTET(sAppData.u8AppLogicalId);
	S_BE_DWORD(ToCoNet_u32GetSerial());

	// 中継フラグ
	S_OCTET(0);

	// CODEC データのコピー
	memcpy(q, pu8buf, u8len);
	q += u8len;

	/// ヘッダの構成
	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA; // パケット種別

	// 送信する
	sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry = 0x81; // 1回再送

	// フレームカウントとコールバック識別子の指定
	sTx.u8Seq = (sAppData.u16TxFrame & 0xFF);
	sTx.u8CbId = sTx.u8Seq;

	sTx.bAckReq = FALSE;
	sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
	sTx.u16RetryDur = 0;
	sTx.u16DelayMax = 0;

	/// 送信
	if (ToCoNet_bMacTxReq(&sTx)) {
		i16Ret = sTx.u8CbId;
	}

	return i16Ret;
}

/** @ingroup MASTER
 * パケットの受信処理を行います。
 *
 *
 * @param pRx 受信データ
 */
static void vReceiveAudioData(tsRxDataApp *pRx) {
	uint8 *p = pRx->auData;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier)
		return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION)
		return;

	uint8 u8AppLogicalId = G_OCTET(); (void)u8AppLogicalId;
	uint32 u32Addr = G_BE_DWORD();

	/* 重複の確認を行う */
	if (bCheckDupPacket(&sDupChk, u32Addr, pRx->u8Seq)) {
		S_PUTCHAR('-');
		return;
	}

	// 中継フラグ
	uint8 u8TxFlag = G_OCTET();

	// 中継の判定 (レベルに達していなければ中継する）
	if (sAppData.u8Mode == E_IO_MODE_ROUTER) {
		if (u8TxFlag < MULTI_RELAY_LEVEL) {
			// リピータの場合はここで中継の判定
			*(p - 1) = *(p - 1) + 1; // 中継済みフラグのセット
			// 中継する
			i16TransmitRepeat(pRx);
		}
		return;
	}

	// 以下オーディオデータ
	sIdleCountDown = IDLE_COUNT_RESET_VALUE;	// アイドル監視タイマーをリセット
	if (sAppData.bPktMon) {
		// 直前のパケットと比較して連続しているかチェック
		static uint16 u16SeqPrev = 0xFFFF;
		uint8 c;

		if (u16SeqPrev <= 0xFF) {
			if (((u16SeqPrev + 1) & 0xFF) == pRx->u8Seq) {
				c = 'o';
				u16SeqPrev = pRx->u8Seq;
			} else {
				c = 'x';
				u16SeqPrev = 0xFFFF;
			}
		} else {
			u16SeqPrev = pRx->u8Seq;
			c = 'n';
		}

		S_PUTCHAR(c);
	}

	uint8 u8len_dat = pRx->u8Len - (p - pRx->auData); // パケットの終わりまでがデータ
	CodecPreBuffer_u8Add(p, u8len_dat);
}

/** @ingroup MASTER
 * pairingを設定する要求コマンドパケットを送信します。
 *
 * - Packet 構造
 *   - OCTET: 識別ヘッダ(APP ID より生成)
 *   - OCTET: プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET: 送信元論理ID
 *   - BE_DWORD: 送信元のシリアル番号
 *   - OCTET: 宛先論理ID
 *   - BE_WORD: 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET: 中継フラグ
 *   - OCTET: 形式 (1固定)
 *   - BE_DWORD: 要求APP ID
 *   - OCTET: 要求channel
 *   - BE_DWORD: 候補APP ID
 *   - BE_DWORD: 対向APP ID
 *   - OCTET: 候補channel
 *   - BE_WORD: 送信元のペアマッチ確認回数
 *
 * @param u8DstAddr 送信先
 * @param pReq 設定データ
 * @return -1:Error, 0..255:CbId
 */
static int16 i16TransmitPairingRequest(uint32 u32State)
{
	int16 i16Ret = 0;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	uint8 *q = sTx.auData;

	S_OCTET(sAppData.u8AppIdentifier);
	S_OCTET(APP_PROTOCOL_VERSION);
	 // アプリケーション論理アドレス
	if (sAppData.u8PairingMode & 2) {
		// SW2が押されていたら親機として振る舞う
		S_OCTET(LOGICAL_ID_PARENT);
	} else {
		S_OCTET(LOGICAL_ID_CHILDREN);
	}
	S_BE_DWORD(ToCoNet_u32GetSerial());  // シリアル番号
	// 宛先アプリケーション論理アドレス
	if (sAppData.u8PairingMode & 2) {
		// SW2が押されていたら親機として振る舞う
		S_OCTET(LOGICAL_ID_CHILDREN);
	} else {
		S_OCTET(LOGICAL_ID_PARENT);
	}
	S_BE_WORD(sAppData.u32CtTimer0 & 0xFFFF); // タイムスタンプ
	S_OCTET(0); // 中継フラグ

	S_BE_DWORD(sAppData.u32ReqAppId);  // 要求APP ID
	S_OCTET(sAppData.u8ReqCh); // 要求channel

	S_BE_DWORD(sAppData.u32CandidateAppId);  // 候補APP ID
	S_BE_DWORD(sAppData.u32AnotherAppId);  // 対向APP ID
	S_OCTET(sAppData.u8CandidateCh); // 受諾channel
	S_BE_WORD(sAppData.u16MatchCount);	// マッチカウンタ

	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_USER_PAIRING; // パケット種別

	// 送信する
	sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry = 0x81; // 1回再送

	{
		/* 送信設定 */
		sTx.bAckReq = FALSE;
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
		sTx.u16RetryDur = 4; // 再送間隔
		sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

		// 送信API
		if (ToCoNet_bMacTxReq(&sTx)) {
			i16Ret = sTx.u8CbId;
		}
	}

	return i16Ret;
}

/** @ingroup MASTER
 * ペアリングパケットの受信処理を行います。
 *
 * - 受信したデータを UART に出力します。
 *   このパケットは、タイムスタンプによる重複除去アルゴリズムとは独立して
 *   処理される。
 *
 * @param pRx 受信データ
 */
static void vReceivePairingData(tsRxDataApp *pRx) {
	uint8 *p = pRx->auData;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier)
		return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION)
		return;

	uint8 u8AppLogicalId = G_OCTET();

	uint32 u32Addr = G_BE_DWORD();

	uint8 u8AppLogicalId_Dest = G_OCTET();
	(void) u8AppLogicalId_Dest;

	uint16 u16TimeStamp = G_BE_WORD();

	/* 重複の確認を行う */
	bool_t bQuick = u16TimeStamp & 0x8000 ? TRUE : FALSE; // 優先パケット（全部処理する）
	u16TimeStamp &= 0x7FFF;
	if (bQuick == FALSE
			&& bCheckDupPacket(&sDupChk, u32Addr, u16TimeStamp)) {
		return;
	}
	static uint32 u32LastQuick;
	if (bQuick) {
		if ((u32TickCount_ms - u32LastQuick) < 20) {
			// Quickパケットを受けて一定期間未満のパケットは無視する
			return;
		} else {
			u32LastQuick = u32TickCount_ms; // タイムスタンプを巻き戻す
		}
	}

	// 中継フラグ
	uint8 u8Relay = G_OCTET();
	(void)u8Relay;

	// 親機子機の判定
	if ((IS_LOGICAL_ID_PARENT(u8AppLogicalId) && IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId))
	|| (IS_LOGICAL_ID_CHILD(u8AppLogicalId) && IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId))){
		; // 親機⇒子機、または子機⇒親機への伝送
	} else {
		vfPrintf(&sSerStream, "!INF Master/Slave mismatch u8AppLogicalId:%02x, sAppData.u8AppLogicalId:%02x, u8AppLogicalId_Dest:%02x"LB, u8AppLogicalId, sAppData.u8AppLogicalId, u8AppLogicalId_Dest);
		return;
	}

	// 要求AppId
	uint32 u32ReqAppId = G_BE_DWORD();
	// 要求ch
	uint8 u8ReqCh = G_OCTET();
	(void)u8ReqCh;
	// 候補AppId
	uint32 u32AcceptAppId = G_BE_DWORD();
	// 対向AppId
	uint32 u32AnotherAppId = G_BE_DWORD();
	// 候補ch
	uint8 u8AcceptCh = G_OCTET();
	(void)u8AcceptCh;
	sAppData.u16PeerMatched = G_BE_WORD();	// 相手方pairingマッチカウンタ

	if (sAppData.u32CandidateAppId == 0) {
		if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
			// 子機ならば相手の提示を受け入れる
			sAppData.u32CandidateAppId = u32ReqAppId;
			sAppData.u32AnotherAppId = sAppData.u32ReqAppId;
			sAppData.u8CandidateCh = u8ReqCh;
		} else if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
			// 親機ならば自身の主張を通す
			sAppData.u32CandidateAppId = sAppData.u32ReqAppId;
			sAppData.u32AnotherAppId = u32ReqAppId;
			sAppData.u8CandidateCh = sAppData.u8ReqCh;
		} else {
			// bad case
		}
	} else if (sAppData.u32CandidateAppId == u32AcceptAppId
			&& sAppData.u32AnotherAppId == u32AnotherAppId) {
		sAppData.u16MatchCount++;
	} else {
		// ignore u32AcceptAppId == 0 or bad case(maybe multiple peer).
	}

	/* UART 出力 */
	if (!sSerCmd.bverbose) {
		// 以下のようにペイロードを書き換えて UART 出力
		pRx->auData[0] = pRx->u8Len; // １バイト目はバイト数
		pRx->auData[2] = pRx->u8Lqi; // ３バイト目(もともとは送信元の LogicalID) は LQI

		vSerOutput_ModbusAscii(&sSerStream, u8AppLogicalId,
				SERCMD_ID_INFORM_IO_DATA, pRx->auData, pRx->u8Len);
	}
}


/** @ingroup MASTER
 * IOデータを中継送信します。
 * 事前にペイロード中の中継カウント数を書き換えておきます。
 *
 * - パケット中の中継フラグのビットは、呼び出し前に設定されています。
 * - 衝突を抑制出来る程度の送信遅延、再送間隔を設定しています。
 *
 * @param pRx 受信したときのデータ
 * @return -1:Error, 0..255:CbId
 */
int16 i16TransmitRepeat(tsRxDataApp *pRx) {
	int16 i16Ret = -1;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	// Payload
	memcpy(sTx.auData, pRx->auData, pRx->u8Len);
	sTx.u8Len = pRx->u8Len;

	// コマンド設定
	sTx.u8Cmd = pRx->u8Cmd; // パケット種別

	// 送信する
	sTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry = 0x81; // ２回再送

	// フレームカウントとコールバック識別子の指定
	sTx.u8Seq = pRx->u8Seq;
	sTx.u8CbId = sTx.u8Seq;

	// 中継時の送信パラメータ
	sTx.bAckReq = FALSE;
	sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
	sTx.u16RetryDur = 8; // 再送間隔

	sTx.u16DelayMin = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)
	sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

	// 送信API
	if (ToCoNet_bMacTxReq(&sTx)) {
		i16Ret = sTx.u8CbId;
	}

	return i16Ret;
}

/** @ingroup MASTER
 * TICKTIMER のイベントから呼び出され、オーディオ処理を行う。
 *
 * - ボタンの押下状態のチェックで、PTT および テストトーン
 * - ADバッファのチェック⇒無線送信
 */
static void vProcessAudio() {
	uint32 bmPorts, bmChanged;
	bool_t bTestTone = FALSE;

	if (bBTM_GetState(&bmPorts, &bmChanged)) {
		// テストトーン
		if (bmChanged & PORT_INPUT_MASK) {
			// 動作モードのチェック
			bTestTone = (bmPorts & (1UL << PORT_INPUT2)) ? TRUE : FALSE;
			vAM_TestToneOn(bTestTone);
		}
	}

	// テストモードの時は強制する
	if (sAppData.bTestMode) {
		bTestTone = TRUE;
		vAM_TestToneOn(bTestTone);
	}

	// サンプルデータの入力および送信
	int16 *pi16SamplePacket;
	if ((pi16SamplePacket = pi16AM_GetData()) != NULL) {
		// サンプルデータが入手できた
		uint8 au8buf[128], *q = au8buf;

		// サンプルの状態データ
		uint8 u8len_stat;
		uint8 *pu8Stat = CODEC_pu8GetStatus(sAppData.sEncode, &u8len_stat);
		if (u8len_stat) {
			memcpy(q, pu8Stat, u8len_stat);
			q += u8len_stat;
		}

		// サンプル列のエンコード（圧縮）
		uint16 u16len_coded = CODEC_u16Encode(sAppData.sEncode, pi16SamplePacket, q, u16AM_GetSampleBufferSize());
		q += u16len_coded;

		if ((bmPorts & ((1UL << PORT_INPUT1) | (1UL << PORT_INPUT2))) || sAppData.bTestMode) {
			sIdleCountDown = IDLE_COUNT_RESET_VALUE;	// アイドル監視タイマーをリセット
			i16TransmitAudioData(au8buf, q - au8buf);
			if(sAppData.bPktMon) {
				S_PUTCHAR('.');
			}
		}
	}

	// サンプルデータをダブルバッファへ投入
	uint8 u8FreeBlk = u8AM_CountFreeDataBlock();
	if (u8FreeBlk != 0) {
		uint8 *pu8dat = CodecPreBuffer_pu8Find();

		if (pu8dat != NULL) {
			int16 *pi16SamplePacket = pi16AM_GetFreeDataBlock();
			if (pi16SamplePacket != NULL) {
				// 状態のアップデート
				uint8 u8lens = CODEC_u8SetStatus(sAppData.sDecode, pu8dat);
				pu8dat += u8lens;

				S_PUTCHAR('s');

				// データのデコード（展開）
				CODEC_u16Decode(sAppData.sDecode, pu8dat, pi16SamplePacket, u16AM_GetSampleBufferSize());

				vAM_DataBlockLoaded(pi16SamplePacket);
			}
		}
	}

	// LED の点灯
#ifdef POSITIVE_LOGIC_LED
	vPortSet_TrueAsLo(PORT_OUT2, !(bmPorts & ((1UL << PORT_INPUT1) | (1UL << PORT_INPUT2)))); // 送信ボタン押下時は LED 点灯
	vPortSet_TrueAsLo(PORT_OUT1, !(u8FreeBlk != 2)); // 出力データが有る時は LED 点灯
#else
	vPortSet_TrueAsLo(PORT_OUT1, (bmPorts & ((1UL << PORT_INPUT1) | (1UL << PORT_INPUT2)))); // 送信ボタン押下時は LED 点灯
	vPortSet_TrueAsLo(PORT_OUT2, (u8FreeBlk != 2)); // 出力データが有る時は LED 点灯
#endif
}

/** @ingroup MASTER
 * スリープ状態に遷移します。
 *
 * @param u32SleepDur_ms スリープ時間[ms]
 * @param bPeriodic TRUE:前回の起床時間から次のウェイクアップタイミングを計る
 * @param bDeep TRUE:RAM OFF スリープ
 */
static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep) {
	// print message.

	// stop interrupt source, if interrupt source is still running.
	vAHI_DioInterruptEnable(0, PORT_INPUT_MASK); // 割り込みの解除）

	// set UART Rx port as interrupt source
	vAHI_DioSetDirection(PORT_INPUT_MASK, 0); // set as input

	(void) u32AHI_DioInterruptStatus(); // clear interrupt register
	vAHI_DioWakeEnable(PORT_INPUT_MASK, 0); // also use as DIO WAKE SOURCE
	vAHI_DioWakeEdge(0, PORT_INPUT_MASK); // 割り込みエッジ（立下がりに設定）

	// wake up using wakeup timer as well.
	ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, u32SleepDur_ms, bPeriodic, bDeep); // PERIODIC RAM OFF SLEEP USING WK0
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
