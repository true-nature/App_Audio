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

/** @file
 * アプリケーションのメイン処理
 *
 * @defgroup MASTER アプリケーションのメイン処理
 */

#ifndef  MASTER_H_INCLUDED
#define  MASTER_H_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <stdlib.h>
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "ToCoNet.h"
#include "flash.h"
#include "btnMgr.h"

#include "codec.h"
#include "IMA_adpcm.h"
#include "RAW.h"

/** @ingroup MASTER
 * 使用する無線チャネル数の最大値 (複数設定すると Channel Agility を利用する)
 */
#define MAX_CHANNELS 3

/** @ingroup MASTER
 * ネットワークのモード列挙体 (ショートアドレス管理か中継ネットワーク層を使用したものか)
 */
typedef enum {
	E_NWKMODE_MAC_DIRECT,//!< ネットワークは構成せず、ショートアドレスでの通信を行う
	E_NWKMODE_LAYERTREE  //!< 中継ネットワークでの通信を行う(これは使用しない)
} teNwkMode;

/** @ingroup MASTER
 * IO の状態
 */
typedef struct {
	uint32 u32BtmBitmap; //!< (0xFFFFFFFF: 未確定)
	uint32 u32BtmUsed; //!< 利用対象ピンかどうか (0xFFFFFFFF: 未確定)
	uint32 u32BtmChanged; //!< (0xFFFFFFFF: 未確定)

	uint16 au16InputADC[4]; //!< (0xFFFF: 未確定) 入力ポートの ADC 値 [mV]
	uint16 au16OutputDAC[4]; //!< (0xFFFF: 未確定) 送られてきた ADC 値 [mV]
#ifdef JN514x
	uint16 au16OutputDAC_val[2]; //!< (0xFFFF: 未確定) 送られてきた ADC 値 [12bit スケール値]
#endif
	uint16 u16Volt; //!< 12bits, 0xFFFF: 未確定
	int16 i16Temp; //!< 12bits

	uint8 au8Input[4]; //!< 入力ポート (0: Hi, 1: Lo, 0xFF: 未確定)
	uint8 au8Output[4]; //!< 出力ポート (0: Hi, 1:Lo, 0xFF: 未確定)

	uint16 au16OutputPWMDuty[4]; //!< 無線経由で送られた DUTY 比 (0xFFFF: 未設定、無効)
	uint16 au16InputPWMDuty[4]; //!< 入力された AD に基づく DUTY 比の計算値 (0xFFFF: 未設定、無効)

	uint8 u8Volt; //!< i16Volt から変換

	uint32 u32TxLastTick; //!< 最後に送った時刻
	uint16 au16InputADC_LastTx[4]; //!< 最後に送信したデータ
	uint32 u32RxLastTick; //!< 最後に受信した時刻

	uint16 au16InputADC_History[4][4]; //!< ADCデータ履歴
	uint16 u16Volt_LastTx; //!< 最後に送信した電圧
	uint16 au16Volt_History[32]; //!< ADCデータ電圧履歴
	uint8 u8HistIdx; //!< 履歴情報のインデックス
	int16 i16TxCbId; //!< 送信時のID
} tsIOData;


#define HIST_VOLT_SCALE 5 //!< 電圧履歴数のスケーラ (2^HIST_VOLT_SCALE)  @ingroup MASTER
#define HIST_VOLT_COUNT (1UL << HIST_VOLT_SCALE) //!< 電圧履歴数 @ingroup MASTER


/** @ingroup MASTER
 * IO 設定要求
 */
typedef struct {
	uint8 u8IOports;          //!< 出力IOの状態 (1=Lo, 0=Hi)
	uint8 u8IOports_use_mask; //!< 設定を行うポートなら TRUE
	uint16 au16PWM_Duty[4];      //!< PWM DUTY 比 (0～1024)
	uint8 au16PWM_use_mask[4];   //!< 設定を行うPWMポートなら TRUE
} tsIOSetReq;

/** @ingroup MASTER
 * アプリケーションの情報
 */
typedef struct {
	// ToCoNet
	uint32 u32ToCoNetVersion; //!< ToCoNet のバージョン番号を保持
	uint16 u16ToCoNetTickDelta_ms; //!< ToCoNet の Tick 周期 [ms]
	uint8 u8AppIdentifier; //!< AppID から自動決定

	// メインアプリケーション処理部
	void *prPrsEv; //!< vProcessEvCoreSlpまたはvProcessEvCorePwrなどの処理部へのポインタ
	uint8 u8Hnd_vProcessEvCore; //!< vProcessEvCore のハンドル

	// DEBUG
	uint8 u8DebugLevel; //!< デバッグ出力のレベル

	// Wakeup
	bool_t bWakeupByButton; //!< TRUE なら起床時に DI 割り込みにより起床した
	uint32 u32SleepDur; //!< スリープ間隔 [ms]

	// Network mode
	uint8 u8AppLogicalId; //!< ネットワーク時の抽象アドレス 0:親機 1~:子機, 0xFF:通信しない

	// Network context

	// Flash Information
	tsFlash sFlash; //!< フラッシュからの読み込みデータ
	tsFlashApp sConfig_UnSaved; //!< フラッシュへの設定データ (0xFF, 0xFFFFFFFF は未設定)
	int8 bFlashLoaded; //!< フラッシュからの読み込みが正しく行われた場合は TRUE

	uint32 u32DIO_startup; //!< 電源投入時のIO状態

	// config mode
	uint8 u8Mode; //!< 動作モード(IO M1,M2,M3 から設定される)
	uint8 u8PairingMode;	// ペアリングモード

	// button manager
	tsBTM_Config sBTM_Config; //!< ボタン入力（連照により状態確定する）管理構造体
	PR_BTM_HANDLER pr_BTM_handler; //!< ボタン入力用のイベントハンドラ (TickTimer 起点で呼び出す)
	uint32 u32BTM_Tick_LastChange; //!< ボタン入力で最後に変化が有ったタイムスタンプ (操作の無効期間を作る様な場合に使用する)

	// Counter
	uint32 u32CtTimer0; //!< 64fps カウンタ。スリープ後も維持
	uint16 u16TxFrame; //!< 送信フレーム数
	uint16 u16CtRndCt; //!< 起動時の送信タイミングにランダムのブレを作る

	// mode
	bool_t bTestMode; //!< テストモード
	bool_t bPktMon; //!< パケットモニタ表示

	// codec
	tsCODEC sEncode; //!< CODEC 構造体 (エンコーダ)
	tsCODEC sDecode; //!< CODEC 構造体 (エンコーダ)

	// auto pairing
	uint32 u32ReqAppId;	//!< 要求AppId
	uint8 u8ReqCh;	//!< 要求Channel
	uint32 u32CandidateAppId;	//!< AppId候補
	uint32 u32AnotherAppId;	//!< 対向AppId
	uint8 u8CandidateCh;	//!< Channel候補
	uint16 u16MatchCount;	//!< pairingマッチカウンタ
	uint16 u16PeerMatched;	//!< 相手方pairingマッチカウンタ
} tsAppData;

extern tsAppData sAppData; //!< アプリケーションデータ  @ingroup MASTER


/****************************************************************************
 * フラッシュ設定情報
 ***************************************************************************/

#define FL_MASTER_u32(c) sAppData.sFlash.sData.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u32(c) sAppData.sConfig_UnSaved.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u32(c) (sAppData.sConfig_UnSaved.u32##c != 0xFFFFFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u16(c) sAppData.sFlash.sData.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u16(c) sAppData.sConfig_UnSaved.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u16(c) (sAppData.sConfig_UnSaved.u16##c != 0xFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u8(c) sAppData.sFlash.sData.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u8(c) sAppData.sConfig_UnSaved.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u8(c) (sAppData.sConfig_UnSaved.u8##c != 0xFF) //!< 構造体要素アクセス用のマクロ @ingroup FLASH

/** @ingroup FLASH
 * フラッシュ設定内容の列挙体
 */
enum {
	E_APPCONF_APPID,     //!< アプリケーションID
	E_APPCONF_CHMASK,    //!< チャネルマスク
	E_APPCONF_TX_POWER,  //!< TX 出力
	E_APPCONF_ID,        //!< 8bitのID(ネットワークアドレス)
	E_APPCONF_ROLE,      //!<
	E_APPCONF_LAYER ,    //!<
	E_APPCONF_SLEEP4,    //!< mode4 のスリープ期間設定
	E_APPCONF_SLEEP7,    //!< mode7 のスリープ期間設定
	E_APPCONF_FPS,       //!< 連続送信モードの秒あたりの送信数
	E_APPCONF_PWM_HZ,    //!< PWM の周波数
	E_APPCONF_SAMP_HZ,   //!< サンプリング周波数
	E_APPCONF_CODEC,     //!< CODECの種類 (0:IMA ADPCM, 1:10bit RAW)
	E_APPCONF_OPT,       //!< DIOの入力方法に関する設定
	E_APPCONF_BAUD_SAFE, //!< BPS ピンをGにしたときのボーレート
	E_APPCONF_BAUD_PARITY, //!< BPS ピンをGにしたときのパリティ設定 (0:None, 1:Odd, 2:Even)
	E_APPCONF_TEST
};

/** @ingroup FLASH
 * フラッシュ設定で ROLE に対する要素名の列挙体
 * (未使用、将来のための拡張のための定義)
 */
enum {
	E_APPCONF_ROLE_MAC_NODE = 0,  //!< MAC直接のノード（親子関係は無し）
	E_APPCONF_ROLE_NWK_MASK = 0x10, //!< NETWORKモードマスク
	E_APPCONF_ROLE_PARENT,          //!< NETWORKモードの親
	E_APPCONF_ROLE_ROUTER,        //!< NETWORKモードの子
	E_APPCONF_ROLE_ENDDEVICE,     //!< NETWORKモードの子（未使用、スリープ対応）
	E_APPCONF_ROLE_SILENT = 0x7F, //!< 何もしない（設定のみ)
};

#endif  /* MASTER_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
