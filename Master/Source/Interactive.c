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

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "Master.h"

#include "utils.h"
#include "flash.h"

#include "common.h"

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "modbus_ascii.h"
#include "input_string.h"

#include "Master.h"
#include "Interactive.h"

#include "audioDevice.h"
#include "audioManager.h"

#include "config.h"

/****************************************************************************/
/***        Imported variables/functions                                  ***/
/****************************************************************************/
extern tsFILE sSerStream;
extern tsSerialPortSetup sSerPort;

tsModbusCmd sSerCmd; //!< シリアル入力系列のパーサー (modbus もどき)  @ingroup MASTER
tsInpStr_Context sSerInpStr; //!< 文字列入力  @ingroup MASTER
uint16 u16HoldUpdateScreen = 0; //!< スクリーンアップデートを行う遅延カウンタ  @ingroup MASTER

extern void vSerInitMessage();
extern void vProcessSerialCmd(tsModbusCmd *);

/****************************************************************************/
/***        Procedures                                                    ***/
/****************************************************************************/


/** @ingroup MASTER
 * シリアルポートからの入力を処理します。
 * - シリアルポートからの入力は uart.c/serial.c により管理される FIFO キューに値が格納されます。
 *   このキューから１バイト値を得るのが SERIAL_i16RxChar() です。
 * - 本関数では、入力したバイトに対し、アプリケーションのモードに依存した処理を行います。
 *   - 文字列入力モード時(INPSTR_ API 群、インタラクティブモードの設定値入力中)は、INPSTR_u8InputByte()
 *     API に渡す。文字列が完了したときは vProcessInputString() を呼び出し、設定値の入力処理を
 *     行います。
 *   - 上記文字列入力ではない場合は、ModBusAscii_u8Parse() を呼び出します。この関数は + + + の
 *     入力判定および : で始まる書式を認識します。
 *   - 上記書式解釈中でない場合は、vProcessInputByte() を呼び出します。この関数はインタラクティブ
 *     モードにおける１文字入力コマンドを処理します。
 *
 */
void vHandleSerialInput() {
	// handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

		// process
		if (i16Char >= 0 && i16Char <= 0xFF) {
			//DBGOUT(0, "[%02x]", i16Char);
			if (INPSTR_bActive(&sSerInpStr)) {
				// 文字列入力モード
				uint8 u8res = INPSTR_u8InputByte(&sSerInpStr, (uint8) i16Char);

				if (u8res == E_INPUTSTRING_STATE_COMPLETE) {
					vProcessInputString(&sSerInpStr);
				} else if (u8res == E_INPUTSTRING_STATE_CANCELED) {
					V_PRINT("(canceled)");
					u16HoldUpdateScreen = 64;
				}
				continue;
			}

			{
				// コマンド書式の系列解釈、および verbose モードの判定
				uint8 u8res = ModBusAscii_u8Parse(&sSerCmd, (uint8) i16Char);

				if (u8res != E_MODBUS_CMD_EMPTY) {
					V_PUTCHAR(i16Char);
				}

				if (u8res == E_MODBUS_CMD_COMPLETE
						|| u8res == E_MODBUS_CMD_LRCERROR) {
					// 解釈完了

					if (u8res == E_MODBUS_CMD_LRCERROR) {
						// command complete, but CRC error
						V_PRINT(LB "!INF LRC_ERR? (might be %02X)" LB,
								sSerCmd.u8lrc);
					}

					if (u8res == E_MODBUS_CMD_COMPLETE) {
						// process command
						vProcessSerialCmd(&sSerCmd);
					}

					continue;
				} else if (u8res != E_MODBUS_CMD_EMPTY) {
					if (u8res == E_MODBUS_CMD_VERBOSE_ON) {
						// verbose モードの判定があった
						vSerUpdateScreen();
					}

					if (u8res == E_MODBUS_CMD_VERBOSE_OFF) {
						vfPrintf(&sSerStream, "!INF EXIT INTERACTIVE MODE."LB);
					}

					// still waiting for bytes.
					continue;
				} else {
					; // コマンド解釈モードではない
				}
			}

			// Verbose モードのときは、シングルコマンドを取り扱う
			if (sSerCmd.bverbose) {
				vProcessInputByte(i16Char);
			}
		}
	}
}

/** @ingroup MASTER
 * １バイト入力コマンドの処理\n
 * - 設定値の入力が必要な項目の場合、INPSTR_vStart() を呼び出して文字列入力モードに遷移します。
 * - フラッシュへのセーブ時の手続きでは、sAppData.sConfig_UnSaved 構造体で入力が有ったものを
 *   sFlash.sData 構造体に格納しています。
 * - デバッグ用の確認コマンドも存在します。
 *
 * @param u8Byte 入力バイト
 */
void vProcessInputByte(uint8 u8Byte) {
	static uint8 u8lastbyte;

	switch (u8Byte) {
	case 0x0d:
	case 'h':
	case 'H':
		// 画面の書き換え
		u16HoldUpdateScreen = 1;
		break;

	case 'a': // set application ID
		V_PRINT("Input Application ID (HEX:32bit): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8,
				E_APPCONF_APPID);
		break;

	case 'c': // チャネルの設定
		V_PRINT("Input Channel(s) (e.g. 11,16,21): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 8,
				E_APPCONF_CHMASK);
		break;

	case 'd': // CODEC の種類
		V_PRINT("Input CODEC typr (0:ADPCM 1:10bitRAW 2:8bitRAW): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_CODEC);
		break;

	case 'i': // set application role
		V_PRINT("Input Device ID (DEC:1-100): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_ID);
		break;

	case 'x': // 出力の変更
		V_PRINT("Tx Power (0[min]-3[max]): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 1,
				E_APPCONF_TX_POWER);
		break;

	case 'p': // PWMの駆動周波数の変更
		V_PRINT("Input PWM Hz (DEC:1-64000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 6,
				E_APPCONF_PWM_HZ);
		break;

	case 'f': // PWMの駆動周波数の変更
		V_PRINT("Input Sample Hz (DEC:1-20000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 6,
				E_APPCONF_SAMP_HZ);
		break;

	case 'b': // ボーレートの変更
		V_PRINT("Input baud rate (DEC:9600-230400): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 10,
				E_APPCONF_BAUD_SAFE);
		break;

	case 'o': // オプションビットの設定
		V_PRINT("Input option bits (HEX): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8,
				E_APPCONF_OPT);
		break;

	case 'S':
		// フラッシュへのデータ保存
		if (u8lastbyte == 'R') {
			// R S と連続入力された場合は、フラッシュエリアを消去する
			V_PRINT("!INF CLEAR SAVE AREA.");
			bFlash_Erase(FLASH_SECTOR_NUMBER - 1); // SECTOR ERASE

			vWait(1000000);
			vAHI_SwReset();
		} else {
			vConfig_SaveAndReset();
		}
		break;

	case 'R':
		vConfig_SetDefaults(&sAppData.sConfig_UnSaved);
		u16HoldUpdateScreen = 1;
		break;

	case '$':
		sAppData.u8DebugLevel++;
		if (sAppData.u8DebugLevel > 5)
			sAppData.u8DebugLevel = 0;

		V_PRINT("* set App debug level to %d." LB, sAppData.u8DebugLevel);
		break;

	case '@':
		_C {
			static uint8 u8DgbLvl;

			u8DgbLvl++;
			if (u8DgbLvl > 5)
				u8DgbLvl = 0;
			ToCoNet_vDebugLevel(u8DgbLvl);

			V_PRINT("* set NwkCode debug level to %d." LB, u8DgbLvl);
		}
		break;

	case '!':
		// リセット
		V_PRINT("!INF RESET SYSTEM.");
		vWait(1000000);
		vAHI_SwReset();
		break;

	case '#': // info
		_C {
			V_PRINT("*** ToCoNet(ver%08X) ***" LB, ToCoNet_u32GetVersion());
			V_PRINT("* AppID %08x, LongAddr, %08x, ShortAddr %04x, Tk: %d" LB,
					sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(),
					sToCoNet_AppContext.u16ShortAddress, u32TickCount_ms);
			if (sAppData.bFlashLoaded) {
				V_PRINT("** Conf "LB);
				V_PRINT("* AppId = %08x"LB, sAppData.sFlash.sData.u32appid);
				V_PRINT("* ChMsk = %08x"LB, sAppData.sFlash.sData.u32chmask);
				V_PRINT("* Ch=%d"LB,
						sToCoNet_AppContext.u8Channel);
			} else {
				V_PRINT("** Conf: none"LB);
			}
		}
		break;

	case 'V':
		vSerInitMessage();
		V_PRINT("---"LB);
		V_PRINT("ToCoNet lib version Core: %08x, Ext: %08x, Utils: %08x"LB,
				ToCoNet_u32GetVersion(), ToCoNet_u32GetVersion_LibEx(),
				ToCoNet_u32GetVersion_LibUtils());
		V_PRINT("ToCoNet Tick Counter: %d"LB, u32TickCount_ms);
		V_PRINT("Run Time: %d+%02d/64 sec"LB, sAppData.u32CtTimer0 >> 6,
				sAppData.u32CtTimer0 & 0x3F);
		V_PRINT(""LB);
		break;

	// テストモード
	case 'T':
		sAppData.bTestMode = sAppData.bTestMode ? FALSE : TRUE;
		V_PRINT("TEST MODE = %s"LB, sAppData.bTestMode ? "TRUE" : "FALSE");

		if (sAppData.bTestMode == FALSE) {
			vAM_TestToneOn(FALSE);
		}
		break;

	// テストモード
	case 'D':
		sAppData.bPktMon = sAppData.bPktMon ? FALSE : TRUE;
		V_PRINT("DISP MODE = %s"LB, sAppData.bPktMon ? "TRUE" : "FALSE");
		break;

	default:
		u8lastbyte = 0xFF;
		break;
	}

	// 一つ前の入力
	if (u8lastbyte != 0xFF) {
		u8lastbyte = u8Byte;
	}

}

/** @ingroup MASTER
 * 文字列入力モードの処理を行います。
 *
 */
void vProcessInputString(tsInpStr_Context *pContext) {
	uint8 *pu8str = pContext->au8Data;
	uint8 u8idx = pContext->u8Idx;

	switch (pContext->u32Opt) {
	case E_APPCONF_APPID:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			uint16 u16h, u16l;
			u16h = u32val >> 16;
			u16l = u32val & 0xFFFF;

			if (u16h == 0x0000 || u16h == 0xFFFF || u16l == 0x0000
					|| u16l == 0xFFFF) {
				V_PRINT(
						"(ignored: 0x0000????,0xFFFF????,0x????0000,0x????FFFF can't be set.)");
			} else {
				sAppData.sConfig_UnSaved.u32appid = u32val;
			}

			V_PRINT(LB"-> %08X"LB, u32val);
		}
		break;

	case E_APPCONF_CHMASK:
		_C {
			// チャネルマスク（リスト）を入力解釈する。
			//  11,15,19 のように１０進数を区切って入力する。
			//  以下では区切り文字は任意で MAX_CHANNELS 分処理したら終了する。

			uint8 b = 0, e = 0, i = 0, n_ch = 0;
			uint32 u32chmask = 0; // 新しいチャネルマスク

			V_PRINT(LB"-> ");

			for (i = 0; i <= pContext->u8Idx; i++) {
				if (pu8str[i] < '0' || pu8str[i] > '9') {
					e = i;
					uint8 u8ch = 0;

					// 最低２文字あるときに処理
					if (e - b > 0) {
						u8ch = u32string2dec(&pu8str[b], e - b);
						if (u8ch >= 11 && u8ch <= 26) {
							if (n_ch) {
								V_PUTCHAR(',');
							}
							V_PRINT("%d", u8ch);
							u32chmask |= (1UL << u8ch);

							n_ch++;
							if (n_ch >= MAX_CHANNELS) {
								break;
							}
						}
					}
					b = i + 1;
				}

				if (pu8str[i] == 0x0) {
					break;
				}
			}

			if (u32chmask == 0x0) {
				V_PRINT("(ignored)");
			} else {
				sAppData.sConfig_UnSaved.u32chmask = u32chmask;
			}

			V_PRINT(LB);
		}
		break;

	case E_APPCONF_ID:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 0x7F) {
				sAppData.sConfig_UnSaved.u8id = u32val;
				V_PRINT("%d(0x%02x)"LB, u32val, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_TX_POWER:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 3) {
				sAppData.sConfig_UnSaved.u8pow = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_PWM_HZ:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");

			if (u32val > 0 && u32val < 0xFFFF) { // Timer API の限界は 16bit なので。
				sAppData.sConfig_UnSaved.u32PWM_Hz = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_SAMP_HZ:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");

			if (u32val > 0 && u32val <= 20000) { // 16kHz
				sAppData.sConfig_UnSaved.u16Samp_Hz = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_OPT:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			V_PRINT(LB"-> ");

			sAppData.sConfig_UnSaved.u32Opt = u32val;
			V_PRINT("0x%08X"LB, u32val);
		}
		break;

	case E_APPCONF_BAUD_SAFE:
		_C {
			uint32 u32val = 0;

			if (pu8str[0] == '0' && pu8str[1] == 'x') {
				u32val = u32string2hex(pu8str + 2, u8idx - 2);
			}
			if (u8idx <= 6) {
				u32val = u32string2dec(pu8str, u8idx);
			}

			V_PRINT(LB"-> ");

			if (u32val) {
				sAppData.sConfig_UnSaved.u32baud_safe = u32val;
				if (u32val & 0x80000000) {
					V_PRINT("%x"LB, u32val);
				} else {
					V_PRINT("%d"LB, u32val);
				}
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_BAUD_PARITY:
		_C {
			V_PRINT(LB"-> ");

			if (pu8str[0] == 'N' || pu8str[0] == 'n') {
				sAppData.sConfig_UnSaved.u8parity = 0;
				V_PRINT("None"LB);
			} else if (pu8str[0] == 'O' || pu8str[0] == 'o') {
				sAppData.sConfig_UnSaved.u8parity = 1;
				V_PRINT("Odd"LB);
			} else if (pu8str[0] == 'E' || pu8str[0] == 'e') {
				sAppData.sConfig_UnSaved.u8parity = 2;
				V_PRINT("Even"LB);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_CODEC:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 2) {
				sAppData.sConfig_UnSaved.u8codec = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	default:
		break;
	}

	// 一定時間待って画面を再描画
	u16HoldUpdateScreen = 96; // 1.5sec
}


/** @ingroup FLASH
 * フラッシュ設定構造体をデフォルトに巻き戻します。
 * - ここでシステムのデフォルト値を決めています。
 *
 * @param p 構造体へのアドレス
 */
void vConfig_SetDefaults(tsFlashApp *p) {
	p->u32appid = APP_ID;
	p->u32chmask = CHMASK;
	p->u8ch = CHANNEL;
	p->u8pow = 3;

	p->u8id = 0;

	p->u32PWM_Hz = AUDIO_DEV_PWM_FREQ;
	p->u16Samp_Hz = AUDIO_DEV_SAMPLE_FREQ;
	p->u8codec = 0;

	p->u32baud_safe = UART_BAUD_SAFE;
	p->u8parity = 0; // none

	p->u32Opt = 0; // デフォルトの設定ビット (2x sample, sync)
	p->u16SleepDur_ms = MODE4_SLEEP_DUR_ms;
}


/** @ingroup FLASH
 * フラッシュ構造体の読み出し
 * バージョン等のチェックを行い、問題が有れば FALSE を返す
 *
 * @param p
 * @return
 */
bool_t bConfig_Load(tsFlash *p) {
	bool_t bRet = FALSE;

	// フラッシュの読み出し
	bRet = bFlash_Read(p, FLASH_SECTOR_NUMBER - 1, 0);

	// Version String のチェック
	if (bRet && APP_ID == p->sData.u32appkey  && VERSION_U32 == p->sData.u32ver) {
		bRet = TRUE;
	}

	return bRet;
}

/** @ingroup FLASH
 * フラッシュ設定構造体を全て未設定状態に設定します。未設定状態は全て 0xFF と
 * なり、逆にいえば 0xFF, 0xFFFF といった設定値を格納できます。
 *
 * @param p 構造体へのアドレス
 */
void vConfig_UnSetAll(tsFlashApp *p) {
	memset(p, 0xFF, sizeof(tsFlashApp));
}

/** @ingroup FLASH
 * フラッシュ(またはEEPROM)に保存し、モジュールをリセットする
 */
void vConfig_SaveAndReset() {
	tsFlash sFlash = sAppData.sFlash;

	if (sAppData.sConfig_UnSaved.u32appid != 0xFFFFFFFF) {
		sFlash.sData.u32appid = sAppData.sConfig_UnSaved.u32appid;
	}
	if (sAppData.sConfig_UnSaved.u32chmask != 0xFFFFFFFF) {
		sFlash.sData.u32chmask = sAppData.sConfig_UnSaved.u32chmask;
	}
	if (sAppData.sConfig_UnSaved.u8id != 0xFF) {
		sFlash.sData.u8id = sAppData.sConfig_UnSaved.u8id;
	}
	if (sAppData.sConfig_UnSaved.u8ch != 0xFF) {
		sFlash.sData.u8ch = sAppData.sConfig_UnSaved.u8ch;
	}
	if (sAppData.sConfig_UnSaved.u8pow != 0xFF) {
		sFlash.sData.u8pow = sAppData.sConfig_UnSaved.u8pow;
	}
	if (sAppData.sConfig_UnSaved.u8codec != 0xFF) {
		sFlash.sData.u8codec = sAppData.sConfig_UnSaved.u8codec;
	}
	if (sAppData.sConfig_UnSaved.u32PWM_Hz != 0xFFFFFFFF) {
		sFlash.sData.u32PWM_Hz = sAppData.sConfig_UnSaved.u32PWM_Hz;
	}
	if (sAppData.sConfig_UnSaved.u16Samp_Hz != 0xFFFF) {
		sFlash.sData.u16Samp_Hz = sAppData.sConfig_UnSaved.u16Samp_Hz;
	}
	if (sAppData.sConfig_UnSaved.u32baud_safe != 0xFFFFFFFF) {
		sFlash.sData.u32baud_safe = sAppData.sConfig_UnSaved.u32baud_safe;
	}
	if (sAppData.sConfig_UnSaved.u8parity != 0xFF) {
		sFlash.sData.u8parity = sAppData.sConfig_UnSaved.u8parity;
	}
	if (sAppData.sConfig_UnSaved.u32Opt != 0xFFFFFFFF) {
		sFlash.sData.u32Opt = sAppData.sConfig_UnSaved.u32Opt;
	}

	sFlash.sData.u32appkey = APP_ID;
	sFlash.sData.u32ver = VERSION_U32;

	bool_t bRet = bFlash_Write(&sFlash, FLASH_SECTOR_NUMBER - 1, 0);
	V_PRINT("!INF FlashWrite %s"LB, bRet ? "Success" : "Failed");
	vConfig_UnSetAll(&sAppData.sConfig_UnSaved);
	vWait(100000);

	V_PRINT("!INF RESET SYSTEM...");
	vWait(1000000);
	vAHI_SwReset();
}

/** @ingroup MASTER
 * インタラクティブモードの画面を再描画する。
 * - 本関数は TIMER_0 のイベント処理時に u16HoldUpdateScreen カウンタがデクリメントされ、
 *   これが0になった時に呼び出される。
 *
 * - 設定内容、設定値、未セーブマーク*を出力している。FL_... のマクロ記述を参照。
 *
 */
void vSerUpdateScreen() {
	V_PRINT("%c[2J%c[H", 27, 27); // CLEAR SCREEN
	V_PRINT(
			"--- CONFIG/" APP_NAME " V%d-%02d-%d/SID=0x%08x/LID=0x%02x ---"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(),
			sAppData.u8AppLogicalId);

	// Application ID
	V_PRINT(" a: set Application ID (0x%08x)%c" LB,
			FL_IS_MODIFIED_u32(appid) ? FL_UNSAVE_u32(appid) : FL_MASTER_u32(appid),
			FL_IS_MODIFIED_u32(appid) ? '*' : ' ');

	// Device ID
	{
		uint8 u8DevID =
				FL_IS_MODIFIED_u8(id) ? FL_UNSAVE_u8(id) : FL_MASTER_u8(id);

		if (u8DevID == 0x00) { // unset
			V_PRINT(" i: set Device ID (--)%c"LB,
					FL_IS_MODIFIED_u8(id) ? '*' : ' ');
		} else {
			V_PRINT(" i: set Device ID (%d=0x%02x)%c"LB, u8DevID, u8DevID,
					FL_IS_MODIFIED_u8(id) ? '*' : ' ');
		}
	}

	V_PRINT(" c: set Channels (");
	{
		// find channels in ch_mask
		uint8 au8ch[MAX_CHANNELS], u8ch_idx = 0;
		int i;
		memset(au8ch, 0, MAX_CHANNELS);
		uint32 u32mask =
				FL_IS_MODIFIED_u32(chmask) ?
						FL_UNSAVE_u32(chmask) : FL_MASTER_u32(chmask);
		for (i = 11; i <= 26; i++) {
			if (u32mask & (1UL << i)) {
				if (u8ch_idx) {
					V_PUTCHAR(',');
				}
				V_PRINT("%d", i);
				au8ch[u8ch_idx++] = i;
			}

			if (u8ch_idx == MAX_CHANNELS) {
				break;
			}
		}
	}
	V_PRINT(")%c" LB, FL_IS_MODIFIED_u32(chmask) ? '*' : ' ');

	V_PRINT(" x: set Tx Power (%d)%c" LB,
			FL_IS_MODIFIED_u8(pow) ? FL_UNSAVE_u8(pow) : FL_MASTER_u8(pow),
			FL_IS_MODIFIED_u8(pow) ? '*' : ' ');

	V_PRINT(" p: set PWM HZ (%d)%c" LB,
			FL_IS_MODIFIED_u32(PWM_Hz) ? FL_UNSAVE_u32(PWM_Hz) : FL_MASTER_u32(PWM_Hz),
			FL_IS_MODIFIED_u32(PWM_Hz) ? '*' : ' ');

	V_PRINT(" f: set Sample Freq (%d)%c" LB,
			FL_IS_MODIFIED_u16(Samp_Hz) ? FL_UNSAVE_u16(Samp_Hz) : FL_MASTER_u16(Samp_Hz),
			FL_IS_MODIFIED_u16(Samp_Hz) ? '*' : ' ');

	V_PRINT(" o: set Option Bits (0x%08X)%c" LB,
			FL_IS_MODIFIED_u32(Opt) ? FL_UNSAVE_u32(Opt) : FL_MASTER_u32(Opt),
			FL_IS_MODIFIED_u32(Opt) ? '*' : ' ');

	V_PRINT(" d: set Codec Type (%d)%c" LB,
			FL_IS_MODIFIED_u8(codec) ? FL_UNSAVE_u8(codec) : FL_MASTER_u8(codec),
			FL_IS_MODIFIED_u8(codec) ? '*' : ' ');

	{
		uint32 u32baud =
				FL_IS_MODIFIED_u32(baud_safe) ?
						FL_UNSAVE_u32(baud_safe) : FL_MASTER_u32(baud_safe);
		if (u32baud & 0x80000000) {
			V_PRINT(" b: set UART baud (%x)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		} else {
			V_PRINT(" b: set UART baud (%d)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		}
	}

	V_PRINT("---"LB);

	V_PRINT(" S: save Configuration" LB " R: reset to Defaults" LB LB);
	//       0123456789+123456789+123456789+1234567894123456789+123456789+123456789+123456789
}

