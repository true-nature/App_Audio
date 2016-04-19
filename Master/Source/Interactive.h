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

#ifndef INTERACTIVE_H_
#define INTERACTIVE_H_

#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "flash.h"
#include "input_string.h"

void vProcessInputByte(uint8 u8Byte);
void vProcessInputString(tsInpStr_Context *pContext);

bool_t bConfig_Load(tsFlash *p);
void vConfig_SaveAndReset();
void vConfig_UnSetAll(tsFlashApp *p);
void vConfig_SetDefaults(tsFlashApp *p);
void vSerUpdateScreen();

void vHandleSerialInput();

#define E_APPCONF_OPT_INPUT_LPF 0x0010 //!< 入力のローパス
#define E_APPCONF_OPT_OUTPUT_LPF 0x0020 //!< 出力のローパス

#define E_APPCONF_OPT_INPUT_DOUBLE_SAMPLE 0x0100 //!< 入力を倍速する
#define E_APPCONF_OPT_OUTPUT_SYNC_PWM 0x1000 //!< 出力同期
#define E_APPCONF_OPT_OUTPUT_SYNC_PWM_PASS_THRU 0x2000 //!< 出力同期

#define IS_APPCONF_OPT_INPUT_LPF() ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_INPUT_LPF) ? TRUE : FALSE) //!< E_APPCONF_OPT_INPUT_LPF 判定 @ingroup FLASH
#define IS_APPCONF_OPT_OUTPUT_LPF() ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_OUTPUT_LPF) ? TRUE : FALSE) //!< APPCONF_OPT_OUTPUT_LPF判定 @ingroup FLASH

#define IS_APPCONF_OPT_INPUT_DOUBLE_SAMPLE() ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_INPUT_DOUBLE_SAMPLE) ? TRUE : FALSE) //!< E_APPCONF_OPT_INPUT_DOUBLE_SAMPLE 判定 @ingroup FLASH
#define IS_APPCONF_OPT_OUTPUT_SYNC_PWM() ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_OUTPUT_SYNC_PWM) ? TRUE : FALSE) //!< E_APPCONF_OPT_OUTPUT_SYNC_PWM判定 @ingroup FLASH
#define IS_APPCONF_OPT_OUTPUT_SYNC_PWM_PASS_THRU() ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_OUTPUT_SYNC_PWM_PASS_THRU) ? TRUE : FALSE) //!< E_APPCONF_OPT_OUTPUT_SYNC_PWM判定 @ingroup FLASH


#endif /* INTERACTIVE_H_ */
