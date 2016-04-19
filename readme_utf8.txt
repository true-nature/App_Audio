App_Audio - PTT (PushToTalk) オーディオ, 信号

■ 仕様
  - PWM周期 32000Hz
  - サンプリング周期 fs=8000Hz (ADPCM 時)
                     fs=4000Hz (RAW 8bit)
                     fs=3200Hz (RAW 10bit)

  - 音声圧縮の場合
    - 音声圧縮 IMA ADPCM (fs/2 近くの高域のひずみが大きくなります)
    - １パケットに160サンプル(8000Hz 時は 20ms 間隔)

■ 接続ピン (TWE-Lite 標準アプリの表記を用いる)
AI1  --- アナログ入力 (0-2.4V, 1.2V 中心だが多少DCが載っていても動作はします)
PWM1 --- 無線からの音声出力 (PWM として)
PWM2 --- A1の入力に応じたPWM出力 (パススルー)
PWM3 --- サンプリング周期の矩形波（ハード出力）

DI1  --- LO 時に音声送信（ボタンを接続、モジュール内でプルアップしているので直結可能）
DI2  --- LO 時にサンプルトーン出力（ボタンまたはスイッチ、モジュール内でプルアップしているので直結可能）

DO1  --- LO 時に無線経由の音声データの出力中
DO2  --- LO 時にテストトーン出力（DI2 の押下状態と同じ）

■ 使い方
(1) TWE-Liteには全て同じファームウェア(App_Audio*.bin)を書き込みます。
(2) 事前にインタラクティブモードで設定を済ませておきます。
(3) AI1, PWM1, DI1 を接続(参考回路図参照)したトランシーバーを２ヶ（以上でも可）
    作成します。
(4) ２ヶのトランシーバーの電源を投入します。
(5) 片方でボタンを押せば、他方から音が出ます。

■ 無線ネットワーク
親機子機と言ったトポロジーはなく、ボタンを押している無線モジュールからの
データを他の無線モジュールが受信して、音声出力します。

同時にボタンを押すと混信により（殆どの場合）音が出なくなります。

■ インタラクティブモード
--- CONFIG/TWE AUDIO APP V0-03-2/SID=0x81000038/LID=0x78 ---
 a: set Application ID (0x67720108)
 i: set Device ID (--)
 c: set Channels (18)
 x: set Tx Power (3)
 p: set PWM HZ (32000)
 f: set Sample Freq (8000)
 d: set Codec Type (0)
 o: set Option Bits (0x00001130)
 b: set UART baud (38400)
---
 S: save Configuration
 R: reset to Defaults

[PWM HZ]
  PWM の周波数を決定します。
  例えば 32000Hz の場合、16MHz/32000=500 カウントによる DUTY 比の変更が可能です。
  ディジタルオーディオで一般的な16bit(=65536)に対して、約9bit(=512段階)と粗くな
  ります。
  周波数を下げると音質（ダイナミックレンジやS/N）が向上するかもしれませんが、
  PWM の制御タイミングなどの揺らぎなどの影響、PWM 周波数自体が音質に影響しやすく
  なりますので、バランスの良いポイントを選ぶ必要があります。

[Sample Freq]
  ADC のサンプリング周波数 (fs) です。fs/2 Hz 以上の信号が含まれると折り返し
  ノイズが発生します。8000Hz なら 4000Hz 以上の音声は含まないようにするか、
  LPF (ローパスフィルタ) により抑え込みます。

  周波数を上げると音質は向上しますが、無線の通信が追いつかなくなります。
  8000Hz では 20ms 毎に無線通信を行います。

[Codec Type]
  ADCから得られたデータ、無線パケットでも送信できるように変換します。
    0: IMA ADPCM
       簡易的な適応的差分パルス符号変調です。1サンプルは4bitで表現されます。
       １パケットは160サンプルでサンプル周期最大8KHzまで伝送できます。
    1: RAW 10bit
       10bit ADデータをそのまま伝送します。
       １パケットは64サンプルでサンプル周期最大3.2KHzまで伝送できます。
    2: RAW 8bit
       10bit ADデータの下２ビットを間引いて 8bit にして伝送します。
       １パケットは80サンプルでサンプル周期最大4KHzまで伝送出来ます。

[オプションビット]
  00000010 --- 入力時、２サンプルの平均を行う(簡易LPF)
  00000020 --- 出力時、２サンプルの平均を行う(簡易LPF)
  00000100 --- 入力時に倍速でサンプルし、２つのサンプルを平均化する。
    ２倍速(16Khz)でサンプルし２サンプルの平均を行い、8Khzの信号として取り扱う
    ※ 上記簡易LPFを適用するとさらに 8Khz の信号を 4KHz 相当として入出力する。
  00001000 --- PWM1のDuty変更時に、PWM1周期の終了間際までポーリング待ちを行う
  00002000 --- PWM2のDuty変更時に、PWM1周期の終了間際までポーリング待ちを行う

  ・デフォルト値は 0x1130 です。
  ・入出力回路の調整など、無線伝送せずに単独で AI1->PWM2 へのパススルーの
    実験を行う場合は、0x2100 または 0x2000 に設定します。
    なお、簡易LPFはパススルーには働きません。

■ 制限事項
・入力と出力は、遅延の発生しうる（無線優先であるため）割り込みハンドラ中で
  行っており、原理的にジッターなどが発生します。
・トランシーバー間で動作クロックの同期を行う事は不可能であるため、バッファー
  アンダー・オーバーランにより音途切れが発生します。
・通信状況が悪くなると無線パケットの欠落による音切れもより発生します。
  緩和するため、本アプリケーションでは同じパケットを２度送っています。
・信号伝送の RAW モードでも、信号の連続性等は厳密に検証されません。

■ IMA ADPCM について
広く使われている(簡易)ADPCMです。

音質面では良好ではないのですが、計算量の少なさと状態ベクトルが小さいため
取り扱いが良いです。

/***********************************************************
 Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
 Netherlands.

 All Rights Reserved

 Permission to use, copy, modify, and distribute this software and its
 documentation for any purpose and without fee is hereby granted,
 provided that the above copyright notice appear in all copies and that
 both that copyright notice and this permission notice appear in
 supporting documentation, and that the names of Stichting Mathematisch
 Centrum or CWI not be used in advertising or publicity pertaining to
 distribution of the software without specific, written prior permission.
 ******************************************************************/

/*
 ** Intel/DVI ADPCM coder/decoder.
 **
 ** The algorithm for this coder was taken from the IMA Compatability Project
 ** proceedings, Vol 2, Number 2; May 1992.
 **
 ** Version 1.2, 18-Dec-92.
 **
 */
