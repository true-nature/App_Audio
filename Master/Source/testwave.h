/****************************************************************************
 *
 * MODULE:             Test Waveform
 *
 * COMPONENT:          $RCSfile: testwave.h,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.4 $
 *
 * DATED:              $Date: 2007/08/03 09:04:24 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             Ian Morris
 *
 * DESCRIPTION:
 *
 * LAST MODIFIED BY:   $Author: snich $
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2005, 2006. All rights reserved
 *
 ****************************************************************************/
#ifndef  TEST_WAVE_H_INCLUDED
#define  TEST_WAVE_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Number of samples in the test waveform */
#define TEST_WAVE_FORM_SAMPLES_8KHZ         20
#define TEST_WAVE_FORM_SAMPLES_16KHZ        40
#define TEST_WAVE_FORM_SAMPLES_24KHZ        53//60
#define TEST_WAVE_FORM_SAMPLES_32KHZ        80
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
/* Samples for test waveform (400Hz sinewave sampled at 8KHz) */
const int16 ai16TestWaveForm8KHz[TEST_WAVE_FORM_SAMPLES_8KHZ] =
{
       0,   2024,   3851,   5301,
    6232,   6553,   6232,   5301,
    3851,   2024,      0,  -2025,
   -3852,  -5302,  -6233,  -6553,
   -6233,  -5302,  -3852,  -2025
};

#if 0
/* Samples for test waveform (400Hz sinewave sampled at 16KHz) */
const int16 ai16TestWaveForm16KHz[TEST_WAVE_FORM_SAMPLES_16KHZ] =
{
       0,   1025,   2024,   2974,
    3851,   4633,   5301,   5838,
    6232,   6472,   6553,   6472,
    6232,   5838,   5301,   4633,
    3851,   2974,   2024,   1025,
       0,  -1026,  -2025,  -2975,
   -3852,  -4634,  -5302,  -5839,
   -6233,  -6473,  -6553,  -6473,
   -6233,  -5839,  -5302,  -4634,
   -3852,  -2975,  -2025,  -1026
};
#endif
const int16 ai16TestWaveForm16KHz[TEST_WAVE_FORM_SAMPLES_16KHZ] =
{
0,
1049,
2071,
3040,
3931,
4720,
5387,
5915,
6290,
6503,
6548,
6424,
6134,
5686,
5090,
4364,
3525,
2594,
1597,
559,
-494,
-1534,
-2535,
-3470,
-4316,
-5050,
-5653,
-6111,
-6411,
-6546,
-6512,
-6309,
-5944,
-5425,
-4766,
-3984,
-3100,
-2135,
-1115,
-66
};




#if 0
const int16 ai16TestWaveForm24KHz[TEST_WAVE_FORM_SAMPLES_24KHZ] =
{
    0,
    684,
    1362,
    2024,
    2665,
    3276,
    3851,
    4384,
    4869,
    5301,
    5675,
    5986,
    6232,
    6409,
    6517,
    6553,
    6517,
    6409,
    6232,
    5986,
    5675,
    5301,
    4869,
    4384,
    3851,
    3276,
    2665,
    2024,
    1362,
    684,
    0,
    -685,
    -1363,
    -2025,
    -2666,
    -3277,
    -3852,
    -4385,
    -4870,
    -5302,
    -5676,
    -5987,
    -6233,
    -6410,
    -6518,
    -6553,
    -6518,
    -6410,
    -6233,
    -5987,
    -5676,
    -5302,
    -4870,
    -4385,
    -3852,
    -3277,
    -2666,
    -2025,
    -1363,
    -685 };
#endif
const int16 ai16TestWaveForm24KHz[TEST_WAVE_FORM_SAMPLES_24KHZ] =
{
0,
789,
1568,
2323,
3045,
3722,
4345,
4904,
5393,
5802,
6127,
6362,
6505,
6553,
6505,
6362,
6127,
5802,
5393,
4904,
4345,
3722,
3045,
2323,
1568,
789,
0,
-790,
-1569,
-2324,
-3046,
-3723,
-4346,
-4905,
-5394,
-5803,
-6128,
-6363,
-6506,
-6553,
-6506,
-6363,
-6128,
-5803,
-5394,
-4905,
-4346,
-3723,
-3046,
-2324,
-1569,
-790
};




const int16 ai16TestWaveForm32KHz[TEST_WAVE_FORM_SAMPLES_32KHZ] =
{ 0, 514, 1025, 1529, 2024, 2507, 2974, 3423, 3851, 4255, 4633, 4982, 5301,
        5587, 5838, 6054, 6232, 6371, 6472, 6532, 6553, 6532, 6472, 6371, 6232,
        6054, 5838, 5587, 5301, 4982, 4633, 4255, 3851, 3423, 2974, 2507, 2024,
        1529, 1025, 514, 0, -515, -1026, -1530, -2025, -2508, -2975, -3424,
        -3852, -4256, -4634, -4983, -5302, -5588, -5839, -6055, -6233, -6372,
        -6473, -6533, -6553, -6533, -6473, -6372, -6233, -6055, -5839, -5588,
        -5302, -4983, -4634, -4256, -3852, -3424, -2975, -2508, -2025, -1530,
        -1026, -515 };




#if defined __cplusplus
}
#endif

#endif  /* TEST_WAVE_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
