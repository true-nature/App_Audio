EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:favorites
LIBS:sensors
LIBS:microphone
LIBS:AppAudio-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TWE_LITE_SMD U1
U 1 1 5714F123
P 5000 2600
F 0 "U1" H 5000 2500 50  0000 C CNN
F 1 "TWE_LITE_SMD" H 5000 3400 50  0000 C CNN
F 2 "favorites:TWE-001L-NC7" H 4750 1350 50  0001 C CNN
F 3 "http://mono-wireless.com/jp/products/TWE-Lite/MNO-PDS-TWE001L-JP-2.10-2.pdf" H 5350 1350 50  0001 C CNN
	1    5000 2600
	1    0    0    -1  
$EndComp
$Comp
L MIC_ELECTRET MIC1
U 1 1 57162C9E
P 1000 2800
F 0 "MIC1" H 1000 3050 60  0000 C CNN
F 1 "MIC_ELECTRET" H 1000 2450 60  0000 C CNN
F 2 "favorites:DB-C9767" H 1000 2800 60  0001 C CNN
F 3 "" H 1000 2800 60  0000 C CNN
	1    1000 2800
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 57162E0D
P 1450 2400
F 0 "R1" V 1530 2400 50  0000 C CNN
F 1 "10k" V 1450 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1380 2400 50  0001 C CNN
F 3 "" H 1450 2400 50  0000 C CNN
	1    1450 2400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR01
U 1 1 57162E59
P 1450 2100
F 0 "#PWR01" H 1450 1950 50  0001 C CNN
F 1 "VCC" H 1450 2250 50  0000 C CNN
F 2 "" H 1450 2100 50  0000 C CNN
F 3 "" H 1450 2100 50  0000 C CNN
	1    1450 2100
	1    0    0    -1  
$EndComp
$Comp
L CP C1
U 1 1 57162E7C
P 1700 2700
F 0 "C1" H 1725 2800 50  0000 L CNN
F 1 "33u" H 1725 2600 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D5_L6_P2.5" H 1738 2550 50  0001 C CNN
F 3 "" H 1700 2700 50  0000 C CNN
	1    1700 2700
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 57162EE2
P 2100 2700
F 0 "R2" V 2180 2700 50  0000 C CNN
F 1 "10k" V 2100 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2030 2700 50  0001 C CNN
F 3 "" H 2100 2700 50  0000 C CNN
	1    2100 2700
	0    1    1    0   
$EndComp
$Comp
L LMC6062 U2
U 1 1 57162FF8
P 2700 2800
F 0 "U2" H 2650 3000 50  0000 L CNN
F 1 "LMC6062" H 2650 2550 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 2700 2800 50  0001 C CNN
F 3 "" H 2700 2800 50  0000 C CNN
	1    2700 2800
	1    0    0    1   
$EndComp
$Comp
L VCC #PWR02
U 1 1 571630F1
P 2100 3100
F 0 "#PWR02" H 2100 2950 50  0001 C CNN
F 1 "VCC" H 2100 3250 50  0000 C CNN
F 2 "" H 2100 3100 50  0000 C CNN
F 3 "" H 2100 3100 50  0000 C CNN
	1    2100 3100
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5716310E
P 2100 3300
F 0 "R3" V 2180 3300 50  0000 C CNN
F 1 "1.5k" V 2100 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2030 3300 50  0001 C CNN
F 3 "" H 2100 3300 50  0000 C CNN
	1    2100 3300
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 57163162
P 2100 3700
F 0 "R4" V 2180 3700 50  0000 C CNN
F 1 "1k" V 2100 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2030 3700 50  0001 C CNN
F 3 "" H 2100 3700 50  0000 C CNN
	1    2100 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5716320D
P 3650 4350
F 0 "#PWR03" H 3650 4100 50  0001 C CNN
F 1 "GND" H 3650 4200 50  0000 C CNN
F 2 "" H 3650 4350 50  0000 C CNN
F 3 "" H 3650 4350 50  0000 C CNN
	1    3650 4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 5716325B
P 3350 4550
F 0 "#PWR04" H 3350 4300 50  0001 C CNN
F 1 "GND" H 3350 4400 50  0000 C CNN
F 2 "" H 3350 4550 50  0000 C CNN
F 3 "" H 3350 4550 50  0000 C CNN
	1    3350 4550
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5716329F
P 2550 1950
F 0 "R5" V 2630 1950 50  0000 C CNN
F 1 "57k" V 2550 1950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2480 1950 50  0001 C CNN
F 3 "" H 2550 1950 50  0000 C CNN
	1    2550 1950
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR05
U 1 1 57163382
P 2900 3100
F 0 "#PWR05" H 2900 2950 50  0001 C CNN
F 1 "VCC" H 2900 3250 50  0000 C CNN
F 2 "" H 2900 3100 50  0000 C CNN
F 3 "" H 2900 3100 50  0000 C CNN
	1    2900 3100
	1    0    0    -1  
$EndComp
$Comp
L C_Small C2
U 1 1 57163402
P 2600 3350
F 0 "C2" H 2610 3420 50  0000 L CNN
F 1 "0.1u" H 2610 3270 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L4_W2.5_P2.5" H 2600 3350 50  0001 C CNN
F 3 "" H 2600 3350 50  0000 C CNN
	1    2600 3350
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 571635E4
P 3500 1950
F 0 "R6" V 3580 1950 50  0000 C CNN
F 1 "10k" V 3500 1950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3430 1950 50  0001 C CNN
F 3 "" H 3500 1950 50  0000 C CNN
	1    3500 1950
	0    1    1    0   
$EndComp
$Comp
L POT VR1
U 1 1 57163963
P 3150 1950
F 0 "VR1" H 3150 1870 50  0000 C CNN
F 1 "50k" H 3150 1950 50  0000 C CNN
F 2 "favorites:RV-3362P" H 3150 1950 50  0001 C CNN
F 3 "" H 3150 1950 50  0000 C CNN
	1    3150 1950
	1    0    0    1   
$EndComp
$Comp
L POT VR2
U 1 1 571641D3
P 6300 3100
F 0 "VR2" H 6300 3020 50  0000 C CNN
F 1 "1k" H 6300 3100 50  0000 C CNN
F 2 "favorites:RV-3362P" H 6300 3100 50  0001 C CNN
F 3 "" H 6300 3100 50  0000 C CNN
	1    6300 3100
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 571645C7
P 6700 3100
F 0 "R7" V 6780 3100 50  0000 C CNN
F 1 "22k" V 6700 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6630 3100 50  0001 C CNN
F 3 "" H 6700 3100 50  0000 C CNN
	1    6700 3100
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 571647E0
P 7100 3100
F 0 "R8" V 7180 3100 50  0000 C CNN
F 1 "22k" V 7100 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7030 3100 50  0001 C CNN
F 3 "" H 7100 3100 50  0000 C CNN
	1    7100 3100
	0    1    1    0   
$EndComp
$Comp
L LMC6062 U2
U 2 1 57164A1D
P 7750 3200
F 0 "U2" H 7700 3400 50  0000 L CNN
F 1 "LMC6062" H 7700 2950 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 7750 3200 50  0001 C CNN
F 3 "" H 7750 3200 50  0000 C CNN
	2    7750 3200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 57166BEB
P 7600 2400
F 0 "C4" H 7610 2470 50  0000 L CNN
F 1 "3300p" H 7610 2320 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L4_W2.5_P2.5" H 7600 2400 50  0001 C CNN
F 3 "" H 7600 2400 50  0000 C CNN
	1    7600 2400
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR06
U 1 1 5716738B
P 5950 1650
F 0 "#PWR06" H 5950 1500 50  0001 C CNN
F 1 "VCC" H 5950 1800 50  0000 C CNN
F 2 "" H 5950 1650 50  0000 C CNN
F 3 "" H 5950 1650 50  0000 C CNN
	1    5950 1650
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR07
U 1 1 57167C5B
P 8000 1000
F 0 "#PWR07" H 8000 850 50  0001 C CNN
F 1 "VCC" H 8000 1150 50  0000 C CNN
F 2 "" H 8000 1000 50  0000 C CNN
F 3 "" H 8000 1000 50  0000 C CNN
	1    8000 1000
	1    0    0    -1  
$EndComp
$Comp
L CP C3
U 1 1 57167E09
P 8000 1350
F 0 "C3" H 8025 1450 50  0000 L CNN
F 1 "220u" H 8025 1250 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D6.3_L11.2_P2.5" H 8038 1200 50  0001 C CNN
F 3 "" H 8000 1350 50  0000 C CNN
	1    8000 1350
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 571683E8
P 8350 3200
F 0 "R9" V 8430 3200 50  0000 C CNN
F 1 "100" V 8350 3200 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 8280 3200 50  0001 C CNN
F 3 "" H 8350 3200 50  0000 C CNN
	1    8350 3200
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 571688A5
P 8700 3200
F 0 "C6" H 8725 3300 50  0000 L CNN
F 1 "22u" H 8725 3100 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D6.3_L11.2_P2.5" H 8738 3050 50  0001 C CNN
F 3 "" H 8700 3200 50  0000 C CNN
	1    8700 3200
	0    1    1    0   
$EndComp
$Comp
L C_Small C5
U 1 1 57168D6C
P 7300 3400
F 0 "C5" H 7310 3470 50  0000 L CNN
F 1 "1000p" H 7310 3320 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L4_W2.5_P2.5" H 7300 3400 50  0001 C CNN
F 3 "" H 7300 3400 50  0000 C CNN
	1    7300 3400
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 57169246
P 8000 1800
F 0 "#PWR08" H 8000 1550 50  0001 C CNN
F 1 "GND" H 8000 1650 50  0000 C CNN
F 2 "" H 8000 1800 50  0000 C CNN
F 3 "" H 8000 1800 50  0000 C CNN
	1    8000 1800
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 5716980D
P 3350 3700
F 0 "SW1" H 3500 3810 50  0000 C CNN
F 1 "SW_PUSH" H 3350 3620 50  0000 C CNN
F 2 "favorites:TVGP01-G73BB" H 3350 3700 50  0001 C CNN
F 3 "" H 3350 3700 50  0000 C CNN
	1    3350 3700
	0    -1   1    0   
$EndComp
$Comp
L HT82V739 U3
U 1 1 5718C50C
P 9650 3300
F 0 "U3" H 9650 3050 50  0000 C CNN
F 1 "HT82V739" H 9650 3700 50  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 9350 2900 50  0001 C CNN
F 3 "DOCUMENTATION" H 10050 2900 50  0001 C CNN
	1    9650 3300
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR09
U 1 1 571902A2
P 9000 2850
F 0 "#PWR09" H 9000 2700 50  0001 C CNN
F 1 "VCC" H 9000 3000 50  0000 C CNN
F 2 "" H 9000 2850 50  0000 C CNN
F 3 "" H 9000 2850 50  0000 C CNN
	1    9000 2850
	1    0    0    -1  
$EndComp
$Comp
L SPEAKER SP1
U 1 1 571907BD
P 10700 3250
F 0 "SP1" H 10600 3500 50  0000 C CNN
F 1 "SPEAKER" H 10600 3000 50  0000 C CNN
F 2 "favorites:B02B-XASK-1" H 10700 3250 50  0001 C CNN
F 3 "" H 10700 3250 50  0000 C CNN
	1    10700 3250
	1    0    0    1   
$EndComp
$Comp
L C C7
U 1 1 57190E1C
P 8850 3800
F 0 "C7" H 8875 3900 50  0000 L CNN
F 1 "1u" H 8875 3700 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L4_W2.5_P2.5" H 8888 3650 50  0001 C CNN
F 3 "" H 8850 3800 50  0000 C CNN
	1    8850 3800
	-1   0    0    1   
$EndComp
$Comp
L WRITER-VCC P1
U 1 1 57192A4B
P 4950 5000
F 0 "P1" V 5050 5000 60  0000 C CNN
F 1 "WRITER-VCC" V 5150 5000 60  0000 C CNN
F 2 "favorites:WRITE-7P" H 4900 4950 60  0001 C CNN
F 3 "" H 4900 4950 60  0000 C CNN
	1    4950 5000
	0    -1   1    0   
$EndComp
NoConn ~ 4650 4600
NoConn ~ 4250 2050
NoConn ~ 4250 2150
NoConn ~ 5750 2100
NoConn ~ 5750 2200
NoConn ~ 5750 2900
NoConn ~ 5750 3100
NoConn ~ 4250 3450
NoConn ~ 4250 3550
NoConn ~ 4250 2900
NoConn ~ 4250 3000
NoConn ~ 4250 3100
NoConn ~ 4250 2650
NoConn ~ 5750 3350
NoConn ~ 5750 3450
NoConn ~ 5750 3550
NoConn ~ 5750 3650
$Comp
L Q_NMOS_SGD Q1
U 1 1 57199575
P 6500 5000
F 0 "Q1" H 6800 5050 50  0000 R CNN
F 1 "Q_NMOS_SGD" H 7150 4950 50  0000 R CNN
F 2 "favorites:EMT3" H 6700 5100 50  0001 C CNN
F 3 "" H 6500 5000 50  0000 C CNN
	1    6500 5000
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5719961D
P 6100 4750
F 0 "R10" V 6180 4750 50  0000 C CNN
F 1 "1k" V 6100 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6030 4750 50  0001 C CNN
F 3 "" H 6100 4750 50  0000 C CNN
	1    6100 4750
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 57199765
P 6100 5250
F 0 "R11" V 6180 5250 50  0000 C CNN
F 1 "2.2M" V 6100 5250 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 6030 5250 50  0001 C CNN
F 3 "" H 6100 5250 50  0000 C CNN
	1    6100 5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5719AE34
P 6600 5600
F 0 "#PWR010" H 6600 5350 50  0001 C CNN
F 1 "GND" H 6600 5450 50  0000 C CNN
F 2 "" H 6600 5600 50  0000 C CNN
F 3 "" H 6600 5600 50  0000 C CNN
	1    6600 5600
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG011
U 1 1 5719CDCE
P 8300 1000
F 0 "#FLG011" H 8300 1095 50  0001 C CNN
F 1 "PWR_FLAG" H 8300 1180 50  0000 C CNN
F 2 "" H 8300 1000 50  0000 C CNN
F 3 "" H 8300 1000 50  0000 C CNN
	1    8300 1000
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 571A0BA2
P 6600 4300
F 0 "R12" V 6680 4300 50  0000 C CNN
F 1 "2.2M" V 6600 4300 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 6530 4300 50  0001 C CNN
F 3 "" H 6600 4300 50  0000 C CNN
	1    6600 4300
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR012
U 1 1 571A110C
P 6600 4050
F 0 "#PWR012" H 6600 3900 50  0001 C CNN
F 1 "VCC" H 6600 4200 50  0000 C CNN
F 2 "" H 6600 4050 50  0000 C CNN
F 3 "" H 6600 4050 50  0000 C CNN
	1    6600 4050
	1    0    0    -1  
$EndComp
NoConn ~ 5750 2550
$Comp
L Led_x2 D1
U 1 1 571A26B8
P 6250 2350
F 0 "D1" H 6250 2575 50  0000 C CNN
F 1 "Led_x2" H 6250 2100 50  0000 C CNN
F 2 "favorites:LED-3MM" H 6250 2350 50  0001 C CNN
F 3 "" H 6250 2350 50  0000 C CNN
	1    6250 2350
	-1   0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 571A3B9A
P 6600 2600
F 0 "R13" V 6680 2600 50  0000 C CNN
F 1 "1k" V 6600 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6530 2600 50  0001 C CNN
F 3 "" H 6600 2600 50  0000 C CNN
	1    6600 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 571A4007
P 6600 2800
F 0 "#PWR013" H 6600 2550 50  0001 C CNN
F 1 "GND" H 6600 2650 50  0000 C CNN
F 2 "" H 6600 2800 50  0000 C CNN
F 3 "" H 6600 2800 50  0000 C CNN
	1    6600 2800
	1    0    0    -1  
$EndComp
Text Label 4350 4300 0    60   ~ 0
CFG
Text Label 4350 4400 0    60   ~ 0
TXD
Text Label 4350 4200 0    60   ~ 0
RXD
Text Label 4250 4500 0    60   ~ 0
RST
Text Label 5350 4200 0    60   ~ 0
PRG
Text Label 5350 4500 0    60   ~ 0
GND
Text Label 6700 4600 0    60   ~ 0
QGND
$Comp
L Battery BT1
U 1 1 571AE44D
P 7400 1300
F 0 "BT1" H 7500 1350 50  0000 L CNN
F 1 "Battery" H 7500 1250 50  0000 L CNN
F 2 "favorites:B02B-XASK-1" V 7400 1340 50  0000 C CNN
F 3 "" V 7400 1340 50  0000 C CNN
	1    7400 1300
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR014
U 1 1 571AFE46
P 7050 4600
F 0 "#PWR014" H 7050 4450 50  0001 C CNN
F 1 "VSS" H 7050 4750 50  0000 C CNN
F 2 "" H 7050 4600 50  0000 C CNN
F 3 "" H 7050 4600 50  0000 C CNN
	1    7050 4600
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR015
U 1 1 571B049F
P 7100 3900
F 0 "#PWR015" H 7100 3750 50  0001 C CNN
F 1 "VSS" H 7100 4050 50  0000 C CNN
F 2 "" H 7100 3900 50  0000 C CNN
F 3 "" H 7100 3900 50  0000 C CNN
	1    7100 3900
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR016
U 1 1 571B05EF
P 8550 4000
F 0 "#PWR016" H 8550 3850 50  0001 C CNN
F 1 "VSS" H 8550 4150 50  0000 C CNN
F 2 "" H 8550 4000 50  0000 C CNN
F 3 "" H 8550 4000 50  0000 C CNN
	1    8550 4000
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR017
U 1 1 571B138F
P 1600 3150
F 0 "#PWR017" H 1600 3000 50  0001 C CNN
F 1 "VSS" H 1600 3300 50  0000 C CNN
F 2 "" H 1600 3150 50  0000 C CNN
F 3 "" H 1600 3150 50  0000 C CNN
	1    1600 3150
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR018
U 1 1 571B154F
P 1900 3950
F 0 "#PWR018" H 1900 3800 50  0001 C CNN
F 1 "VSS" H 1900 4100 50  0000 C CNN
F 2 "" H 1900 3950 50  0000 C CNN
F 3 "" H 1900 3950 50  0000 C CNN
	1    1900 3950
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR019
U 1 1 571B18DB
P 2600 2450
F 0 "#PWR019" H 2600 2300 50  0001 C CNN
F 1 "VSS" H 2600 2600 50  0000 C CNN
F 2 "" H 2600 2450 50  0000 C CNN
F 3 "" H 2600 2450 50  0000 C CNN
	1    2600 2450
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR020
U 1 1 571B199A
P 3700 1950
F 0 "#PWR020" H 3700 1800 50  0001 C CNN
F 1 "VSS" H 3700 2100 50  0000 C CNN
F 2 "" H 3700 1950 50  0000 C CNN
F 3 "" H 3700 1950 50  0000 C CNN
	1    3700 1950
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR021
U 1 1 571B1DEF
P 6450 3350
F 0 "#PWR021" H 6450 3200 50  0001 C CNN
F 1 "VSS" H 6450 3500 50  0000 C CNN
F 2 "" H 6450 3350 50  0000 C CNN
F 3 "" H 6450 3350 50  0000 C CNN
	1    6450 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 571B25D9
P 5900 4700
F 0 "#PWR022" H 5900 4450 50  0001 C CNN
F 1 "GND" H 5900 4550 50  0000 C CNN
F 2 "" H 5900 4700 50  0000 C CNN
F 3 "" H 5900 4700 50  0000 C CNN
	1    5900 4700
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG023
U 1 1 571B3284
P 8350 1750
F 0 "#FLG023" H 8350 1845 50  0001 C CNN
F 1 "PWR_FLAG" H 8350 1930 50  0000 C CNN
F 2 "" H 8350 1750 50  0000 C CNN
F 3 "" H 8350 1750 50  0000 C CNN
	1    8350 1750
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 5725EDAD
P 3650 3950
F 0 "SW2" H 3800 4060 50  0000 C CNN
F 1 "SW_PUSH" H 3650 3870 50  0000 C CNN
F 2 "favorites:JB-15HFBP2" H 3650 3950 50  0001 C CNN
F 3 "" H 3650 3950 50  0000 C CNN
	1    3650 3950
	0    -1   1    0   
$EndComp
Wire Wire Line
	4850 4200 4850 4600
Wire Wire Line
	5050 4400 5050 4600
Connection ~ 8000 1750
Wire Wire Line
	8350 1750 8000 1750
Wire Wire Line
	6300 3350 6450 3350
Connection ~ 2100 3950
Wire Wire Line
	1900 3950 2600 3950
Wire Wire Line
	1450 3150 1600 3150
Wire Wire Line
	7100 3900 7300 3900
Connection ~ 8000 1600
Wire Wire Line
	7400 1600 8000 1600
Wire Wire Line
	7400 1450 7400 1600
Wire Wire Line
	7400 1150 7400 1100
Wire Wire Line
	3650 4350 3650 4250
Wire Wire Line
	6600 2800 6600 2750
Wire Wire Line
	6600 2350 6600 2450
Wire Wire Line
	6550 2350 6600 2350
Wire Wire Line
	5850 2250 5950 2250
Wire Wire Line
	5850 2350 5850 2250
Wire Wire Line
	5750 2350 5850 2350
Wire Wire Line
	5750 2450 5950 2450
Wire Wire Line
	3650 3350 4250 3350
Wire Wire Line
	3650 3650 3650 3350
Connection ~ 9000 3500
Wire Wire Line
	9050 3500 9000 3500
Connection ~ 6600 4600
Wire Wire Line
	6600 4050 6600 4150
Connection ~ 8000 1100
Wire Wire Line
	7400 1100 8300 1100
Wire Wire Line
	8300 1100 8300 1000
Wire Wire Line
	6600 4600 7050 4600
Wire Wire Line
	6600 4450 6600 4800
Connection ~ 6600 5500
Wire Wire Line
	6100 5500 6600 5500
Wire Wire Line
	6100 5400 6100 5500
Wire Wire Line
	6600 5200 6600 5600
Connection ~ 6100 5000
Wire Wire Line
	6300 5000 6100 5000
Wire Wire Line
	6100 4900 6100 5100
Wire Wire Line
	6100 2650 6100 4600
Wire Wire Line
	5750 2650 6100 2650
Wire Wire Line
	4950 4200 4950 4600
Wire Wire Line
	6000 4200 4950 4200
Wire Wire Line
	6000 3000 6000 4200
Wire Wire Line
	5750 3000 6000 3000
Wire Wire Line
	5250 4300 5250 4600
Wire Wire Line
	3850 4300 5250 4300
Wire Wire Line
	3850 1950 3850 4300
Wire Wire Line
	4250 1950 3850 1950
Wire Wire Line
	3950 4200 4850 4200
Wire Wire Line
	3950 2300 3950 4200
Wire Wire Line
	4250 2300 3950 2300
Wire Wire Line
	4050 4400 5050 4400
Wire Wire Line
	4050 2400 4050 4400
Wire Wire Line
	4250 2400 4050 2400
Wire Wire Line
	4750 4500 4750 4600
Wire Wire Line
	4150 4500 4750 4500
Wire Wire Line
	4150 2550 4150 4500
Wire Wire Line
	4250 2550 4150 2550
Connection ~ 5900 4500
Wire Wire Line
	5150 4500 5900 4500
Wire Wire Line
	5150 4600 5150 4500
Connection ~ 8850 4000
Wire Wire Line
	9000 3400 9000 4000
Wire Wire Line
	9050 3400 9000 3400
Wire Wire Line
	9000 4000 8550 4000
Wire Wire Line
	8850 3950 8850 4000
Wire Wire Line
	8850 3300 8850 3650
Wire Wire Line
	9050 3300 8850 3300
Wire Wire Line
	10350 3350 10400 3350
Wire Wire Line
	10350 3250 10350 3350
Wire Wire Line
	10250 3250 10350 3250
Wire Wire Line
	10250 3150 10400 3150
Wire Wire Line
	9000 3100 9050 3100
Wire Wire Line
	9000 2850 9000 3100
Wire Wire Line
	9050 3200 8850 3200
Wire Wire Line
	1300 2900 1450 2900
Wire Wire Line
	1450 2900 1450 3150
Wire Wire Line
	1300 2700 1550 2700
Wire Wire Line
	1450 2700 1450 2550
Wire Wire Line
	1450 2100 1450 2250
Connection ~ 1450 2700
Wire Wire Line
	1850 2700 1950 2700
Wire Wire Line
	2250 2700 2400 2700
Wire Wire Line
	2100 3100 2100 3150
Wire Wire Line
	2100 3450 2100 3550
Wire Wire Line
	2100 3850 2100 3950
Wire Wire Line
	2400 2900 2300 2900
Wire Wire Line
	2300 2900 2300 3500
Wire Wire Line
	2300 3500 2100 3500
Connection ~ 2100 3500
Wire Wire Line
	2300 2700 2300 1950
Wire Wire Line
	2300 1950 2400 1950
Connection ~ 2300 2700
Wire Wire Line
	2700 1950 3000 1950
Wire Wire Line
	3300 1950 3350 1950
Wire Wire Line
	3150 2100 3150 2800
Wire Wire Line
	3000 2800 4250 2800
Wire Wire Line
	3650 1950 3700 1950
Connection ~ 3150 2800
Wire Wire Line
	5750 2800 6300 2800
Wire Wire Line
	6300 2800 6300 2950
Wire Wire Line
	6300 3250 6300 3350
Wire Wire Line
	6450 3100 6550 3100
Wire Wire Line
	6850 3100 6950 3100
Wire Wire Line
	7250 3100 7450 3100
Wire Wire Line
	2600 2500 2600 2450
Wire Wire Line
	2600 3100 2600 3250
Wire Wire Line
	2900 3100 2900 3150
Wire Wire Line
	2900 3150 2600 3150
Connection ~ 2600 3150
Wire Wire Line
	2600 3950 2600 3450
Wire Wire Line
	7400 3300 7450 3300
Wire Wire Line
	7400 3800 7400 3300
Wire Wire Line
	8100 3800 7400 3800
Wire Wire Line
	8100 2400 8100 3800
Wire Wire Line
	8050 3200 8200 3200
Wire Wire Line
	6900 3100 6900 2400
Wire Wire Line
	6900 2400 7500 2400
Connection ~ 6900 3100
Wire Wire Line
	8100 2400 7700 2400
Connection ~ 8100 3200
Wire Wire Line
	5950 1650 5950 1950
Wire Wire Line
	5950 1950 5750 1950
Wire Wire Line
	5750 3250 5900 3250
Wire Wire Line
	5900 3250 5900 4700
Wire Wire Line
	8000 1000 8000 1200
Wire Wire Line
	8000 1500 8000 1800
Wire Wire Line
	8500 3200 8550 3200
Wire Wire Line
	7300 3300 7300 3100
Connection ~ 7300 3100
Wire Wire Line
	7300 3900 7300 3500
Wire Wire Line
	4250 3250 3350 3250
Wire Wire Line
	3350 3250 3350 3400
Wire Wire Line
	3350 4550 3350 4000
Text Label 10300 3150 0    60   ~ 0
OUTN
Text Label 10250 3250 0    60   ~ 0
OUTP
Text Label 8850 3200 0    60   ~ 0
Aud_In
Text Label 8900 3300 0    60   ~ 0
VREF
$EndSCHEMATC
