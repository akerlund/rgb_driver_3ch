EESchema Schematic File Version 2
LIBS:frame_strip-rescue
LIBS:power
LIBS:w_analog
LIBS:w_connectors
LIBS:w_device
LIBS:w_logic
LIBS:w_memory
LIBS:w_microcontrollers
LIBS:w_opto
LIBS:w_relay
LIBS:w_rtx
LIBS:w_transistor
LIBS:w_vacuum
LIBS:crf_1
LIBS:freakuency
LIBS:frame_strip-cache
EELAYER 25 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 1
Title ""
Date "2016-08-01"
Rev ""
Comp "Fredrik Åkerlund"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L +3.3V #PWR01
U 1 1 57994060
P 1050 1050
F 0 "#PWR01" H 1050 1010 30  0001 C CNN
F 1 "+3.3V" H 1050 1160 30  0000 C CNN
F 2 "" H 1050 1050 60  0000 C CNN
F 3 "" H 1050 1050 60  0000 C CNN
	1    1050 1050
	1    0    0    -1  
$EndComp
$Comp
L STM32F100_LQFP48 U1
U 1 1 579F9D33
P 5300 2750
F 0 "U1" H 6400 1600 60  0000 C CNN
F 1 "STM32F100_LQFP48" H 4500 700 60  0000 C CNN
F 2 "w_smd_lqfp:lqfp48" H 4000 3950 60  0001 C CNN
F 3 "" H 4000 3950 60  0000 C CNN
	1    5300 2750
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 579F9E7A
P 1050 1150
F 0 "C1" H 1100 1160 50  0000 L CNN
F 1 "C" H 1100 1090 50  0000 L CNN
F 2 "w_smd_cap:c_0603" H 1050 1150 60  0001 C CNN
F 3 "" H 1050 1150 60  0000 C CNN
	1    1050 1150
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 579F9EDB
P 1250 1150
F 0 "C2" H 1300 1160 50  0000 L CNN
F 1 "C" H 1300 1090 50  0000 L CNN
F 2 "w_smd_cap:c_0603" H 1250 1150 60  0001 C CNN
F 3 "" H 1250 1150 60  0000 C CNN
	1    1250 1150
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 579F9F09
P 1450 1150
F 0 "C3" H 1500 1160 50  0000 L CNN
F 1 "C" H 1500 1090 50  0000 L CNN
F 2 "w_smd_cap:c_0603" H 1450 1150 60  0001 C CNN
F 3 "" H 1450 1150 60  0000 C CNN
	1    1450 1150
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 579F9F2F
P 1650 1150
F 0 "C4" H 1700 1160 50  0000 L CNN
F 1 "C" H 1700 1090 50  0000 L CNN
F 2 "w_smd_cap:c_0603" H 1650 1150 60  0001 C CNN
F 3 "" H 1650 1150 60  0000 C CNN
	1    1650 1150
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 579F9F54
P 1850 1150
F 0 "C5" H 1900 1160 50  0000 L CNN
F 1 "C" H 1900 1090 50  0000 L CNN
F 2 "w_smd_cap:c_0603" H 1850 1150 60  0001 C CNN
F 3 "" H 1850 1150 60  0000 C CNN
	1    1850 1150
	1    0    0    -1  
$EndComp
Text Label 3750 2200 2    60   ~ 0
USART1_TX
Text Label 3750 2300 2    60   ~ 0
USART1_RX
Text Label 3750 3200 2    60   ~ 0
BOOT1
Text Label 7050 2000 0    60   ~ 0
BOOT0
$Comp
L R R2
U 1 1 579FA2B1
P 7050 1750
F 0 "R2" V 7000 1750 50  0000 C CNN
F 1 "R" V 7100 1750 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 7050 1750 60  0001 C CNN
F 3 "" H 7050 1750 60  0000 C CNN
	1    7050 1750
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 579FA336
P 7250 1750
F 0 "R3" V 7200 1750 50  0000 C CNN
F 1 "R" V 7300 1750 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 7250 1750 60  0001 C CNN
F 3 "" H 7250 1750 60  0000 C CNN
	1    7250 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2000 7050 1850
Wire Wire Line
	7050 1650 7250 1650
Wire Wire Line
	7250 1850 7350 1850
Text Label 7350 1850 0    60   ~ 0
BOOT1
Wire Wire Line
	6050 900  7050 900 
Connection ~ 6350 900 
Connection ~ 6250 900 
Connection ~ 6150 900 
$Comp
L R R1
U 1 1 579FA458
P 7050 1000
F 0 "R1" H 6950 1000 50  0000 C CNN
F 1 "R" H 7100 1000 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 7050 1000 60  0001 C CNN
F 3 "" H 7050 1000 60  0000 C CNN
	1    7050 1000
	1    0    0    -1  
$EndComp
Connection ~ 6450 900 
Wire Wire Line
	7050 1100 7050 1200
Text Label 7050 1200 0    60   ~ 0
RESET
Wire Wire Line
	6450 4900 6750 4900
Connection ~ 6650 4900
Connection ~ 6550 4900
Text Label 6750 4900 0    60   ~ 0
GND
Text Label 7250 1650 0    60   ~ 0
GND
Text Label 1600 1700 2    60   ~ 0
BOOT0
Text Label 1600 1800 2    60   ~ 0
RESET
Text Label 1600 1900 2    60   ~ 0
USART1_RX
Text Label 1600 2000 2    60   ~ 0
USART1_TX
Text Label 1600 2100 2    60   ~ 0
GND
$Comp
L MOS_N_GDS Q1
U 1 1 579FAB15
P 4700 5900
F 0 "Q1" H 4900 5900 70  0000 C CNN
F 1 "MOS_N_GDS" V 4900 5900 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 4700 5900 60  0001 C CNN
F 3 "" H 4700 5900 60  0000 C CNN
	1    4700 5900
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 579FAE14
P 4600 6000
F 0 "R4" H 4500 5950 50  0000 C CNN
F 1 "R" H 4500 6050 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 4600 6000 60  0001 C CNN
F 3 "" H 4600 6000 60  0000 C CNN
	1    4600 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 6100 4800 6100
Wire Wire Line
	4800 6100 4800 6050
Text Label 4800 6100 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q2
U 1 1 579FBAE7
P 5250 5900
F 0 "Q2" H 5450 5900 70  0000 C CNN
F 1 "MOS_N_GDS" V 5450 5900 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 5250 5900 60  0001 C CNN
F 3 "" H 5250 5900 60  0000 C CNN
	1    5250 5900
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 579FBAED
P 5150 6000
F 0 "R5" H 5050 5950 50  0000 C CNN
F 1 "R" H 5050 6050 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 5150 6000 60  0001 C CNN
F 3 "" H 5150 6000 60  0000 C CNN
	1    5150 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 6100 5350 6100
Wire Wire Line
	5350 6100 5350 6050
Text Label 5350 6100 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q3
U 1 1 579FBB7C
P 5800 5900
F 0 "Q3" H 6000 5900 70  0000 C CNN
F 1 "MOS_N_GDS" V 6000 5900 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 5800 5900 60  0001 C CNN
F 3 "" H 5800 5900 60  0000 C CNN
	1    5800 5900
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 579FBB82
P 5700 6000
F 0 "R6" H 5600 5950 50  0000 C CNN
F 1 "R" H 5600 6050 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 5700 6000 60  0001 C CNN
F 3 "" H 5700 6000 60  0000 C CNN
	1    5700 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6100 5900 6100
Wire Wire Line
	5900 6100 5900 6050
Text Label 5900 6100 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q4
U 1 1 579FBF99
P 4700 6600
F 0 "Q4" H 4900 6600 70  0000 C CNN
F 1 "MOS_N_GDS" V 4900 6600 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 4700 6600 60  0001 C CNN
F 3 "" H 4700 6600 60  0000 C CNN
	1    4700 6600
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 579FBF9F
P 4600 6700
F 0 "R7" H 4500 6650 50  0000 C CNN
F 1 "R" H 4500 6750 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 4600 6700 60  0001 C CNN
F 3 "" H 4600 6700 60  0000 C CNN
	1    4600 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 6800 4800 6800
Wire Wire Line
	4800 6800 4800 6750
Text Label 4800 6800 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q5
U 1 1 579FBFA8
P 5250 6600
F 0 "Q5" H 5450 6600 70  0000 C CNN
F 1 "MOS_N_GDS" V 5450 6600 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 5250 6600 60  0001 C CNN
F 3 "" H 5250 6600 60  0000 C CNN
	1    5250 6600
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 579FBFAE
P 5150 6700
F 0 "R8" H 5050 6650 50  0000 C CNN
F 1 "R" H 5050 6750 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 5150 6700 60  0001 C CNN
F 3 "" H 5150 6700 60  0000 C CNN
	1    5150 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 6800 5350 6800
Wire Wire Line
	5350 6800 5350 6750
Text Label 5350 6800 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q6
U 1 1 579FBFB7
P 5800 6600
F 0 "Q6" H 6000 6600 70  0000 C CNN
F 1 "MOS_N_GDS" V 6000 6600 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 5800 6600 60  0001 C CNN
F 3 "" H 5800 6600 60  0000 C CNN
	1    5800 6600
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 579FBFBD
P 5700 6700
F 0 "R9" H 5600 6650 50  0000 C CNN
F 1 "R" H 5600 6750 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 5700 6700 60  0001 C CNN
F 3 "" H 5700 6700 60  0000 C CNN
	1    5700 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6800 5900 6800
Wire Wire Line
	5900 6800 5900 6750
Text Label 5900 6800 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q7
U 1 1 579FC0F4
P 4700 7300
F 0 "Q7" H 4900 7300 70  0000 C CNN
F 1 "MOS_N_GDS" V 4900 7300 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 4700 7300 60  0001 C CNN
F 3 "" H 4700 7300 60  0000 C CNN
	1    4700 7300
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 579FC0FA
P 4600 7400
F 0 "R10" H 4500 7350 50  0000 C CNN
F 1 "R" H 4500 7450 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 4600 7400 60  0001 C CNN
F 3 "" H 4600 7400 60  0000 C CNN
	1    4600 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 7500 4800 7500
Wire Wire Line
	4800 7500 4800 7450
Text Label 4800 7500 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q8
U 1 1 579FC103
P 5250 7300
F 0 "Q8" H 5450 7300 70  0000 C CNN
F 1 "MOS_N_GDS" V 5450 7300 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 5250 7300 60  0001 C CNN
F 3 "" H 5250 7300 60  0000 C CNN
	1    5250 7300
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 579FC109
P 5150 7400
F 0 "R11" H 5050 7350 50  0000 C CNN
F 1 "R" H 5050 7450 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 5150 7400 60  0001 C CNN
F 3 "" H 5150 7400 60  0000 C CNN
	1    5150 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 7500 5350 7500
Wire Wire Line
	5350 7500 5350 7450
Text Label 5350 7500 0    60   ~ 0
GND
$Comp
L MOS_N_GDS Q9
U 1 1 579FC112
P 5800 7300
F 0 "Q9" H 6000 7300 70  0000 C CNN
F 1 "MOS_N_GDS" V 6000 7300 60  0001 C CNN
F 2 "w_smd_trans:sot23" H 5800 7300 60  0001 C CNN
F 3 "" H 5800 7300 60  0000 C CNN
	1    5800 7300
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 579FC118
P 5700 7400
F 0 "R12" H 5600 7350 50  0000 C CNN
F 1 "R" H 5600 7450 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 5700 7400 60  0001 C CNN
F 3 "" H 5700 7400 60  0000 C CNN
	1    5700 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 7500 5900 7500
Wire Wire Line
	5900 7500 5900 7450
Text Label 5900 7500 0    60   ~ 0
GND
Text Label 3750 1300 2    60   ~ 0
TIM2_CH2
Text Label 3750 3900 2    60   ~ 0
TIM4_CH3
Text Label 3750 4000 2    60   ~ 0
TIM4_CH4
Text Label 3750 1900 2    60   ~ 0
TIM3_CH2
Text Label 3750 2100 2    60   ~ 0
TIM1_CH1
Text Label 3750 3000 2    60   ~ 0
TIM3_CH3
Text Label 3750 3100 2    60   ~ 0
TIM3_CH4
Text Label 3750 3600 2    60   ~ 0
TIM4_CH1
Text Label 3750 3700 2    60   ~ 0
TIM4_CH2
Text Label 5150 6600 1    60   ~ 0
TIM2_CH2
Text Label 5700 5900 1    60   ~ 0
TIM4_CH3
Text Label 4600 6600 1    60   ~ 0
TIM4_CH4
Text Label 5700 6600 1    60   ~ 0
TIM3_CH2
Text Label 5700 7300 1    60   ~ 0
TIM1_CH1
Text Label 4600 7300 1    60   ~ 0
TIM3_CH3
Text Label 5150 7300 1    60   ~ 0
TIM3_CH4
Text Label 4600 5900 1    60   ~ 0
TIM4_CH1
Text Label 5150 5900 1    60   ~ 0
TIM4_CH2
Text Label 4800 5750 0    60   ~ 0
1A
Text Label 5350 5750 0    60   ~ 0
1B
Text Label 5900 5750 0    60   ~ 0
1C
Text Label 4800 6450 0    60   ~ 0
2A
Text Label 5350 6450 0    60   ~ 0
2B
Text Label 5900 6450 0    60   ~ 0
2C
Text Label 4800 7150 0    60   ~ 0
3A
Text Label 5350 7150 0    60   ~ 0
3B
Text Label 5900 7150 0    60   ~ 0
3C
$Comp
L HEADER_4 J3
U 1 1 579FD5A2
P 6300 5900
F 0 "J3" H 6300 6150 60  0000 C CNN
F 1 "HEADER_4" H 6300 5650 60  0001 C CNN
F 2 "w_pin_strip:pin_strip_4" H 6300 5900 60  0001 C CNN
F 3 "" H 6300 5900 60  0000 C CNN
	1    6300 5900
	-1   0    0    1   
$EndComp
$Comp
L HEADER_4 J4
U 1 1 579FD803
P 6300 6600
F 0 "J4" H 6300 6850 60  0000 C CNN
F 1 "HEADER_4" H 6300 6350 60  0001 C CNN
F 2 "w_pin_strip:pin_strip_4" H 6300 6600 60  0001 C CNN
F 3 "" H 6300 6600 60  0000 C CNN
	1    6300 6600
	-1   0    0    1   
$EndComp
$Comp
L HEADER_4 J5
U 1 1 579FD8A7
P 6300 7300
F 0 "J5" H 6300 7550 60  0000 C CNN
F 1 "HEADER_4" H 6300 7050 60  0001 C CNN
F 2 "w_pin_strip:pin_strip_4" H 6300 7300 60  0001 C CNN
F 3 "" H 6300 7300 60  0000 C CNN
	1    6300 7300
	-1   0    0    1   
$EndComp
Text Label 6400 5950 0    60   ~ 0
1A
Text Label 6400 5850 0    60   ~ 0
1B
Text Label 6400 5750 0    60   ~ 0
1C
Text Label 6400 6650 0    60   ~ 0
2A
Text Label 6400 6550 0    60   ~ 0
2B
Text Label 6400 6450 0    60   ~ 0
2C
Text Label 6400 7350 0    60   ~ 0
3A
Text Label 6400 7250 0    60   ~ 0
3B
Text Label 6400 7150 0    60   ~ 0
3C
Text Label 6400 6750 0    60   ~ 0
12V
Text Label 6400 7450 0    60   ~ 0
12V
Text Label 6400 6050 0    60   ~ 0
12V
$Comp
L GND #PWR02
U 1 1 579FE497
P 1050 1350
F 0 "#PWR02" H 1050 1350 30  0001 C CNN
F 1 "GND" H 1050 1280 30  0001 C CNN
F 2 "" H 1050 1350 60  0000 C CNN
F 3 "" H 1050 1350 60  0000 C CNN
	1    1050 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 1050 1050 1100
Wire Wire Line
	1050 1100 1850 1100
Connection ~ 1250 1100
Connection ~ 1450 1100
Connection ~ 1650 1100
Wire Wire Line
	1050 1250 1850 1250
Connection ~ 1650 1250
Connection ~ 1450 1250
Connection ~ 1250 1250
Wire Wire Line
	1050 1350 1050 1250
Text Label 1850 1100 0    60   ~ 0
3.3V
Text Label 1050 1350 2    60   ~ 0
GND
$Comp
L HEADER_3 J2
U 1 1 579FF302
P 1700 2600
F 0 "J2" H 1700 2800 60  0000 C CNN
F 1 "USART2" V 1850 2600 60  0000 C CNN
F 2 "w_pin_strip:pin_strip_3" H 1700 2600 60  0001 C CNN
F 3 "" H 1700 2600 60  0000 C CNN
	1    1700 2600
	1    0    0    -1  
$EndComp
Text Label 3750 1400 2    60   ~ 0
USART2_TX
Text Label 3750 1500 2    60   ~ 0
USART2_RX
Text Label 1600 2700 2    60   ~ 0
GND
$Comp
L SW_PUSH SW1
U 1 1 57A00D8C
P 1300 3400
F 0 "SW1" H 1310 3520 50  0000 C CNN
F 1 "SW_PUSH" H 1300 3320 50  0000 C CNN
F 2 "w_switch:smd_push" H 1300 3400 60  0001 C CNN
F 3 "" H 1300 3400 60  0000 C CNN
	1    1300 3400
	0    -1   -1   0   
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 57A00E70
P 1650 3400
F 0 "SW2" H 1660 3520 50  0000 C CNN
F 1 "SW_PUSH" H 1650 3320 50  0000 C CNN
F 2 "w_switch:smd_push" H 1650 3400 60  0001 C CNN
F 3 "" H 1650 3400 60  0000 C CNN
	1    1650 3400
	0    1    1    0   
$EndComp
$Comp
L R R13
U 1 1 57A0113C
P 1300 3700
F 0 "R13" H 1200 3700 50  0000 C CNN
F 1 "R" H 1350 3700 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 1300 3700 60  0001 C CNN
F 3 "" H 1300 3700 60  0000 C CNN
	1    1300 3700
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 57A01522
P 1650 3700
F 0 "R14" H 1550 3700 50  0000 C CNN
F 1 "R" H 1700 3700 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 1650 3700 60  0001 C CNN
F 3 "" H 1650 3700 60  0000 C CNN
	1    1650 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3800 2250 3800
Text Label 1650 3250 0    60   ~ 0
3.3V
Text Label 1550 3800 2    60   ~ 0
GND
Text Label 1800 3600 0    60   ~ 0
SW2
Text Label 1150 3600 2    60   ~ 0
SW1
Wire Wire Line
	1650 3250 1650 3200
Wire Wire Line
	1650 3200 1300 3200
Wire Wire Line
	1300 3200 1300 3250
Wire Wire Line
	1300 3600 1300 3550
Wire Wire Line
	1300 3600 1150 3600
Wire Wire Line
	1650 3550 1650 3600
Wire Wire Line
	1650 3600 1800 3600
Text Label 3750 2400 2    60   ~ 0
SW1
Text Label 3750 2500 2    60   ~ 0
SW2
Text Label 3750 1600 2    60   ~ 0
ADC1_4
Text Label 3750 1700 2    60   ~ 0
ADC1_5
NoConn ~ 3750 3400
$Comp
L SW_SPDT SW3
U 1 1 57A03707
P 2000 8000
F 0 "SW3" H 2000 7900 50  0000 C CNN
F 1 "ON/OFF" H 2000 8150 50  0000 C CNN
F 2 "w_switch:switch_mmp122-r" H 2000 8000 60  0001 C CNN
F 3 "" H 2000 8000 60  0000 C CNN
	1    2000 8000
	1    0    0    -1  
$EndComp
$Comp
L DC_POWER_JACK J7
U 1 1 57A03774
P 1500 8100
F 0 "J7" H 1500 8350 60  0000 C CNN
F 1 "DC_POWER_JACK" H 1500 7850 60  0000 C CNN
F 2 "w_conn_misc:dc_socket" H 1500 8100 60  0001 C CNN
F 3 "" H 1500 8100 60  0000 C CNN
	1    1500 8100
	-1   0    0    -1  
$EndComp
$Comp
L HEADER_2 J6
U 1 1 57A03BC6
P 2300 7650
F 0 "J6" V 2350 7900 60  0000 C CNN
F 1 "HEADER_2" V 2500 7650 60  0000 C CNN
F 2 "w_pin_strip:pin_socket_2" H 2300 7650 60  0001 C CNN
F 3 "" H 2300 7650 60  0000 C CNN
	1    2300 7650
	0    -1   -1   0   
$EndComp
$Comp
L C C6
U 1 1 57A04670
P 2450 8000
F 0 "C6" H 2500 8010 50  0000 L CNN
F 1 "C" H 2500 7940 50  0000 L CNN
F 2 "w_smd_cap:c_elec_5x5.3" H 2450 8000 60  0001 C CNN
F 3 "" H 2450 8000 60  0000 C CNN
	1    2450 8000
	1    0    0    -1  
$EndComp
Text Label 1800 7750 2    60   ~ 0
12V
Text Label 2450 8300 0    60   ~ 0
GND
$Comp
L sot223 U6
U 1 1 57A04F95
P 3100 8100
F 0 "U6" H 2950 8350 60  0000 C CNN
F 1 "sot223" H 3100 8500 60  0001 C CNN
F 2 "w_smd_trans:sot223" H 3100 8100 60  0001 C CNN
F 3 "" H 3100 8100 60  0000 C CNN
	1    3100 8100
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 57A05675
P 3700 8000
F 0 "C7" H 3750 8010 50  0000 L CNN
F 1 "C" H 3750 7940 50  0000 L CNN
F 2 "w_smd_cap:c_0603" H 3700 8000 60  0001 C CNN
F 3 "" H 3700 8000 60  0000 C CNN
	1    3700 8000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 8100 2450 8300
Wire Wire Line
	1800 8150 2450 8150
Wire Wire Line
	2450 8300 3700 8300
Connection ~ 2450 8150
Wire Wire Line
	2150 7950 2650 7950
Connection ~ 2450 7950
Connection ~ 2250 7950
Wire Wire Line
	2350 7750 2350 8150
Connection ~ 2350 8150
Wire Wire Line
	3600 7850 3600 7950
Wire Wire Line
	3600 7950 3700 7950
Wire Wire Line
	3700 8300 3700 8100
Connection ~ 3100 8300
$Comp
L HEADER_6 J1
U 1 1 57A05EED
P 1700 1950
F 0 "J1" H 1700 2300 60  0000 C CNN
F 1 "PROGRAM" V 1850 1950 60  0000 C CNN
F 2 "w_pin_strip:pin_strip_6" H 1700 1950 60  0001 C CNN
F 3 "" H 1700 1950 60  0000 C CNN
	1    1700 1950
	1    0    0    -1  
$EndComp
Text Label 1600 2200 2    60   ~ 0
3.3V
Text Label 3600 7950 0    60   ~ 0
3.3V
Wire Wire Line
	1800 8000 1850 8000
Wire Wire Line
	1800 8100 1800 8200
Connection ~ 1800 8150
NoConn ~ 2150 8050
$Comp
L R R15
U 1 1 57A0826C
P 1550 4350
F 0 "R15" H 1450 4350 50  0000 C CNN
F 1 "R" H 1400 4250 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 1550 4350 60  0001 C CNN
F 3 "" H 1550 4350 60  0000 C CNN
	1    1550 4350
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 57A08504
P 1800 4350
F 0 "R16" H 1700 4350 50  0000 C CNN
F 1 "R" H 1650 4250 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 1800 4350 60  0001 C CNN
F 3 "" H 1800 4350 60  0000 C CNN
	1    1800 4350
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 57A085B4
P 2050 4350
F 0 "R17" H 1950 4350 50  0000 C CNN
F 1 "R" H 1900 4250 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 2050 4350 60  0001 C CNN
F 3 "" H 2050 4350 60  0000 C CNN
	1    2050 4350
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 57A0865F
P 2300 4350
F 0 "R18" H 2200 4350 50  0000 C CNN
F 1 "R" H 2150 4250 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 2300 4350 60  0001 C CNN
F 3 "" H 2300 4350 60  0000 C CNN
	1    2300 4350
	1    0    0    -1  
$EndComp
$Comp
L LED LD1
U 1 1 57A086DD
P 1550 4550
F 0 "LD1" H 1550 4650 40  0000 C CNN
F 1 "LED1" H 1600 4450 40  0000 C CNN
F 2 "w_smd_leds:Led_0805" H 1550 4550 60  0001 C CNN
F 3 "" H 1550 4550 60  0000 C CNN
	1    1550 4550
	0    1    1    0   
$EndComp
$Comp
L LED LD2
U 1 1 57A087C3
P 1800 4550
F 0 "LD2" H 1800 4650 40  0000 C CNN
F 1 "LED2" H 1850 4450 40  0000 C CNN
F 2 "w_smd_leds:Led_0805" H 1800 4550 60  0001 C CNN
F 3 "" H 1800 4550 60  0000 C CNN
	1    1800 4550
	0    1    1    0   
$EndComp
$Comp
L LED LD3
U 1 1 57A0887B
P 2050 4550
F 0 "LD3" H 2050 4650 40  0000 C CNN
F 1 "LED3" H 2100 4450 40  0000 C CNN
F 2 "w_smd_leds:Led_0805" H 2050 4550 60  0001 C CNN
F 3 "" H 2050 4550 60  0000 C CNN
	1    2050 4550
	0    1    1    0   
$EndComp
$Comp
L LED LD4
U 1 1 57A0893A
P 2300 4550
F 0 "LD4" H 2300 4650 40  0000 C CNN
F 1 "LED4" H 2350 4450 40  0000 C CNN
F 2 "w_smd_leds:Led_0805" H 2300 4550 60  0001 C CNN
F 3 "" H 2300 4550 60  0000 C CNN
	1    2300 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 4450 1550 4500
Wire Wire Line
	1800 4500 1800 4450
Wire Wire Line
	2050 4500 2050 4450
Wire Wire Line
	2300 4500 2300 4450
Wire Wire Line
	1550 4650 2300 4650
Connection ~ 2050 4650
Connection ~ 1800 4650
Text Label 1550 4650 3    60   ~ 0
GND
Text Label 1550 4250 0    60   ~ 0
LED1
Text Label 1800 4250 0    60   ~ 0
LED2
Text Label 2050 4250 0    60   ~ 0
LED3
Text Label 2300 4250 0    60   ~ 0
LED4
Text Label 3750 2600 2    60   ~ 0
LED1
Text Label 3750 2700 2    60   ~ 0
LED2
Text Label 3750 2800 2    60   ~ 0
LED3
Text Label 3750 3300 2    60   ~ 0
LED4
Text Label 3750 4400 2    60   ~ 0
SPI1_SCK
Text Label 3750 4500 2    60   ~ 0
SPI1_MISO
Text Label 3750 4600 2    60   ~ 0
SPI1_MOSI
$Comp
L HEADER_5 J8
U 1 1 57A0B1B4
P 2600 1900
F 0 "J8" H 2600 2200 60  0000 C CNN
F 1 "SPI1" V 2750 1900 60  0000 C CNN
F 2 "w_pin_strip:pin_strip_5" H 2600 1900 60  0001 C CNN
F 3 "" H 2600 1900 60  0000 C CNN
	1    2600 1900
	1    0    0    -1  
$EndComp
Text Label 2500 1900 2    60   ~ 0
SPI1_SCK
Text Label 2500 1800 2    60   ~ 0
SPI1_MISO
Text Label 2500 1700 2    60   ~ 0
SPI1_MOSI
Text Label 2500 2000 2    60   ~ 0
GND
Text Label 2500 2100 2    60   ~ 0
3.3V
Text Label 1600 2500 2    60   ~ 0
USART2_TX
Text Label 1600 2600 2    60   ~ 0
USART2_RX
Text Label 3750 1800 2    60   ~ 0
ADC1_6
Text Label 2100 5350 2    60   ~ 0
ADC1_4
Text Label 2100 5800 2    60   ~ 0
ADC1_5
Text Label 2100 6250 2    60   ~ 0
ADC1_6
NoConn ~ 7050 2250
NoConn ~ 7050 2350
NoConn ~ 7050 2450
NoConn ~ 7050 2650
$Comp
L R R19
U 1 1 57A0CF6B
P 2200 5350
F 0 "R19" V 2300 5350 50  0000 C CNN
F 1 "R" V 2100 5350 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 2200 5350 60  0001 C CNN
F 3 "" H 2200 5350 60  0000 C CNN
	1    2200 5350
	0    1    1    0   
$EndComp
$Comp
L POT ADC4
U 1 1 57A0D08D
P 2400 5350
F 0 "ADC4" V 2450 5250 50  0000 C CNN
F 1 "POT" H 2400 5250 50  0001 C CNN
F 2 "w_pin_strip:pin_socket_3" H 2400 5350 60  0001 C CNN
F 3 "" H 2400 5350 60  0000 C CNN
	1    2400 5350
	0    -1   -1   0   
$EndComp
Text Label 2400 5500 0    60   ~ 0
GND
Text Label 2450 5250 0    60   ~ 0
3.3V
Wire Wire Line
	2400 5500 2400 5450
$Comp
L R R20
U 1 1 57A0DD39
P 2200 5800
F 0 "R20" V 2300 5800 50  0000 C CNN
F 1 "R" V 2100 5800 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 2200 5800 60  0001 C CNN
F 3 "" H 2200 5800 60  0000 C CNN
	1    2200 5800
	0    1    1    0   
$EndComp
$Comp
L POT ADC5
U 1 1 57A0DD3F
P 2400 5800
F 0 "ADC5" V 2450 5700 50  0000 C CNN
F 1 "POT" H 2400 5700 50  0001 C CNN
F 2 "w_pin_strip:pin_socket_3" H 2400 5800 60  0001 C CNN
F 3 "" H 2400 5800 60  0000 C CNN
	1    2400 5800
	0    -1   -1   0   
$EndComp
Text Label 2400 5950 0    60   ~ 0
GND
Text Label 2450 5700 0    60   ~ 0
3.3V
Wire Wire Line
	2400 5950 2400 5900
$Comp
L R R21
U 1 1 57A0DF1F
P 2200 6250
F 0 "R21" V 2300 6250 50  0000 C CNN
F 1 "R" V 2100 6250 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" H 2200 6250 60  0001 C CNN
F 3 "" H 2200 6250 60  0000 C CNN
	1    2200 6250
	0    1    1    0   
$EndComp
$Comp
L POT ADC6
U 1 1 57A0DF25
P 2400 6250
F 0 "ADC6" V 2450 6150 50  0000 C CNN
F 1 "POT" H 2400 6150 50  0001 C CNN
F 2 "w_pin_strip:pin_socket_3" H 2400 6250 60  0001 C CNN
F 3 "" H 2400 6250 60  0000 C CNN
	1    2400 6250
	0    -1   -1   0   
$EndComp
Text Label 2400 6400 0    60   ~ 0
GND
Text Label 2450 6150 0    60   ~ 0
3.3V
Wire Wire Line
	2400 6400 2400 6350
Text Label 6050 900  2    60   ~ 0
3.3V
$Comp
L HEADER_1 J9
U 1 1 579FD06D
P 900 10250
F 0 "J9" H 900 10400 60  0000 C CNN
F 1 "HEADER_1" H 900 10100 60  0000 C CNN
F 2 "freaquency:drill_2.0mm" H 900 10250 60  0001 C CNN
F 3 "" H 900 10250 60  0000 C CNN
	1    900  10250
	0    -1   -1   0   
$EndComp
$Comp
L HEADER_1 J10
U 1 1 579FD17A
P 1300 10250
F 0 "J10" H 1300 10400 60  0000 C CNN
F 1 "HEADER_1" H 1300 10100 60  0000 C CNN
F 2 "freaquency:drill_2.0mm" H 1300 10250 60  0001 C CNN
F 3 "" H 1300 10250 60  0000 C CNN
	1    1300 10250
	0    -1   -1   0   
$EndComp
$Comp
L HEADER_1 J11
U 1 1 579FD259
P 1700 10250
F 0 "J11" H 1700 10400 60  0000 C CNN
F 1 "HEADER_1" H 1700 10100 60  0000 C CNN
F 2 "freaquency:drill_2.0mm" H 1700 10250 60  0001 C CNN
F 3 "" H 1700 10250 60  0000 C CNN
	1    1700 10250
	0    -1   -1   0   
$EndComp
$Comp
L HEADER_1 J12
U 1 1 579FD33B
P 2100 10250
F 0 "J12" H 2100 10400 60  0000 C CNN
F 1 "HEADER_1" H 2100 10100 60  0000 C CNN
F 2 "freaquency:drill_2.0mm" H 2100 10250 60  0001 C CNN
F 3 "" H 2100 10250 60  0000 C CNN
	1    2100 10250
	0    -1   -1   0   
$EndComp
Text Label 2100 10350 3    60   ~ 0
GND
Text Label 1700 10350 3    60   ~ 0
GND
Text Label 1300 10350 3    60   ~ 0
GND
Text Label 900  10350 3    60   ~ 0
GND
Wire Wire Line
	2250 7750 1800 7750
Wire Wire Line
	1800 7750 1800 8000
Connection ~ 1800 8000
Wire Wire Line
	2400 5250 2450 5250
Wire Wire Line
	2400 5700 2450 5700
Wire Wire Line
	2400 6150 2450 6150
$Comp
L SW_PUSH SW4
U 1 1 57BD9842
P 2250 3650
F 0 "SW4" H 2260 3770 50  0000 C CNN
F 1 "RESET" H 2250 3570 50  0000 C CNN
F 2 "w_switch:smd_push" H 2250 3650 60  0001 C CNN
F 3 "" H 2250 3650 60  0000 C CNN
	1    2250 3650
	0    -1   -1   0   
$EndComp
Text Label 950  4450 0    60   ~ 0
GND
Text Label 2200 3500 2    60   ~ 0
RESET
Wire Wire Line
	2200 3500 2250 3500
Connection ~ 1650 3800
$Comp
L HEADER_3 J13
U 1 1 57BDC9EB
P 6200 7950
F 0 "J13" H 6327 8003 60  0000 L CNN
F 1 "HEADER_3" H 6327 7897 60  0000 L CNN
F 2 "w_pin_strip:pin_socket_3" H 6200 7950 60  0001 C CNN
F 3 "" H 6200 7950 60  0000 C CNN
	1    6200 7950
	1    0    0    -1  
$EndComp
$Comp
L HEADER_3 J14
U 1 1 57BDCB62
P 6200 8350
F 0 "J14" H 6327 8403 60  0000 L CNN
F 1 "HEADER_3" H 6327 8297 60  0000 L CNN
F 2 "w_pin_strip:pin_socket_3" H 6200 8350 60  0001 C CNN
F 3 "" H 6200 8350 60  0000 C CNN
	1    6200 8350
	1    0    0    -1  
$EndComp
$Comp
L HEADER_3 J15
U 1 1 57BDCC52
P 6200 8750
F 0 "J15" H 6327 8803 60  0000 L CNN
F 1 "HEADER_3" H 6327 8697 60  0000 L CNN
F 2 "w_pin_strip:pin_socket_3" H 6200 8750 60  0001 C CNN
F 3 "" H 6200 8750 60  0000 C CNN
	1    6200 8750
	1    0    0    -1  
$EndComp
Text Label 6100 7850 2    60   ~ 0
TIM4_CH1
Text Label 6100 7950 2    60   ~ 0
TIM4_CH2
Text Label 6100 8050 2    60   ~ 0
TIM4_CH3
Text Label 6100 8250 2    60   ~ 0
TIM4_CH4
Text Label 6100 8350 2    60   ~ 0
TIM2_CH2
Text Label 6100 8450 2    60   ~ 0
TIM3_CH2
Text Label 6100 8650 2    60   ~ 0
TIM3_CH3
Text Label 6100 8750 2    60   ~ 0
TIM3_CH4
Text Label 6100 8850 2    60   ~ 0
TIM1_CH1
NoConn ~ 7050 2550
$Comp
L HEADER_3 J16
U 1 1 57BE2968
P 6200 9150
F 0 "J16" H 6327 9203 60  0000 L CNN
F 1 "I2C2" H 6327 9097 60  0000 L CNN
F 2 "w_pin_strip:pin_socket_3" H 6200 9150 60  0001 C CNN
F 3 "" H 6200 9150 60  0000 C CNN
	1    6200 9150
	1    0    0    -1  
$EndComp
Text Label 6100 9050 2    60   ~ 0
SCL
Text Label 6100 9150 2    60   ~ 0
SDA
Text Label 6100 9250 2    60   ~ 0
PB12
Text Label 3750 4100 2    60   ~ 0
SCL
Text Label 3750 4200 2    60   ~ 0
SDA
Text Label 3750 4300 2    60   ~ 0
PB12
NoConn ~ 3750 1200
NoConn ~ 3750 3500
$EndSCHEMATC
