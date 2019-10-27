EESchema Schematic File Version 4
LIBS:skytracker_kicad-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SkyTracker Schematic"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R1
U 1 1 5DACCE8F
P 4250 5000
F 0 "R1" V 4457 5000 50  0000 C CNN
F 1 "220" V 4366 5000 50  0000 C CNN
F 2 "" V 4180 5000 50  0001 C CNN
F 3 "~" H 4250 5000 50  0001 C CNN
	1    4250 5000
	0    -1   -1   0   
$EndComp
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5DACFD1C
P 5400 4500
F 0 "A1" H 5400 3411 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 5400 3320 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 5550 3550 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5400 3500 50  0001 C CNN
	1    5400 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5DACE66B
P 4250 5700
F 0 "R3" V 4043 5700 50  0000 C CNN
F 1 "220" V 4134 5700 50  0000 C CNN
F 2 "" V 4180 5700 50  0001 C CNN
F 3 "~" H 4250 5700 50  0001 C CNN
	1    4250 5700
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5DACE114
P 4250 5350
F 0 "R2" V 4043 5350 50  0000 C CNN
F 1 "220" V 4134 5350 50  0000 C CNN
F 2 "" V 4180 5350 50  0001 C CNN
F 3 "~" H 4250 5350 50  0001 C CNN
	1    4250 5350
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5DADFED9
P 3800 5000
F 0 "D1" H 3793 5216 50  0000 C CNN
F 1 "LIMIT_LED" H 3793 5125 50  0000 C CNN
F 2 "" H 3800 5000 50  0001 C CNN
F 3 "~" H 3800 5000 50  0001 C CNN
	1    3800 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5DAE13B5
P 3800 5350
F 0 "D2" H 3793 5566 50  0000 C CNN
F 1 "STATE_LED" H 3793 5475 50  0000 C CNN
F 2 "" H 3800 5350 50  0001 C CNN
F 3 "~" H 3800 5350 50  0001 C CNN
	1    3800 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5DAE1860
P 3800 5700
F 0 "D3" H 3793 5916 50  0000 C CNN
F 1 "POWER_LED" H 3793 5825 50  0000 C CNN
F 2 "" H 3800 5700 50  0001 C CNN
F 3 "~" H 3800 5700 50  0001 C CNN
	1    3800 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 5350 3950 5350
Wire Wire Line
	3950 5000 4100 5000
Wire Wire Line
	3950 5700 4100 5700
Wire Wire Line
	3650 5000 3550 5000
Wire Wire Line
	3550 5000 3550 5350
Wire Wire Line
	3650 5350 3550 5350
Connection ~ 3550 5350
Wire Wire Line
	3550 5350 3550 5700
Wire Wire Line
	3650 5700 3550 5700
Connection ~ 3550 5700
$Comp
L Switch:SW_SPDT_MSM SW3
U 1 1 5DAE50D2
P 3900 4400
F 0 "SW3" H 3900 4685 50  0000 C CNN
F 1 "STATE_SELECT" H 3900 4594 50  0000 C CNN
F 2 "" H 3900 4400 50  0001 C CNN
F 3 "~" H 3900 4400 50  0001 C CNN
	1    3900 4400
	1    0    0    -1  
$EndComp
Connection ~ 3550 4400
Wire Wire Line
	3550 4400 3550 5000
Wire Wire Line
	3550 4400 3700 4400
$Comp
L Switch:SW_Push SW2
U 1 1 5DAF47E4
P 4050 4000
F 0 "SW2" H 4050 4285 50  0000 C CNN
F 1 "LIM_DETECT" H 4050 4194 50  0000 C CNN
F 2 "" H 4050 4200 50  0001 C CNN
F 3 "~" H 4050 4200 50  0001 C CNN
	1    4050 4000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5DAF67F0
P 4050 3500
F 0 "SW1" H 4050 3785 50  0000 C CNN
F 1 "HOME_DETECT" H 4050 3694 50  0000 C CNN
F 2 "" H 4050 3700 50  0001 C CNN
F 3 "~" H 4050 3700 50  0001 C CNN
	1    4050 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3500 4450 3500
Wire Wire Line
	4450 3500 4450 3900
Wire Wire Line
	4450 3900 4900 3900
Wire Wire Line
	3850 3500 3550 3500
Wire Wire Line
	3550 3500 3550 4000
Connection ~ 3550 5000
Wire Wire Line
	3850 4000 3550 4000
Wire Wire Line
	5500 5500 6100 5500
Wire Wire Line
	6100 5500 6100 3350
$Comp
L Transistor_Array:ULN2003A U1
U 1 1 5DB072C3
P 5350 2150
F 0 "U1" H 5350 2817 50  0000 C CNN
F 1 "ULN2003A" H 5350 2726 50  0000 C CNN
F 2 "" H 5400 1600 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 5450 1950 50  0001 C CNN
	1    5350 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2250 4950 2250
Wire Wire Line
	6100 3350 5350 3350
$Comp
L Device:C C1
U 1 1 5DB0EE88
P 6200 2650
F 0 "C1" H 6315 2696 50  0000 L CNN
F 1 "10nF" H 6315 2605 50  0000 L CNN
F 2 "" H 6238 2500 50  0001 C CNN
F 3 "~" H 6200 2650 50  0001 C CNN
	1    6200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2800 6200 3350
Wire Wire Line
	6200 3350 6100 3350
Connection ~ 6100 3350
Wire Wire Line
	6200 1750 6200 2500
Wire Wire Line
	8400 2250 8400 2350
Wire Wire Line
	5750 1950 6650 1950
Wire Wire Line
	5750 2050 7000 2050
Wire Wire Line
	5750 2150 7350 2150
Wire Wire Line
	5750 2250 7700 2250
$Comp
L Device:R R7
U 1 1 5DB2D978
P 7700 2650
F 0 "R7" H 7770 2696 50  0000 L CNN
F 1 "460" H 7770 2605 50  0000 L CNN
F 2 "" V 7630 2650 50  0001 C CNN
F 3 "~" H 7700 2650 50  0001 C CNN
	1    7700 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5DB2E5E9
P 7350 2650
F 0 "R6" H 7420 2696 50  0000 L CNN
F 1 "460" H 7420 2605 50  0000 L CNN
F 2 "" V 7280 2650 50  0001 C CNN
F 3 "~" H 7350 2650 50  0001 C CNN
	1    7350 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5DB2E922
P 7000 2650
F 0 "R5" H 7070 2696 50  0000 L CNN
F 1 "460" H 7070 2605 50  0000 L CNN
F 2 "" V 6930 2650 50  0001 C CNN
F 3 "~" H 7000 2650 50  0001 C CNN
	1    7000 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5DB2EC9A
P 6650 2650
F 0 "R4" H 6720 2696 50  0000 L CNN
F 1 "460" H 6720 2605 50  0000 L CNN
F 2 "" V 6580 2650 50  0001 C CNN
F 3 "~" H 6650 2650 50  0001 C CNN
	1    6650 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 2500 6650 1950
Connection ~ 6650 1950
Wire Wire Line
	6650 1950 8750 1950
Wire Wire Line
	7000 2500 7000 2050
Connection ~ 7000 2050
Wire Wire Line
	7000 2050 8400 2050
Wire Wire Line
	7350 2500 7350 2150
Connection ~ 7350 2150
Wire Wire Line
	7350 2150 8550 2150
Wire Wire Line
	7700 2500 7700 2250
Connection ~ 7700 2250
Wire Wire Line
	7700 2250 8400 2250
$Comp
L Device:LED D4
U 1 1 5DB34BAD
P 6650 3100
F 0 "D4" V 6689 2983 50  0000 R CNN
F 1 "LED" V 6598 2983 50  0000 R CNN
F 2 "" H 6650 3100 50  0001 C CNN
F 3 "~" H 6650 3100 50  0001 C CNN
	1    6650 3100
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 5DB35B02
P 7000 3100
F 0 "D5" V 7039 2983 50  0000 R CNN
F 1 "LED" V 6948 2983 50  0000 R CNN
F 2 "" H 7000 3100 50  0001 C CNN
F 3 "~" H 7000 3100 50  0001 C CNN
	1    7000 3100
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D6
U 1 1 5DB35DFA
P 7350 3100
F 0 "D6" V 7389 2983 50  0000 R CNN
F 1 "LED" V 7298 2983 50  0000 R CNN
F 2 "" H 7350 3100 50  0001 C CNN
F 3 "~" H 7350 3100 50  0001 C CNN
	1    7350 3100
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D7
U 1 1 5DB36389
P 7700 3100
F 0 "D7" V 7739 2983 50  0000 R CNN
F 1 "LED" V 7648 2983 50  0000 R CNN
F 2 "" H 7700 3100 50  0001 C CNN
F 3 "~" H 7700 3100 50  0001 C CNN
	1    7700 3100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6650 2800 6650 2950
Wire Wire Line
	7000 2800 7000 2950
Wire Wire Line
	7350 2800 7350 2950
Wire Wire Line
	7700 2800 7700 2950
Wire Wire Line
	7700 3350 7700 3250
Connection ~ 6200 3350
Wire Wire Line
	7350 3250 7350 3350
Connection ~ 7350 3350
Wire Wire Line
	7350 3350 7700 3350
Wire Wire Line
	7000 3250 7000 3350
Connection ~ 7000 3350
Wire Wire Line
	7000 3350 7350 3350
Wire Wire Line
	6650 3250 6650 3350
Connection ~ 6650 3350
Wire Wire Line
	6650 3350 7000 3350
Text Notes 4950 5850 0    50   ~ 0
Arduino powered via USB
Wire Wire Line
	6200 3350 6400 3350
Wire Wire Line
	3550 4000 3550 4400
Connection ~ 3550 4000
Wire Wire Line
	4250 4000 4900 4000
Wire Wire Line
	4550 4300 4900 4300
Wire Wire Line
	4550 4400 4900 4400
Wire Wire Line
	4550 4500 4900 4500
Wire Wire Line
	4550 4600 4900 4600
Wire Wire Line
	4700 1950 4950 1950
Wire Wire Line
	4700 2050 4950 2050
Wire Wire Line
	4700 2150 4950 2150
Text Label 4550 4600 0    50   ~ 0
M4
Text Label 4550 4300 0    50   ~ 0
M1
Text Label 4550 4400 0    50   ~ 0
M2
Text Label 4550 4500 0    50   ~ 0
M3
Text Label 4700 1950 0    50   ~ 0
M1
Text Label 4700 2050 0    50   ~ 0
M2
Text Label 4700 2150 0    50   ~ 0
M3
Text Label 4700 2250 0    50   ~ 0
M4
Wire Wire Line
	4400 5700 4600 5700
Wire Wire Line
	4600 5700 4600 5000
Wire Wire Line
	4600 5000 4900 5000
Wire Wire Line
	4400 5000 4500 5000
Wire Wire Line
	4500 5000 4500 4800
Wire Wire Line
	4500 4800 4900 4800
Wire Wire Line
	4400 5350 4550 5350
Wire Wire Line
	4550 5350 4550 4900
Wire Wire Line
	4550 4900 4900 4900
Wire Wire Line
	4850 6000 4850 5500
Wire Wire Line
	4850 5500 5400 5500
$Comp
L power:GND #PWR01
U 1 1 5DAEDE1E
P 6400 3550
F 0 "#PWR01" H 6400 3300 50  0001 C CNN
F 1 "GND" H 6405 3377 50  0000 C CNN
F 2 "" H 6400 3550 50  0001 C CNN
F 3 "" H 6400 3550 50  0001 C CNN
	1    6400 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3550 6400 3350
Connection ~ 6400 3350
Wire Wire Line
	6400 3350 6650 3350
Wire Wire Line
	3550 5700 3550 6000
Wire Wire Line
	4100 4300 4250 4300
Wire Wire Line
	4250 4300 4250 4100
Wire Wire Line
	4250 4100 4900 4100
Wire Wire Line
	4900 4200 4350 4200
Wire Wire Line
	4350 4200 4350 4500
Wire Wire Line
	4350 4500 4100 4500
Wire Wire Line
	4750 3500 5300 3500
$Comp
L Device:Battery BT1
U 1 1 5DB5A285
P 3550 2800
F 0 "BT1" H 3658 2846 50  0000 L CNN
F 1 "12V" H 3658 2755 50  0000 L CNN
F 2 "" V 3550 2860 50  0001 C CNN
F 3 "~" V 3550 2860 50  0001 C CNN
	1    3550 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 6000 4850 6000
Wire Wire Line
	3550 3000 3550 3050
Connection ~ 3550 3500
Wire Wire Line
	3550 2600 4750 2600
Wire Wire Line
	4750 2600 4750 3500
Wire Wire Line
	5350 2750 5350 3050
Wire Wire Line
	3550 3050 5350 3050
Connection ~ 3550 3050
Wire Wire Line
	3550 3050 3550 3500
Connection ~ 5350 3050
Wire Wire Line
	5350 3050 5350 3350
$Comp
L Motor:Stepper_Motor_bipolar M1
U 1 1 5DB847A9
P 8850 2250
F 0 "M1" H 9038 2374 50  0000 L CNN
F 1 "Stepper_Motor_bipolar" H 9038 2283 50  0000 L CNN
F 2 "" H 8860 2240 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 8860 2240 50  0001 C CNN
	1    8850 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 2050 8400 1850
Wire Wire Line
	8400 1850 8950 1850
Wire Wire Line
	8950 1850 8950 1950
Wire Wire Line
	8400 2350 8550 2350
Wire Wire Line
	5750 1750 6200 1750
Wire Wire Line
	3550 2600 3550 1300
Wire Wire Line
	3550 1300 6200 1300
Wire Wire Line
	6200 1300 6200 1750
Connection ~ 3550 2600
Connection ~ 6200 1750
$EndSCHEMATC
