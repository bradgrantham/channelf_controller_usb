EESchema Schematic File Version 4
LIBS:channelf_minim4-cache
EELAYER 30 0
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
L minim4-d40:MiniM4-D40 U1
U 1 1 5DA7DF65
P 3600 3500
F 0 "U1" H 3600 4987 60  0000 C CNN
F 1 "MiniM4-D40" H 3600 4881 60  0000 C CNN
F 2 "Housings_DIP:DIP-40_W15.24mm_Socket" H 3450 2000 60  0001 C CNN
F 3 "" H 3450 2000 60  0000 C CNN
	1    3600 3500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x18_Male J2
U 1 1 5DA806E7
P 5975 3375
F 0 "J2" H 6083 4356 50  0000 C CNN
F 1 "Conn_01x18_Male" H 6083 4265 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x18_Pitch2.54mm" H 5975 3375 50  0001 C CNN
F 3 "~" H 5975 3375 50  0001 C CNN
	1    5975 3375
	-1   0    0    -1  
$EndComp
Text GLabel 2950 4200 0    50   Input ~ 0
C1DOWN
Text GLabel 2950 3700 0    50   Input ~ 0
C1UP
Text GLabel 2950 3600 0    50   Input ~ 0
C1CW
Text GLabel 2950 3300 0    50   Input ~ 0
C1CCW
Text GLabel 4250 4000 2    50   Input ~ 0
C2CCW
Text GLabel 4250 4100 2    50   Input ~ 0
C2CW
Text GLabel 4250 4300 2    50   Input ~ 0
C2DOWN
Text GLabel 4250 4200 2    50   Input ~ 0
C2UP
$Comp
L power:GND #PWR0101
U 1 1 5DAA84AA
P 2625 4950
F 0 "#PWR0101" H 2625 4700 50  0001 C CNN
F 1 "GND" H 2630 4777 50  0000 C CNN
F 2 "" H 2625 4950 50  0001 C CNN
F 3 "" H 2625 4950 50  0001 C CNN
	1    2625 4950
	1    0    0    -1  
$EndComp
Text GLabel 2950 2900 0    50   Input ~ 0
C1WEST
Text GLabel 4250 3200 2    50   Input ~ 0
C2SOUTH
Text GLabel 2950 3000 0    50   Input ~ 0
C1SOUTH
Text GLabel 4250 3300 2    50   Input ~ 0
C2NORTH
Text GLabel 2950 3100 0    50   Input ~ 0
C1NORTH
Text GLabel 2950 2800 0    50   Input ~ 0
C1EAST
Text GLabel 4250 2800 2    50   Input ~ 0
C2EAST
Text GLabel 4250 2900 2    50   Input ~ 0
C2WEST
$Comp
L power:GND #PWR0102
U 1 1 5DAAA6D9
P 1900 2575
F 0 "#PWR0102" H 1900 2325 50  0001 C CNN
F 1 "GND" H 1905 2402 50  0000 C CNN
F 2 "" H 1900 2575 50  0001 C CNN
F 3 "" H 1900 2575 50  0001 C CNN
	1    1900 2575
	1    0    0    -1  
$EndComp
Text GLabel 5775 2575 0    50   Output ~ 0
C2DOWN
Text GLabel 5775 2775 0    50   Output ~ 0
C2UP
Text GLabel 5775 2975 0    50   Output ~ 0
C2CW
Text GLabel 5775 3175 0    50   Output ~ 0
C2CCW
Text GLabel 5775 3375 0    50   Output ~ 0
C2NORTH
Text GLabel 5775 3575 0    50   Output ~ 0
C2SOUTH
Text GLabel 5775 3775 0    50   Output ~ 0
C2WEST
Text GLabel 5775 3975 0    50   Output ~ 0
C2EAST
Wire Wire Line
	5775 4275 5150 4275
NoConn ~ 4250 3000
NoConn ~ 4250 3100
NoConn ~ 4250 3400
NoConn ~ 4250 3500
NoConn ~ 4250 3600
NoConn ~ 4250 3700
NoConn ~ 4250 3800
NoConn ~ 4250 3900
NoConn ~ 2950 2300
NoConn ~ 2950 2400
NoConn ~ 2950 2600
NoConn ~ 2950 3200
NoConn ~ 2950 3400
NoConn ~ 2950 3500
NoConn ~ 2950 3800
NoConn ~ 2950 3900
NoConn ~ 2950 4000
NoConn ~ 2950 4100
NoConn ~ 2950 4300
NoConn ~ 2950 4400
NoConn ~ 2950 4700
NoConn ~ 5775 4175
NoConn ~ 5775 4075
NoConn ~ 5775 3875
NoConn ~ 5775 3675
NoConn ~ 5775 3475
NoConn ~ 5775 3275
NoConn ~ 5775 3075
NoConn ~ 5775 2875
NoConn ~ 5775 2675
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DA8B359
P 1750 1225
F 0 "#FLG0101" H 1750 1300 50  0001 C CNN
F 1 "PWR_FLAG" H 1750 1398 50  0000 C CNN
F 2 "" H 1750 1225 50  0001 C CNN
F 3 "~" H 1750 1225 50  0001 C CNN
	1    1750 1225
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5DA8B75B
P 1750 1375
F 0 "#PWR0104" H 1750 1125 50  0001 C CNN
F 1 "GND" H 1755 1202 50  0000 C CNN
F 2 "" H 1750 1375 50  0001 C CNN
F 3 "" H 1750 1375 50  0001 C CNN
	1    1750 1375
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1375 1750 1225
NoConn ~ 1400 2575
NoConn ~ 1400 2775
NoConn ~ 1400 2975
NoConn ~ 1400 3175
NoConn ~ 1400 3375
NoConn ~ 1400 3575
NoConn ~ 1400 3775
NoConn ~ 1400 3975
NoConn ~ 1400 4175
NoConn ~ 1400 4275
Text GLabel 1400 2875 2    50   Output ~ 0
C1EAST
Text GLabel 1400 3075 2    50   Output ~ 0
C1WEST
Text GLabel 1400 3275 2    50   Output ~ 0
C1SOUTH
Text GLabel 1400 3475 2    50   Output ~ 0
C1NORTH
Text GLabel 1400 3675 2    50   Output ~ 0
C1CCW
Text GLabel 1400 3875 2    50   Output ~ 0
C1CW
Text GLabel 1400 4075 2    50   Output ~ 0
C1UP
Text GLabel 1400 4275 2    50   Output ~ 0
C1DOWN
$Comp
L Connector:Conn_01x18_Male J1
U 1 1 5DA8360D
P 1200 3475
F 0 "J1" H 1172 3449 50  0000 R CNN
F 1 "Conn_01x18_Male" H 1172 3358 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x18_Pitch2.54mm" H 1200 3475 50  0001 C CNN
F 3 "~" H 1200 3475 50  0001 C CNN
	1    1200 3475
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5DAAB21F
P 5150 4675
F 0 "#PWR0103" H 5150 4425 50  0001 C CNN
F 1 "GND" H 5155 4502 50  0000 C CNN
F 2 "" H 5150 4675 50  0001 C CNN
F 3 "" H 5150 4675 50  0001 C CNN
	1    5150 4675
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5150 4275 5150 4675
Wire Wire Line
	2625 4600 2625 4950
Wire Wire Line
	2625 4600 2950 4600
Wire Wire Line
	1400 2575 1900 2575
NoConn ~ 4250 2700
NoConn ~ 4250 2600
$EndSCHEMATC
