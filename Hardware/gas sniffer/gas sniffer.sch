EESchema Schematic File Version 4
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
L SamacSys_Parts:LM317MBTG IC1
U 1 1 62D0FAD7
P 4350 2850
F 0 "IC1" V 4738 2522 50  0000 R CNN
F 1 "LM317MBTG" V 4647 2522 50  0000 R CNN
F 2 "Mouser Import:TO254P482X997X2018-3P" H 5100 2950 50  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/LM317M-D.PDF" H 5100 2850 50  0001 L CNN
F 4 "" H 5100 2750 50  0001 L CNN "Description"
F 5 "4.82" H 5100 2650 50  0001 L CNN "Height"
F 6 "863-LM317MBTG" H 5100 2550 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/onsemi/LM317MBTG?qs=2OtswVQKCOHWWQM%252BNYhxOw%3D%3D" H 5100 2450 50  0001 L CNN "Mouser Price/Stock"
F 8 "onsemi" H 5100 2350 50  0001 L CNN "Manufacturer_Name"
F 9 "LM317MBTG" H 5100 2250 50  0001 L CNN "Manufacturer_Part_Number"
	1    4350 2850
	0    1    -1   0   
$EndComp
$Comp
L SamacSys_Parts:PT15NV15-102A1010-S VR1
U 1 1 62D107E4
P 5350 3000
F 0 "VR1" V 6096 2772 50  0000 R CNN
F 1 "PT15NV15-102A1010-S" V 6005 2772 50  0000 R CNN
F 2 "Mouser Import:PT15NV15102A1010S" H 6600 3100 50  0001 L CNN
F 3 "https://www.mouser.de/datasheet/2/18/1/14_PT15v03-1915311.pdf" H 6600 3000 50  0001 L CNN
F 4 "Trimmer Resistors - Through Hole 15mm control/sensor trimmr potentiometer" H 6600 2900 50  0001 L CNN "Description"
F 5 "7.2" H 6600 2800 50  0001 L CNN "Height"
F 6 "531-PT15NV15102A101S" H 6600 2700 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Amphenol-Piher/PT15NV15-102A1010-S?qs=DPoM0jnrROW7FjN7ghMpfA%3D%3D" H 6600 2600 50  0001 L CNN "Mouser Price/Stock"
F 8 "PIHER" H 6600 2500 50  0001 L CNN "Manufacturer_Name"
F 9 "PT15NV15-102A1010-S" H 6600 2400 50  0001 L CNN "Manufacturer_Part_Number"
	1    5350 3000
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 62D116D3
P 4350 4500
F 0 "J1" V 4222 4680 50  0000 L CNN
F 1 "Conn_01x04" V 4313 4680 50  0000 L CNN
F 2 "Mouser Import:1963447" H 4350 4500 50  0001 C CNN
F 3 "~" H 4350 4500 50  0001 C CNN
	1    4350 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 62D12E26
P 5000 3000
F 0 "R2" H 5070 3046 50  0000 L CNN
F 1 "270" H 5070 2955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4930 3000 50  0001 C CNN
F 3 "~" H 5000 3000 50  0001 C CNN
	1    5000 3000
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C1
U 1 1 62D13353
P 4300 3450
F 0 "C1" H 4415 3496 50  0000 L CNN
F 1 "0.1 uF" H 4415 3405 50  0000 L CNN
F 2 "Capacitor_THT:CP_Axial_L10.0mm_D4.5mm_P15.00mm_Horizontal" H 4338 3300 50  0001 C CNN
F 3 "https://www.mouser.co.uk/datasheet/2/212/KEM_C1041_AXIMAX_X7R-1140508.pdf" H 4300 3450 50  0001 C CNN
	1    4300 3450
	0    -1   -1   0   
$EndComp
$Comp
L SamacSys_Parts:SGAS701 U1
U 1 1 62D13689
P 1900 3000
F 0 "U1" H 3544 3046 50  0000 L CNN
F 1 "SGAS701" H 3544 2955 50  0000 L CNN
F 2 "Mouser Import:SGAS701" H 3350 3600 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/SGAS701.pdf" H 3350 3500 50  0001 L CNN
F 4 "TRANSISTOR (T0-39)" H 3350 3400 50  0001 L CNN "Description"
F 5 "8.2296" H 3350 3300 50  0001 L CNN "Height"
F 6 "972-SGAS701" H 3350 3200 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=972-SGAS701" H 3350 3100 50  0001 L CNN "Mouser Price/Stock"
F 8 "IDT" H 3350 3000 50  0001 L CNN "Manufacturer_Name"
F 9 "SGAS701" H 3350 2900 50  0001 L CNN "Manufacturer_Part_Number"
	1    1900 3000
	1    0    0    -1  
$EndComp
Connection ~ 4150 3450
Wire Wire Line
	4150 2850 4150 3450
$Comp
L Device:C C2
U 1 1 62D1A319
P 4800 3150
F 0 "C2" H 4915 3196 50  0000 L CNN
F 1 "10 uF" H 4915 3105 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm" H 4838 3000 50  0001 C CNN
F 3 "https://www.mouser.co.uk/datasheet/2/445/860010472002-1725261.pdf" H 4800 3150 50  0001 C CNN
	1    4800 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 3300 4800 3450
Wire Wire Line
	5450 3000 5450 3450
Wire Wire Line
	5450 3450 4800 3450
Connection ~ 4800 3450
Wire Wire Line
	4800 3450 4800 3600
Wire Wire Line
	4250 3600 4250 4300
$Comp
L power:GND #PWR0101
U 1 1 62D1A87E
P 4800 3600
F 0 "#PWR0101" H 4800 3350 50  0001 C CNN
F 1 "GND" H 4805 3427 50  0000 C CNN
F 2 "" H 4800 3600 50  0001 C CNN
F 3 "" H 4800 3600 50  0001 C CNN
	1    4800 3600
	1    0    0    -1  
$EndComp
Connection ~ 4800 3600
$Comp
L power:VDD #PWR0102
U 1 1 62D1AFA8
P 4150 4200
F 0 "#PWR0102" H 4150 4050 50  0001 C CNN
F 1 "VDD" V 4168 4327 50  0000 L CNN
F 2 "" H 4150 4200 50  0001 C CNN
F 3 "" H 4150 4200 50  0001 C CNN
	1    4150 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4250 3000 4250 2850
Wire Wire Line
	5200 3000 5200 2850
Wire Wire Line
	5150 3000 5200 3000
Connection ~ 5200 3000
Wire Wire Line
	5200 3000 5350 3000
Connection ~ 4800 3000
Wire Wire Line
	4800 3000 4850 3000
Wire Wire Line
	4250 3600 4800 3600
Wire Wire Line
	4450 3450 4800 3450
Wire Wire Line
	4250 3000 4800 3000
Wire Wire Line
	4350 2850 5200 2850
Wire Wire Line
	4250 3000 3500 3000
Connection ~ 4250 3000
$Comp
L power:GND #PWR0103
U 1 1 62D1F4CD
P 1900 3000
F 0 "#PWR0103" H 1900 2750 50  0001 C CNN
F 1 "GND" H 1905 2827 50  0000 C CNN
F 2 "" H 1900 3000 50  0001 C CNN
F 3 "" H 1900 3000 50  0001 C CNN
	1    1900 3000
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM2931AZ-5.0_TO92 U2
U 1 1 62D1FEA0
P 3350 4000
F 0 "U2" H 3350 4242 50  0000 C CNN
F 1 "LM2931AZ-5.0_TO92" H 3350 4151 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3350 4225 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2931-n.pdf" H 3350 3950 50  0001 C CNN
	1    3350 4000
	1    0    0    -1  
$EndComp
Connection ~ 4150 4200
Wire Wire Line
	4150 4200 4150 4300
Wire Wire Line
	4150 3450 4150 4000
Wire Wire Line
	3650 4000 4150 4000
Connection ~ 4150 4000
Wire Wire Line
	4150 4000 4150 4200
Wire Wire Line
	3050 4000 2800 4000
Wire Wire Line
	2700 4000 2700 3800
$Comp
L power:GND #PWR0104
U 1 1 62D2322D
P 3350 4300
F 0 "#PWR0104" H 3350 4050 50  0001 C CNN
F 1 "GND" H 3355 4127 50  0000 C CNN
F 2 "" H 3350 4300 50  0001 C CNN
F 3 "" H 3350 4300 50  0001 C CNN
	1    3350 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 62D23AFE
P 2700 1900
F 0 "R1" H 2770 1946 50  0000 L CNN
F 1 "649k" H 2770 1855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2630 1900 50  0001 C CNN
F 3 "https://www.mouser.co.uk/datasheet/2/418/4/NG_DS_1773265_A-721971.pdf" H 2700 1900 50  0001 C CNN
	1    2700 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 62D24633
P 2700 1750
F 0 "#PWR0105" H 2700 1500 50  0001 C CNN
F 1 "GND" H 2705 1577 50  0000 C CNN
F 2 "" H 2700 1750 50  0001 C CNN
F 3 "" H 2700 1750 50  0001 C CNN
	1    2700 1750
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 2200 2700 2050
Text GLabel 2700 4000 0    50   Input ~ 0
V_reg
Text GLabel 2700 2150 0    50   Input ~ 0
V_out
Text GLabel 4450 4300 1    50   Input ~ 0
V_out
Text GLabel 4350 4300 1    50   Input ~ 0
V_reg
$Comp
L Device:C C3
U 1 1 62D25EAB
P 2800 4150
F 0 "C3" H 2915 4196 50  0000 L CNN
F 1 "100 uF" H 2915 4105 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm" H 2838 4000 50  0001 C CNN
F 3 "https://www.mouser.co.uk/datasheet/2/231/161587801891-2256037.pdf" H 2800 4150 50  0001 C CNN
	1    2800 4150
	-1   0    0    1   
$EndComp
Connection ~ 3350 4300
Connection ~ 2800 4000
Wire Wire Line
	2800 4000 2700 4000
Wire Wire Line
	2800 4300 3350 4300
$EndSCHEMATC
