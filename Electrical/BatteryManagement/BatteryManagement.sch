EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Battery Management"
Date "2020-02-25"
Rev "v1.0.0"
Comp ""
Comment1 "Author: Jack Amsbry"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5E5560AB
P 10200 3000
F 0 "J?" H 10200 3100 50  0000 C CNN
F 1 "Battery Connection" H 10200 2800 50  0000 C CNN
F 2 "" H 10200 3000 50  0001 C CNN
F 3 "~" H 10200 3000 50  0001 C CNN
	1    10200 3000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J?
U 1 1 5E558B92
P 10000 2550
F 0 "J?" H 10000 2750 50  0000 C CNN
F 1 "JST-XH Connector" H 10000 2350 50  0000 C CNN
F 2 "" H 10000 2550 50  0001 C CNN
F 3 "~" H 10000 2550 50  0001 C CNN
	1    10000 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 5E55AE8D
P 6550 1600
F 0 "D?" H 6550 1700 50  0000 C CNN
F 1 "LED" H 6550 1500 50  0000 C CNN
F 2 "" H 6550 1600 50  0001 C CNN
F 3 "~" H 6550 1600 50  0001 C CNN
	1    6550 1600
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 5E55B477
P 6550 1950
F 0 "D?" H 6550 2050 50  0000 C CNN
F 1 "LED" H 6550 1850 50  0000 C CNN
F 2 "" H 6550 1950 50  0001 C CNN
F 3 "~" H 6550 1950 50  0001 C CNN
	1    6550 1950
	-1   0    0    1   
$EndComp
$Comp
L Battery_Management:BQ24005 U?
U 1 1 5E55BBFD
P 8250 1900
F 0 "U?" H 7900 2550 50  0000 C CNN
F 1 "BQ24005" H 8450 2550 50  0000 C CNN
F 2 "Package_SO:HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm_Mask2.4x3.7mm" H 8250 3050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/bq24006.pdf" H 8250 3700 50  0001 C CNN
	1    8250 1900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
