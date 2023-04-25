/*                                                     https://oshwlab.com/ratti3
  _|_|_|                _|      _|      _|  _|_|_|     https://youtube.com/@Ratti3
  _|    _|    _|_|_|  _|_|_|_|_|_|_|_|            _|   https://projecthub.arduino.cc/Ratti3
  _|_|_|    _|    _|    _|      _|      _|    _|_|     https://ratti3.blogspot.com
  _|    _|  _|    _|    _|      _|      _|        _|   https://hackaday.io/Ratti3
  _|    _|    _|_|_|      _|_|    _|_|  _|  _|_|_|     https://www.hackster.io/Ratti3
.                                                      https://github.com/Ratti3

Code forked from https://github.com/spencer1979/bq2589x

This file is part of https://github.com/Ratti3/BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED

BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED is free software: you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef _BQ2589X_REG_H_
#define _BQ2589X_REG_H_

/* SUGGESTED SETTINGS - RUN IN THIS ORDER
	0x14: 10000110 Full BQ reset
	0x07: 1xxxxxxx Enable Charging Termination [default]
		  x1xxxxxx Disable STAT PIN [x0xxxxxx default]
		  xx00xxxx Disable I2C Watchdog Timer [xx01xxxx default]
		  xxxxx01x Set Charge Timer to 8 hrs [xxxxx10x 12 hrs default]
	0x00: x0xxxxxx Disable ILIM PIN [x1xxxxxx default]
		  xx111111 Set Input Current Limit to 3250mA [xx001010 default]
	0x02: 11xxxxxx Enable ADC Conversion and Set Rate [00xxxxxx default]
	0x03: xx1xxxxx Enable OTG [xx0xxxxx default]
	0x04: x0101111 Set Fast Charge Current Limit to 3008mA [x0100000 2048mA default]
	0x05: 0001xxxx Set Precharge Current Limit to 128mA [default]
		  xxxx0011 Set Termination Current Limit to 256mA [default]
	0x06: 01011100 Set Charge Voltage Limit to 4192mV (010111xx) [010111xx 4208v default]
	0x0A: 0111xxxx Set Boost Mode Voltage Regulation to 4.998 (0111xxxx) [default]
		  xxxxx110 Set Boost Mode Current Limit to 2.15A xxxxx110 [xxxxx011 1.4A default]
*/

/* Register 00 ************************************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 1	 0	 0	 0 : Chip Default
 0	 1	 1	 1	 1	 1	 1	 1 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset					Description
 7	EN_HIZ			R/W		REG_RST/Watchdog		Enable HIZ Mode
													 0 - Disable (default)
								 					 1 - Enable
 6	EN_ILIM			R/W		REG_RST/Watchdog		Enable ILIM Pin
													 0 - Disable
											 		 1 - Enable (default: Enable ILIM pin (1))
 5	IINLIM[5]		R/W		REG_RST					1600mA  Input Current Limit
 4	IINLIM[4]		R/W		REG_RST					800mA    Offset: 100mA
 3	IINLIM[3]		R/W		REG_RST					400mA    Range: 100mA (000000) - 3.25A (111111)
 2	IINLIM[2]		R/W		REG_RST					200mA    Default:0001000 (500mA)
 1	IINLIM[1]		R/W		REG_RST					100mA    (Actual input current limit is the lower of I2C or ILIM pin)
 0	IINLIM[0]		R/W		REG_RST					50mA     IINLIM bits are changed automaticallly after input source type detection is completed
															 PSEL = Hi (USB500) = 500mA
															 PSEL = Lo = 3.25A
*/
#define BQ2589X_REG_00         0x00

#define BQ2589X_ENHIZ_MASK     0x80 // BIT 7   10000000
#define BQ2589X_ENHIZ_SHIFT    7
#define BQ2589X_HIZ_ENABLE     1
#define BQ2589X_HIZ_DISABLE    0    // Default

#define BQ2589X_ENILIM_MASK    0x40 // BIT 6   01000000
#define BQ2589X_ENILIM_SHIFT   6
#define BQ2589X_ENILIM_ENABLE  1    // Default
#define BQ2589X_ENILIM_DISABLE 0
#define BQ2589X_IINLIM_MASK    0x3F // BIT 0-5 00111111
#define BQ2589X_IINLIM_SHIFT   0
#define BQ2589X_IINLIM_BASE    100  // 100mA
#define BQ2589X_IINLIM_LSB     50   // 50mA

/* Register 01 *****************************************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 1	 1	 0 : Chip Default
 0	 0	 0	 0	 0	 1	 1	 0 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	BHOT[1]			R/W		REG_RST/Watchdog	Boost Mode Hot Temperature Monitor Threshold
 6	BHOT[0]			R/W		REG_RST/Watchdog	 00 - VBHOT1 Threshold (34.75%) (default)
												 01 - VBHOT0 Threshold (Typ. 37.75%)
												 10 - VBHOT2 Threshold (Typ. 31.25%)
												 11 - Disable boost mode thermal protection
 5	BCOLD			R/W		REG_RST/Watchdog	Boost Mode Cold Temperature Monitor Threshold
												 0 - VBCOLD0 Threshold (Typ. 77%) (default)
												 1 - VBCOLD1 Threshold (Typ. 80%)
 4	VINDPM_OS[4]	R/W		REG_RST				1600mV	Input Voltage Limit Offset
 3	VINDPM_OS[3]	R/W		REG_RST				800mV    Default: 600mV (00110)
 2	VINDPM_OS[2]	R/W		REG_RST				400mV    Range: 0mV - 3100mV
 1	VINDPM_OS[1]	R/W		REG_RST				200mV    Minimum VINDPM threshold is clamped at 3.9V. Maximum VINDPM threshold is clamped at 15.3V
 0	VINDPM_OS[0]	R/W		REG_RST				100mV    When VBUS at noLoad is ≤ 6V, the VINDPM_OS is used to calculate VINDPM threhold
														 When VBUS at noLoad is > 6V, the VINDPM_OS multiple by 2 is used to calculate VINDPM threshold
*/
#define BQ2589X_REG_01         0x01

#define BQ2589X_BHOT_MASK      0xC0 // BIT 6-7 11000000
#define BQ2589X_BHOT_SHIFT     6
#define BQ2589X_BCOLD_MASK     0x20 // BIT 5   00100000
#define BQ2589X_BCOLD_SHIFT    5

#define BQ2589X_VINDPMOS_MASK  0x1F // BIT 0-4 00011111
#define BQ2589X_VINDPMOS_SHIFT 0
#define BQ2589X_VINDPMOS_BASE  0
#define BQ2589X_VINDPMOS_LSB   100  // 100mV

/* Register 0x02 *****************************************************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 1	 0	 0	 0	 1 : Chip Default
 1	 1	 0	 1	 0	 0	 0	 1 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	CONV_START		R/W		REG_RST/Watchdog	ADC Conversion Start Control
												 0 - ADC conversion not active (default).
												 1 - Start ADC Conversion
												 This bit is read-only when CONV_RATE = 1. The bit stays high during ADC conversion and during input source detection.
 6	CONV_RATE		R/W		REG_RST/Watchdog	ADC Conversion Rate Selection
												 0 - One shot ADC conversion (default)
												 1 - Start 1s Continuous Conversion
 5	BOOST_FREQ		R/W		REG_RST/Watchdog	Boost Mode Frequency Selection
												 0 - 1.5MHz (default)
												 1 - 500KHz
												 Note: Write to this bit is ignored when OTG_CONFIG is enabled.
 4	ICO_EN			R/W		REG_RST				Input Current Optimizer (ICO) Enable
												 0 - Disable ICO Algorithm
												 1 - Enable ICO Algorithm (default)
 3	Reserved		R/W		REG_RST				Reserved (default = 0)
 2	Reserved		R/W		REG_RST				Reserved (default = 0)
 1	FORCE_DPDM		R/W		REG_RST/Watchdog	Force Input Detection
												 0 - Not in PSEL detection (default)
												 1 - Force PSEL detection
 0	AUTO_DPDM_EN	R/W		REG_RST				Automatic Input Detection Enable
												 0 - Disable PSEL detection when VBUS is plugged-in
												 1 - Enable PSEL detection when VBUS is plugged-in (default)
*/
#define BQ2589X_REG_02               0x02

#define BQ2589X_CONV_START_MASK      0x80 // BIT 7   10000000
#define BQ2589X_CONV_START_SHIFT     7
#define BQ2589X_CONV_START           1
#define BQ2589X_CONV_RATE_MASK       0x40 // BIT 6   01000000
#define BQ2589X_CONV_RATE_SHIFT      6
#define BQ2589X_ADC_CONTINUE_ENABLE  1
#define BQ2589X_ADC_CONTINUE_DISABLE 0

#define BQ2589X_BOOST_FREQ_MASK      0x20 // BIT 5   00100000
#define BQ2589X_BOOST_FREQ_SHIFT     5
#define BQ2589X_BOOST_FREQ_1500K     0    // Default
#define BQ2589X_BOOST_FREQ_500K      1

#define BQ2589X_ICOEN_MASK           0x10 // BIT 4   00010000
#define BQ2589X_ICOEN_SHIFT          4
#define BQ2589X_ICO_ENABLE           1    // Default
#define BQ2589X_ICO_DISABLE          0

#define BQ2589X_HVDCPEN_MASK         0x08 // N/A for BQ25896
#define BQ2589X_HVDCPEN_SHIFT        3
#define BQ2589X_HVDCP_ENABLE         1
#define BQ2589X_HVDCP_DISABLE        0
#define BQ2589X_MAXCEN_MASK          0x04
#define BQ2589X_MAXCEN_SHIFT         2
#define BQ2589X_MAXC_ENABLE          1
#define BQ2589X_MAXC_DISABLE         0

#define BQ2589X_FORCE_DPDM_MASK      0x02 // BIT 1   00000010
#define BQ2589X_FORCE_DPDM_SHIFT     1
#define BQ2589X_FORCE_DPDM           1
#define BQ2589X_AUTO_DPDM_EN_MASK    0x01 // BIT 0   00000001
#define BQ2589X_AUTO_DPDM_EN_SHIFT   0
#define BQ2589X_AUTO_DPDM_ENABLE     1    // Default
#define BQ2589X_AUTO_DPDM_DISABLE    0

/* Register 0x03 ***********************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 1	 1	 0	 1	 0 : Chip Default
 0	 0	 0	 1	 1	 0	 1	 0 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	BAT_LOADEN		R/W		REG_RST/Watchdog	Battery Load (IBATLOAD) Enable
												 0 - Disabled (default)
												 1 - Enabled
 6	WD_RST			R/W		REG_RST/Watchdog	I2C Watchdog Timer Reset
												 0 - Normal (default)
												 1 - Reset (Back to 0 after timer reset)
 5	OTG_CONFIG		R/W		REG_RST/Watchdog	Boost (OTG) Mode Configuration
												 0 - OTG Disable (default)
												 1 - OTG Enable
 4	CHG_CONFIG		R/W		REG_RST/Watchdog	Charge Enable Configuration
												 0 - Charge Disable
												 1 - Charge Enable (default)
 3	SYS_MIN[2]		R/W		REG_RST				0.4V  Minimum System Voltage Limit
 2	SYS_MIN[1]		R/W		REG_RST				0.2V   Offset: 3.0V
 1	SYS_MIN[0]		R/W		REG_RST				0.1V   Range 3.0V-3.7V
													   Default: 3.5V (101)
 0	MIN_VBAT_SEL	R/W		REG_RST/Watchdog	Minimum Battery Voltage (falling) to exit boost mode
												 0 - 2.9V (default)
												 1 - 2.5V
*/

#define BQ2589X_REG_03             0x03

#define BQ2589X_BAT_LOADEN_MASK    0x80 // BIT 7   10000000 ############# NEED TO CODE
#define BQ2589X_BAT_LOADEN_SHIFT   7
#define BQ2589X_BAT_LOADEN_ENABLE  1
#define BQ2589X_BAT_LOADEN_DISABLE 0    // Default

#define BQ2589X_WDT_RESET_MASK     0x40 // BIT 6   01000000
#define BQ2589X_WDT_RESET_SHIFT    6
#define BQ2589X_WDT_RESET          1

#define BQ2589X_OTG_CONFIG_MASK    0x20 // BIT 5   00100000
#define BQ2589X_OTG_CONFIG_SHIFT   5
#define BQ2589X_OTG_ENABLE         1
#define BQ2589X_OTG_DISABLE        0    // Default

#define BQ2589X_CHG_CONFIG_MASK    0x10 // BIT 4   00010000
#define BQ2589X_CHG_CONFIG_SHIFT   4
#define BQ2589X_CHG_ENABLE         1    // Default
#define BQ2589X_CHG_DISABLE        0

#define BQ2589X_SYS_MINV_MASK      0x0E // BIT 1-3 00001110
#define BQ2589X_SYS_MINV_SHIFT     1
#define BQ2589X_SYS_MINV_BASE      3000 // 3V Offset
#define BQ2589X_SYS_MINV_LSB       100  // 0.1V

/* Register 0x04 *********************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 1	 0	 0	 0	 0	 0 : Chip Default
 0	 0	 1	 0	 0	 0	 0	 0 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	EN_PUMPX		R/W		REG_RST/Watchdog	Current pulse control Enable
												 0 - Disable Current pulse control (default)
												 1 - Enable Current pulse control (PUMPX_UP and PUMPX_DN)
 6	ICHG[6]			R/W		REG_RST/Watchdog	4096mA  Fast Charge Current Limit
 5	ICHG[5]			R/W		REG_RST/Watchdog	2048mA   Offset: 0mA
 4	ICHG[4]			R/W		REG_RST/Watchdog	1024mA   Range: 0mA (0000000) – 3008mA (0101111)
 3	ICHG[3]			R/W		REG_RST/Watchdog	512mA    Default: 2048mA (0100000)
 2	ICHG[2]			R/W		REG_RST/Watchdog	256mA    Note: ICHG=000000 (0mA) disables charge
 1	ICHG[1]			R/W		REG_RST/Watchdog	128mA    ICHG > 0101111 (3008mA) is clamped to register value 0101111 (3008mA)
 0	ICHG[0]			R/W		REG_RST/Watchdog	64mA
*/
#define BQ2589X_REG_04         0x04

#define BQ2589X_EN_PUMPX_MASK  0x80 // BIT 7   10000000
#define BQ2589X_EN_PUMPX_SHIFT 7
#define BQ2589X_PUMPX_ENABLE   1
#define BQ2589X_PUMPX_DISABLE  0    // Default

#define BQ2589X_ICHG_MASK      0x7F // BIT 0-6 01111111
#define BQ2589X_ICHG_SHIFT     0
#define BQ2589X_ICHG_BASE      0
#define BQ2589X_ICHG_LSB       64   // 64mA

/* Register 0x05 ***************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 1	 0	 0	 1	 1 : Chip Default
 0	 0	 0	 1	 0	 0	 1	 1 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	IPRECHG[3]		R/W		REG_RST/Watchdog	512mA  Precharge Current Limit
 6	IPRECHG[2]		R/W		REG_RST/Watchdog	256mA   Offset: 64mA
 5	IPRECHG[1]		R/W		REG_RST/Watchdog	128mA   Range: 64mA – 1024mA
 4	IPRECHG[0]		R/W		REG_RST/Watchdog	64mA    Default: 128mA (0001)
 3	ITERM[3]		R/W		REG_RST/Watchdog	512mA  Termination Current Limit
 2	ITERM[2]		R/W		REG_RST/Watchdog	256mA   Offset: 64mA
 1	ITERM[1]		R/W		REG_RST/Watchdog	128mA   Range: 64mA – 1024mA
 0	ITERM[0]		R/W		REG_RST/Watchdog	64mA    Default: 256mA (0011)
*/
#define BQ2589X_REG_05        0x05

#define BQ2589X_IPRECHG_MASK  0xF0 // BIT 4-7 11110000
#define BQ2589X_IPRECHG_SHIFT 4
#define BQ2589X_IPRECHG_BASE  64   // 64mA Offset
#define BQ2589X_IPRECHG_LSB   64   // 64mA

#define BQ2589X_ITERM_MASK    0x0F // BIT 0-3 00001111
#define BQ2589X_ITERM_SHIFT   0
#define BQ2589X_ITERM_BASE    64   // 64mA Offset
#define BQ2589X_ITERM_LSB     64   // 64mA

/* Register 0x06 ****************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 1	 0	 1	 1	 1	 1	 0 : Chip Default
 0	 1	 0	 1	 1	 0	 1	 0 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	VREG[5]			R/W		REG_RST/Watchdog	512mV  Charge Voltage Limit
 6	VREG[4]			R/W		REG_RST/Watchdog	256mV   Offset: 3.840V
 5	VREG[3]			R/W		REG_RST/Watchdog	128mV   Range: 3.840V – 4.608V (110000)
 4	VREG[2]			R/W		REG_RST/Watchdog	64mV    Default: 4.208V (010111)
 3 	VREG[1]			R/W		REG_RST/Watchdog	32mV    Note: VREG > 110000 (4.608V) is clamped to register value 110000 (4.608V)
 2	VREG[0]			R/W		REG_RST/Watchdog	16mV
 1	BATLOWV			R/W		REG_RST/Watchdog	Battery Precharge to Fast Charge Threshold
												 0 – 2.8V
												 1 – 3.0V (default)
 0	VRECHG			R/W		REG_RST/Watchdog	Battery Recharge Threshold Offset (below Charge Voltage Limit)
												 0 – 100mV (VRECHG) below VREG (REG06[7:2]) (default)
												 1 – 200mV (VRECHG) below VREG (REG06[7:2])
*/
#define BQ2589X_REG_06         0x06

#define BQ2589X_VREG_MASK      0xFC // BIT 2-7 11111100
#define BQ2589X_VREG_SHIFT     2
#define BQ2589X_VREG_BASE      3840 // 3.84V Offset
#define BQ2589X_VREG_LSB       16   // 16mV

#define BQ2589X_BATLOWV_MASK   0x02 // BIT 1   00000010
#define BQ2589X_BATLOWV_SHIFT  1
#define BQ2589X_BATLOWV_2800MV 0    // 2.8V
#define BQ2589X_BATLOWV_3000MV 1    // 3V Default

#define BQ2589X_VRECHG_MASK    0x01 // BIT 0   00000001
#define BQ2589X_VRECHG_SHIFT   0
#define BQ2589X_VRECHG_100MV   0    // 100mV Default
#define BQ2589X_VRECHG_200MV   1    // 200mV

/* Register 0x07 **********************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 1	 0	 0	 1	 1	 1	 0	 1 : Chip Default
 1	 0	 0	 0	 1	 1	 0	 1 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	EN_TERM			R/W		REG_RST/Watchdog	Charging Termination Enable
												 0 – Disable
												 1 – Enable (default)
 6	STAT_DIS		R/W		REG_RST/Watchdog	STAT Pin Disable
												 0 – Enable STAT pin function (default)
												 1 – Disable STAT pin function
 5	WATCHDOG[1]		R/W		REG_RST/Watchdog	I2C Watchdog Timer Setting
 4	WATCHDOG[0]		R/W		REG_RST/Watchdog	 00 – Disable watchdog timer
												 01 – 40s (default)
												 10 – 80s
												 11 – 160s
 3	EN_TIMER		R/W		REG_RST/Watchdog	Charging Safety Timer Enable
												 0 – Disable
												 1 – Enable (default)
 2	CHG_TIMER[1]	R/W		REG_RST/Watchdog	Fast Charge Timer Setting
 1	CHG_TIMER[0]	R/W		REG_RST/Watchdog	 00 – 5 hrs
												 01 – 8 hrs
												 10 – 12 hrs (default)
												 11 – 20 hrs
 0	JEITA_ISET		R/W		REG_RST/Watchdog	JEITA Low Temperature Current Setting
		0C-10C									 0 – 50% of ICHG (REG04[6:0])
												 1 – 20% of ICHG (REG04[6:0]) (default)
*/
#define BQ2589X_REG_07            0x07

#define BQ2589X_EN_TERM_MASK      0x80 // BIT 7   10000000  ###### CODE
#define BQ2589X_EN_TERM_SHIFT     7
#define BQ2589X_TERM_ENABLE       1    // Default
#define BQ2589X_TERM_DISABLE      0

#define BQ2589X_STAT_DIS_MASK     0x40 // BIT 6   01000000
#define BQ2589X_STAT_DIS_SHIFT    6
#define BQ2589X_STAT_DIS_ENABLE   0    // Default
#define BQ2589X_STAT_DIS_DISABLE  1

#define BQ2589X_WDT_MASK          0x30 // BIT 4-5 00110000
#define BQ2589X_WDT_SHIFT         4
#define BQ2589X_WDT_DISABLE       0
#define BQ2589X_WDT_40S           1    // 40s Default
#define BQ2589X_WDT_80S           2
#define BQ2589X_WDT_160S          3
#define BQ2589X_WDT_BASE          0
#define BQ2589X_WDT_LSB           40

#define BQ2589X_EN_TIMER_MASK     0x08 // BIT 3   00001000
#define BQ2589X_EN_TIMER_SHIFT    3
#define BQ2589X_CHG_TIMER_ENABLE  1    // Default
#define BQ2589X_CHG_TIMER_DISABLE 0

#define BQ2589X_CHG_TIMER_MASK    0x06 // BIT 1-2 00000110
#define BQ2589X_CHG_TIMER_SHIFT   1
#define BQ2589X_CHG_TIMER_5HOURS  0
#define BQ2589X_CHG_TIMER_8HOURS  1
#define BQ2589X_CHG_TIMER_12HOURS 2    // 12H Default
#define BQ2589X_CHG_TIMER_20HOURS 3

#define BQ2589X_JEITA_ISET_MASK   0x01 // BIT 0   00000001
#define BQ2589X_JEITA_ISET_SHIFT  0
#define BQ2589X_JEITA_ISET_50PCT  0
#define BQ2589X_JEITA_ISET_20PCT  1    // 20% Default

/* Register 0x08 ***********************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 1	 1 : Chip Default
 0	 0	 0	 0	 0	 0	 1	 1 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	BAT_COMP[2]		R/W		REG_RST/Watchdog	80mΩ  IR Compensation Resistor Setting
 6	BAT_COMP[1]		R/W		REG_RST/Watchdog	40mΩ   Range: 0 – 140mΩ
 5	BAT_COMP[0]		R/W		REG_RST/Watchdog	20mΩ   Default: 0Ω (000) (i.e. Disable IRComp)
 4	VCLAMP[2]		R/W		REG_RST/Watchdog	128mV IR Compensation Voltage Clamp
 3	VCLAMP[1]		R/W		REG_RST/Watchdog	64mV   Offset: 0mV
 2	VCLAMP[0]		R/W		REG_RST/Watchdog	32mV   Range: 0-224mV
												       Default: 0mV (000)
 1	TREG[1]			R/W		REG_RST/Watchdog	Thermal Regulation Threshold above VREG (REG06[7:2])
 0	TREG[0]			R/W		REG_RST/Watchdog	 00 – 60°C
												 01 – 80°C
												 10 – 100°C
												 11 – 120°C (default)
*/
#define BQ2589X_REG_08         0x08

#define BQ2589X_BAT_COMP_MASK  0xE0 // BIT 5-7 11100000
#define BQ2589X_BAT_COMP_SHIFT 5
#define BQ2589X_BAT_COMP_BASE  0
#define BQ2589X_BAT_COMP_LSB   20   // 20mΩ

#define BQ2589X_VCLAMP_MASK    0x1C // BIT 2-4 00011100
#define BQ2589X_VCLAMP_SHIFT   2
#define BQ2589X_VCLAMP_BASE    0    // 0 Offset
#define BQ2589X_VCLAMP_LSB     32   // 32mV

#define BQ2589X_TREG_MASK      0x03 // BIT 0-1 00000011
#define BQ2589X_TREG_SHIFT     0
#define BQ2589X_TREG_60C       0
#define BQ2589X_TREG_80C       1
#define BQ2589X_TREG_100C      2
#define BQ2589X_TREG_120C      3    // 120°C (default)

/* Register 0x09 *************************************************************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 1	 0	 0	 0	 1	 0	 0 : Chip Default
 0	 1	 0	 0	 0	 1	 0	 0 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	FORCE_ICO		R/W		REG_RST/Watchdog	Force Start Input Current Optimizer (ICO)
												 0 – Do not force ICO (default)
												 1 – Force ICO
												 Note: This bit is can only be set only and always returns to 0 after ICO starts
 6	TMR2X_EN		R/W		REG_RST/Watchdog	Safety Timer Setting during DPM or Thermal Regulation
												 0 – Safety timer not slowed by 2X during input DPM or thermal regulation
												 1 – Safety timer slowed by 2X during input DPM or thermal regulation (default)
 5	BATFET_DIS		R/W		REG_RST				Force BATFET off to enable ship mode
												 0 – Allow BATFET turn on (default)
												 1 – Force BATFET off
 4	JEITA_VSET		R/W		REG_RST/Watchdog	JEITA High Temperature Voltage Setting
		45-60C									 0 – Set Charge Voltage to VREG-200mV during JEITA hig temperature (default)
												 1 – Set Charge Voltage to VREG during JEITA high temperature
 3	BATFET_DLY		R/W		REG_RST				BATFET turn off delay control
												 0 – BATFET turn off immediately when BATFET_DIS bit is set (default)
												 1 – BATFET turn off delay by tSM_DLY when BATFET_DIS bit is set
 2	BATFET_RST_EN	R/W		REG_RST				BATFET full system reset enable
												 0 – Disable BATFET full system reset
												 1 – Enable BATFET full system reset (default)
 1	PUMPX_UP		R/W		REG_RST/Watchdog	Current pulse control voltage up enable
												 0 – Disable (default)
												 1 – Enable
												 Note: This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed
 0	PUMPX_DN		R/W		REG_RST/Watchdog	Current pulse control voltage down enable
												 0 – Disable (default)
												 1 – Enable
												 Note: This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed
*/
#define BQ2589X_REG_09              0x09

#define BQ2589X_FORCE_ICO_MASK      0x80 // BIT 7   10000000
#define BQ2589X_FORCE_ICO_SHIFT     7
#define BQ2589X_FORCE_ICO           1

#define BQ2589X_TMR2X_EN_MASK       0x40 // BIT 6   01000000
#define BQ2589X_TMR2X_EN_SHIFT      6

#define BQ2589X_BATFET_DIS_MASK     0x20 // BIT 5   00100000
#define BQ2589X_BATFET_DIS_SHIFT    5
#define BQ2589X_BATFET_OFF          1

#define BQ2589X_JEITA_VSET_MASK     0x10 // BIT 4   00010000
#define BQ2589X_JEITA_VSET_SHIFT    4
#define BQ2589X_JEITA_VSET_N150MV   0
#define BQ2589X_JEITA_VSET_VREG     1

#define BQ2589X_BATFET_RST_EN_MASK  0x04 // BIT 2   00000100
#define BQ2589X_BATFET_RST_EN_SHIFT 2

#define BQ2589X_PUMPX_UP_MASK       0x02 // BIT 1   00000010
#define BQ2589X_PUMPX_UP_SHIFT      1
#define BQ2589X_PUMPX_UP            1

#define BQ2589X_PUMPX_DOWN_MASK     0x01 // BIT 0   00000001
#define BQ2589X_PUMPX_DOWN_SHIFT    0
#define BQ2589X_PUMPX_DOWN          1

/* Register 0x0A **********************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 1	 1	 1	 0	 0	 1	 1 : Chip Default
 0	 1	 1	 1	 0	 1	 1	 0 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	BOOSTV[3]		R/W		REG_RST/Watchdog	512mV  Boost Mode Voltage Regulation
 6	BOOSTV[2]		R/W		REG_RST/Watchdog	256mV   Offset: 4.55V
 5	BOOSTV[1]		R/W		REG_RST				128mV   Range: 4.55V – 5.51V
 4	BOOSTV[0]		R/W		REG_RST/Watchdog	64mV    Default:4.998V(0111)
 3	PFM_OTG_DIS		R/W		REG_RST				PFM mode allowed in boost mode
												 0 – Allow PFM in boost mode (default)
												 1 – Disable PFM in boost mode
 2	BOOST_LIM[2]	R/W		REG_RST/Watchdog	000: 0.5A      Boost Mode Current Limit
 1	BOOST_LIM[1]	R/W		REG_RST/Watchdog	001: 0.75A      Default: 1.4A (011)
 0	BOOST_LIM[0]	R/W		REG_RST/Watchdog	010: 1.2A
												011: 1.4A
												100: 1.65A
												101: 1.875A
												110: 2.15A
												111: Reserved
*/
#define BQ2589X_REG_0A           0x0A

#define BQ2589X_BOOSTV_MASK      0xF0 // BIT 4-7 11110000
#define BQ2589X_BOOSTV_SHIFT     4
#define BQ2589X_BOOSTV_BASE      4550 // 4550mV Offset
#define BQ2589X_BOOSTV_LSB       64   // 64mV

#define BQ2589X_BOOST_LIM_MASK   0x07 // BIT 0-2 00000111
#define BQ2589X_BOOST_LIM_SHIFT  0
#define BQ2589X_BOOST_LIM_500MA  0x00 // 0.5A
#define BQ2589X_BOOST_LIM_500MA_VALUE 500
#define BQ2589X_BOOST_LIM_750MA  0x01 // 0.75A
#define BQ2589X_BOOST_LIM_750MA_VALUE 750
#define BQ2589X_BOOST_LIM_1200MA 0x02 // 1.2A
#define BQ2589X_BOOST_LIM_1200MA_VALUE 1200
#define BQ2589X_BOOST_LIM_1400MA 0x03 // 1.4A (default)
#define BQ2589X_BOOST_LIM_1400MA_VALUE 1400
#define BQ2589X_BOOST_LIM_1650MA 0x04 // 1.65A
#define BQ2589X_BOOST_LIM_1650MA_VALUE 1650
#define BQ2589X_BOOST_LIM_1875MA 0x05 // 1.87A
#define BQ2589X_BOOST_LIM_1875MA_VALUE 1875
#define BQ2589X_BOOST_LIM_2150MA 0x06 // 2.15A
#define BQ2589X_BOOST_LIM_2150MA_VALUE 2150

/* Register 0x0B ********************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 x	 x	 x	 x	 x	 x	 x	 x : Chip Default
 0	 1	 0	 1	 1	 1	 1	 0 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	VBUS_STAT[2]	R		N/A					VBUS Status register
 6	VBUS_STAT[1]	R		N/A					 000: No Input
 5	VBUS_STAT[0]	R		N/A					 001: USB Host SDP
												 010: Adapter (3.25A)
												 111: OTG
												 Note: Software current limit is reported in IINLIM register
 4	CHRG_STAT[1]	R		N/A					Charging Status
 3	CHRG_STAT[0]	R		N/A					 00 – Not Charging
												 01 – Pre-charge (< VBATLOWV)
												 10 – Fast Charging
												 11 – Charge Termination Done
 2	PG_STAT			R		N/A					Power Good Status
												 0 – Not Power Good
												 1 – Power Good
 1	Reserved		R		N/A					Reserved: Always reads 1
 0	VSYS_STAT		R		N/A					VSYS Regulation Status
												 0 – Not in VSYSMIN regulation (BAT > VSYSMIN)
												 1 – In VSYSMIN regulation (BAT < VSYSMIN)
*/
#define BQ2589X_REG_0B            0x0B

#define BQ2589X_VBUS_STAT_MASK    0xE0            // BIT 5-7 11100000
#define BQ2589X_VBUS_STAT_SHIFT   5
#define BQ2589X_VBUS_STAT_00      "Battery"
#define BQ2589X_VBUS_STAT_01      "USB Host SDP"
#define BQ2589X_VBUS_STAT_02      "Adapter 3.25A"
#define BQ2589X_VBUS_STAT_03      "USB CDP 3.25A"
#define BQ2589X_VBUS_STAT_04      "USB DCP 3.25A"
#define BQ2589X_VBUS_STAT_05      "Unknown 500mA"
#define BQ2589X_VBUS_STAT_06      "Non Standard"
#define BQ2589X_VBUS_STAT_07      "USB OTG"
#define BQ2589X_VBUS_STAT_XX      "0Be1"          // Error
#define BQ2589X_VBUS_UNKNOWN      8

#define BQ2589X_CHRG_STAT_MASK    0x18            // BIT 3-4 00011000
#define BQ2589X_CHRG_STAT_SHIFT   3
#define BQ2589X_CHRG_STAT_00      "Idle"
#define BQ2589X_CHRG_STAT_01      "Pre-charge"
#define BQ2589X_CHRG_STAT_02      "Fast Charge"
#define BQ2589X_CHRG_STAT_03      "Complete"
#define BQ2589X_CHRG_STAT_XX      "0Be2"          // Error
#define BQ2589X_CHRG_STAT_CHGDONE 3

#define BQ2589X_PG_STAT_MASK      0x04            // BIT 2   00000100
#define BQ2589X_PG_STAT_SHIFT     2

#define BQ2589X_VSYS_STAT_MASK    0x01            // BIT 0   00000001
#define BQ2589X_VSYS_STAT_SHIFT   0

/* Register 0x0C ************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 x	 x	 x	 x	 x	 x	 x	 x : Chip Default
 0	 0	 0	 0	 0	 0	 0	 0 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	WATCHDOG_FAULT	R		N/A					Watchdog Fault Status
												 Status 0 – Normal
												 1 - Watchdog timer expiration
 6	BOOST_FAULT		R		N/A					Boost Mode Fault Status
												 0 – Normal
												 1 - VBUS overloaded in OTG, or VBUS OVP, or battery is too low in boost mode
 5	CHRG_FAULT[1]	R		N/A					Charge Fault Status
 4	CHRG_FAULT[0]	R		N/A					 00 – Normal
												 01 – Input fault (VBUS > VACOV or VBAT < VBUS < VVBUSMIN(typical 3.8V))
												 10 - Thermal shutdown
												 11 – Charge Safety Timer Expiration
 3	BAT_FAULT		R		N/A					Battery Fault Status
												 0 – Normal
												 1 – BATOVP (VBAT > VBATOVP)
 2	NTC_FAULT[2]	R		N/A					NTC Fault Status
 1	NTC_FAULT[1]	R		N/A					 Buck Mode:
 0	NTC_FAULT[0]	R		N/A					 000 – Normal
												 010 – TS Warm
												 011 – TS Cool
												 101 – TS Cold
												 110 – TS Hot
												 Boost Mode:
												 000 – Normal
												 101 – TS Cold
												 110 – TS Hot
*/
#define BQ2589X_REG_0C             0x0C

#define BQ2589X_FAULT_WDT_MASK     0x80 // BIT 7   10000000
#define BQ2589X_FAULT_WDT_SHIFT    7

#define BQ2589X_FAULT_BOOST_MASK   0x40 // BIT 6   01000000
#define BQ2589X_FAULT_BOOST_SHIFT  6

#define BQ2589X_FAULT_CHRG_MASK    0x30 // BIT 4-5 00110000
#define BQ2589X_FAULT_CHRG_SHIFT   4
#define BQ2589X_FAULT_CHRG_NORMAL  0
#define BQ2589X_FAULT_CHRG_INPUT   1
#define BQ2589X_FAULT_CHRG_THERMAL 2
#define BQ2589X_FAULT_CHRG_TIMER   3

#define BQ2589X_FAULT_BAT_MASK     0x08 // BIT 3   00001000
#define BQ2589X_FAULT_BAT_SHIFT    3

#define BQ2589X_FAULT_NTC_MASK     0x07 // BIT 0-2 00000111
#define BQ2589X_FAULT_NTC_SHIFT    0

#define BQ2589X_FAULT_NTC_NORM     0
#define BQ2589X_FAULT_NTC_WARM     2
#define BQ2589X_FAULT_NTC_COOL     3
#define BQ2589X_FAULT_NTC_COLD     5
#define BQ2589X_FAULT_NTC_HOT      6

/* Register 0x0D ********************************************************************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 1	 0	 0	 1	 0 : Chip Default
 0	 0	 0	 1	 0	 0	 1	 1 : Code Default
R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W

Bit	Field			Type	Reset				Description
 7	FORCE_VINDPM	R/W		REG_RST				VINDPM Threshold Setting Method
												 0 – Run Relative VINDPM Threshold (default)
												 1 – Run Absolute VINDPM Threshold
												 Note: Register is reset to default value when input source is plugged-in
 6	VINDPM[6]		R/W		REG_RST				6400mV  Absolute VINDPM Threshold
 5	VINDPM[5]		R/W		REG_RST				3200mV   Offset: 2.6V
 4	VINDPM[4]		R/W		REG_RST				1600mV   Range: 3.9V (0001101) – 15.3V (1111111)
 3	VINDPM[3]		R/W		REG_RST				800mV    Default: 4.4V (0010010)
 2	VINDPM[2]		R/W		REG_RST				400mV    Note: Value < 0001101 is clamped to 3.9V (0001101)
 1	VINDPM[1]		R/W		REG_RST				200mV    Register is read only when FORCE_VINDPM=0 and can be written by internal control based on relative VINDPM threshold setting
 0	VINDPM[0]		R/W		REG_RST				100mV    Register can be read/write when FORCE_VINDPM = 1
														 Note: Register is reset to default value when input source is plugged-in
*/
#define BQ2589X_REG_0D               0x0D

#define BQ2589X_FORCE_VINDPM_MASK    0x80 // BIT 7   10000000
#define BQ2589X_FORCE_VINDPM_SHIFT   7
#define BQ2589X_FORCE_VINDPM_ENABLE  1
#define BQ2589X_FORCE_VINDPM_DISABLE 0

#define BQ2589X_VINDPM_MASK          0x7F // BIT 0-6 01111111
#define BQ2589X_VINDPM_SHIFT         0
#define BQ2589X_VINDPM_BASE          2600 // 2600mV Offset
#define BQ2589X_VINDPM_LSB           100  // 100mV

/* Register 0x0E **********************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 0	 0 : Chip Default
 0	 0	 0	 0	 0	 0	 0	 0 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	THERM_STAT		R		N/A					Thermal Regulation Status
												 0 – Normal
												 1 – In Thermal Regulation
 6	BATV[6]			R		N/A					1280mV  ADC conversion of Battery Voltage (VBAT)
 5	BATV[5]			R		N/A					640mV    Offset: 2.304V
 4	BATV[4]			R		N/A					320mV    Range: 2.304V (0000000) – 4.848V (1111111)
 3	BATV[3]			R		N/A					160mV    Default: 2.304V (0000000)
 2	BATV[2]			R		N/A					80mV
 1	BATV[1]			R		N/A					40mV
 0	BATV[0]			R		N/A					20mV
*/
#define BQ2589X_REG_0E           0x0E

#define BQ2589X_THERM_STAT_MASK  0x80 // BIT 7   10000000
#define BQ2589X_THERM_STAT_SHIFT 7

#define BQ2589X_BATV_MASK        0x7F // BIT 0-6 01111111
#define BQ2589X_BATV_SHIFT       0
#define BQ2589X_BATV_BASE        2304
#define BQ2589X_BATV_LSB         20

/* Register 0x0F **********************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 0	 0 : Chip Default
 0	 1	 0	 0	 0	 1	 0	 0 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	Reserved		R		N/A					Reserved: Always reads 0
 6	SYSV[6]			R		N/A					1280mV  ADC conversion of System Voltage (VSYS)
 5	SYSV[5]			R		N/A					640mV    Offset: 2.304V
 4	SYSV[4]			R		N/A					320mV	 Range: 2.304V (0000000) – 4.848V (1111111)
 3	SYSV[3]			R		N/A					160mV	 Default: 2.304V (0000000)
 2	SYSV[2]			R		N/A					80mV
 1	SYSV[1]			R		N/A					40mV
 0	SYSV[0]			R		N/A					20mV
*/
#define BQ2589X_REG_0F     0x0F

#define BQ2589X_SYSV_MASK  0x7F // BIT 0-6 01111111
#define BQ2589X_SYSV_SHIFT 0
#define BQ2589X_SYSV_BASE  2304
#define BQ2589X_SYSV_LSB   20

/* Register 0x10 **********************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 0	 0 : Chip Default
 0	 1	 0	 1	 0	 1	 1	 0 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	Reserved		R		N/A					Reserved: Always reads 0
 6	TSPCT[6]		R		N/A					29.76%  ADC conversion of TS Voltage (TS) as percentage of REGN
 5	TSPCT[5]		R		N/A					14.88%   Offset: 21%
 4	TSPCT[4]		R		N/A					7.44%    Range 21% (0000000) – 80% (1111111)
 3	TSPCT[3]		R		N/A					3.72%    Default: 21% (0000000)
 2	TSPCT[2]		R		N/A					1.86%
 1	TSPCT[1]		R		N/A					0.93%
 0	TSPCT[0]		R		N/A					0.465%
*/
#define BQ2589X_REG_10      0x10

#define BQ2589X_TSPCT_MASK  0x7F // BIT 0-6 01111111
#define BQ2589X_TSPCT_SHIFT 0
#define BQ2589X_TSPCT_BASE  21
#define BQ2589X_TSPCT_LSB   0.465 //should be 0.465,kernel does not support float

/* Register 0x11 ******************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 0	 0 : Chip Default
 1	 0	 0	 1	 1	 0	 0	 1 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	VBUS_GD			R		N/A					VBUS Good Status
												 0 – Not VBUS attached
												 1 – VBUS Attached
 6	VBUSV[6]		R		N/A					6400mV  ADC conversion of VBUS voltage (VBUS)
 5	VBUSV[5]		R		N/A					3200mV   Offset: 2.6V
 4	VBUSV[4]		R		N/A					1600mV   Range 2.6V (0000000) – 15.3V (1111111)
 3	VBUSV[3]		R		N/A					800mV    Default: 2.6V (0000000)
 2	VBUSV[2]		R		N/A					400mV
 1	VBUSV[1]		R		N/A					200mV
 0	VBUSV[0]		R		N/A					100mV
*/
#define BQ2589X_REG_11        0x11

#define BQ2589X_VBUS_GD_MASK  0x80 // BIT 7   10000000
#define BQ2589X_VBUS_GD_SHIFT 7

#define BQ2589X_VBUSV_MASK    0x7F // BIT 0-6 01111111
#define BQ2589X_VBUSV_SHIFT   0
#define BQ2589X_VBUSV_BASE    2600
#define BQ2589X_VBUSV_LSB     100

/* Register 0x12 ****************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 0	 0 : Chip Default
 0	 0	 0	 0	 0	 0	 0	 0 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	Unused			R		N/A					Always reads 0
 6	ICHGR[6]		R		N/A					3200mA  ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT
 5	ICHGR[5]		R		N/A					1600mA   Offset: 0mA
 4	ICHGR[4]		R		N/A					800mA    Range 0mA (0000000) – 6350mA (1111111)
 3	ICHGR[3]		R		N/A					400mA    Default: 0mA (0000000)
 2	ICHGR[2]		R		N/A					200mA    Note: This register returns 0000000 for VBAT < VBATSHORT
 1	ICHGR[1]		R		N/A					100mA
 0	ICHGR[0]		R		N/A					50mA
*/
#define BQ2589X_REG_12      0x12

#define BQ2589X_ICHGR_MASK  0x7F // BIT 0-6 01111111
#define BQ2589X_ICHGR_SHIFT 0
#define BQ2589X_ICHGR_BASE  0    // 0mA Offset
#define BQ2589X_ICHGR_LSB   50   // 50mA

/* Register 0x13 *******************************************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 0	 0 : Chip Default
 0	 0	 1	 1	 1	 1	 1	 1 : Code Default
 R	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	VDPM_STAT		R		N/A					VINDPM Status
												 0 – Not in VINDPM
												 1 – VINDPM
 6	IDPM_STAT		R		N/A					IINDPM Status
												 0 – Not in IINDPM
												 1 – IINDPM
 5	IDPM_LIM[5]		R		N/A					1600mA  Input Current Limit in effect while Input Current Optimizer (ICO) is enabled
 4	IDPM_LIM[4]		R		N/A					800mA    Offset: 100mA (default)
 3	IDPM_LIM[3]		R		N/A					400mA    Range 100mA (0000000) – 3.25A (111111)
 2	IDPM_LIM[2]		R		N/A					200mA
 1	IDPM_LIM[1]		R		N/A					100mA
 0	IDPM_LIM[0]		R		N/A					50mA
*/
#define BQ2589X_REG_13          0x13

#define BQ2589X_VDPM_STAT_MASK  0x80 // BIT 7   10000000
#define BQ2589X_VDPM_STAT_SHIFT 7

#define BQ2589X_IDPM_STAT_MASK  0x40 // BIT 6   01000000
#define BQ2589X_IDPM_STAT_SHIFT 6

#define BQ2589X_IDPM_LIM_MASK   0x3F // BIT 0-5 00111111
#define BQ2589X_IDPM_LIM_SHIFT  0
#define BQ2589X_IDPM_LIM_BASE   100  // 100mA Offset
#define BQ2589X_IDPM_LIM_LSB    50   // 50mA

/* Register 0x14 ******************************************************************************************
 7	 6	 5	 4	 3	 2	 1	 0 : BIT
 0	 0	 0	 0	 0	 0	 1	 0 : Chip Default
 0	 0	 0	 0	 0	 1	 1	 0 : Code Default
R/W	 R	 R	 R	 R	 R	 R	 R

Bit	Field			Type	Reset				Description
 7	REG_RST			R/W		N/A					Register Reset
												 0 – Keep current register setting (default)
												 1 – Reset to default register value and reset safety timer
												 Note: Reset to 0 after register reset is completed
 6	ICO_OPTIMIZED	R		N/A					Input Current Optimizer (ICO) Status
												 0 – Optimization is in progress
												 1 – Maximum Input Current Detected
 5	PN[2]			R		N/A					Device Configuration
 4	PN[1]			R		N/A					 000: bq25896
 3	PN[0]			R		N/A					
 2	TS_PROFILE		R		N/A					Temperature Profile
												 1- JEITA (default)
 1	DEV_REV[1]		R		N/A					Device Revision
 0	DEV_REV[0]		R		N/A					 10
*/
#define BQ2589X_REG_14              0x14

#define BQ2589X_RESET_MASK          0x80 // BIT 7   10000000
#define BQ2589X_RESET_SHIFT         7
#define BQ2589X_RESET               1

#define BQ2589X_ICO_OPTIMIZED_MASK  0x40 // BIT 6   01000000
#define BQ2589X_ICO_OPTIMIZED_SHIFT 6

#define BQ2589X_PN_MASK             0x38 // BIT 3-5 00111000
#define BQ2589X_PN_SHIFT            3

#define BQ2589X_TS_PROFILE_MASK     0x04 // BIT 2   00000100
#define BQ2589X_TS_PROFILE_SHIFT    2

#define BQ2589X_DEV_REV_MASK        0x03 // BIT 0-1 00000011
#define BQ2589X_DEV_REV_SHIFT       0

#endif