/*
 * arch/arm/mach-tegra/odm_kit/query/harmony/subboards/nvodm_query_discovery_e1162_peripherals.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1162 module
 *
 * Copyright (c) 2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


// AP20 doesn't have PLL_D rail.
// PLLD (NV reserved) / Use PLL_U
{
    NV_VDD_PLLD_ODM_ID,
    s_PllUAddresses,
    NV_ARRAY_SIZE(s_PllUAddresses),
    NvOdmPeripheralClass_Other
},

// RTC (NV reserved)
{
    NV_VDD_RTC_ODM_ID,
    s_RtcAddresses,
    NV_ARRAY_SIZE(s_RtcAddresses),
    NvOdmPeripheralClass_Other
},

// CORE (NV reserved)
{
    NV_VDD_CORE_ODM_ID,
    s_CoreAddresses,
    NV_ARRAY_SIZE(s_CoreAddresses),
    NvOdmPeripheralClass_Other
},

// CPU (NV reserved)
{
    NV_VDD_CPU_ODM_ID,
    s_ffaCpuAddresses,
    NV_ARRAY_SIZE(s_ffaCpuAddresses),
    NvOdmPeripheralClass_Other
},

// PLLA (NV reserved)
{
    NV_VDD_PLLA_ODM_ID,
    s_PllAAddresses,
    NV_ARRAY_SIZE(s_PllAAddresses),
    NvOdmPeripheralClass_Other
},

// PLLM (NV reserved)
{
    NV_VDD_PLLM_ODM_ID,
    s_PllMAddresses,
    NV_ARRAY_SIZE(s_PllMAddresses),
    NvOdmPeripheralClass_Other
},
    
// PLLP (NV reserved)
{
    NV_VDD_PLLP_ODM_ID,
    s_PllPAddresses,
    NV_ARRAY_SIZE(s_PllPAddresses),
    NvOdmPeripheralClass_Other
},

// PLLC (NV reserved)
{
    NV_VDD_PLLC_ODM_ID,
    s_PllCAddresses,
    NV_ARRAY_SIZE(s_PllCAddresses),
    NvOdmPeripheralClass_Other
},

// PLLE (NV reserved)
{
    NV_VDD_PLLE_ODM_ID,
    s_PllEAddresses,
    NV_ARRAY_SIZE(s_PllEAddresses),
    NvOdmPeripheralClass_Other
},

// PLLU (NV reserved)
{
    NV_VDD_PLLU_ODM_ID,
    s_PllUAddresses,
    NV_ARRAY_SIZE(s_PllUAddresses),
    NvOdmPeripheralClass_Other
},

// PLLU1 (NV reserved)
{
    NV_VDD_PLLU1_ODM_ID,
    s_ffaPllU1Addresses,
    NV_ARRAY_SIZE(s_ffaPllU1Addresses),
    NvOdmPeripheralClass_Other
},

// PLLS (NV reserved)
{
    NV_VDD_PLLS_ODM_ID,
    s_PllSAddresses,
    NV_ARRAY_SIZE(s_PllSAddresses),
    NvOdmPeripheralClass_Other
},

// HDMI PLL (NV reserved)
{
    NV_VDD_PLLHDMI_ODM_ID,
    s_PllHdmiAddresses,
    NV_ARRAY_SIZE(s_PllHdmiAddresses),
    NvOdmPeripheralClass_Other
},

// OSC VDD (NV reserved)
{
    NV_VDD_OSC_ODM_ID,
    s_VddOscAddresses,
    NV_ARRAY_SIZE(s_VddOscAddresses),
    NvOdmPeripheralClass_Other
},

// PLLX (NV reserved)
{
    NV_VDD_PLLX_ODM_ID,
    s_PllXAddresses,
    NV_ARRAY_SIZE(s_PllXAddresses),
    NvOdmPeripheralClass_Other
},

// PLL_USB (NV reserved)
{
    NV_VDD_PLL_USB_ODM_ID,
    s_PllUsbAddresses,
    NV_ARRAY_SIZE(s_PllUsbAddresses),
    NvOdmPeripheralClass_Other
},

// System IO VDD (NV reserved)
{
    NV_VDD_SYS_ODM_ID,
    s_VddSysAddresses,
    NV_ARRAY_SIZE(s_VddSysAddresses),
    NvOdmPeripheralClass_Other
},

// USB VDD (NV reserved)
{
    NV_VDD_USB_ODM_ID,
    s_VddUsbAddresses,
    NV_ARRAY_SIZE(s_VddUsbAddresses),
    NvOdmPeripheralClass_Other
},

// HDMI VDD (NV reserved)
{
    NV_VDD_HDMI_ODM_ID,
    s_VddHdmiAddresses,
    NV_ARRAY_SIZE(s_VddHdmiAddresses),
    NvOdmPeripheralClass_Other
},

// MIPI VDD (NV reserved) / AVDD_DSI_CSI
{
    NV_VDD_MIPI_ODM_ID,
    s_VddMipiAddresses,
    NV_ARRAY_SIZE(s_VddMipiAddresses),
    NvOdmPeripheralClass_Other
},

// LCD VDD (NV reserved)
{
    NV_VDD_LCD_ODM_ID,
    s_VddLcdAddresses,
    NV_ARRAY_SIZE(s_VddLcdAddresses),
    NvOdmPeripheralClass_Other
},

// AUDIO VDD (NV reserved)
{
    NV_VDD_AUD_ODM_ID,
    s_VddAudAddresses,
    NV_ARRAY_SIZE(s_VddAudAddresses),
    NvOdmPeripheralClass_Other
},

// DDR VDD (NV reserved)
{
    NV_VDD_DDR_ODM_ID,
    s_VddDdrAddresses,
    NV_ARRAY_SIZE(s_VddDdrAddresses),
    NvOdmPeripheralClass_Other
},

// DDR_RX (NV reserved)
{
    NV_VDD_DDR_RX_ODM_ID,
    s_VddDdrRxAddresses,
    NV_ARRAY_SIZE(s_VddDdrRxAddresses),
    NvOdmPeripheralClass_Other
},

// NAND VDD (NV reserved)
{
    NV_VDD_NAND_ODM_ID,
    s_VddNandAddresses,
    NV_ARRAY_SIZE(s_VddNandAddresses),
    NvOdmPeripheralClass_Other
},

// UART VDD (NV reserved)
{
    NV_VDD_UART_ODM_ID,
    s_VddUartAddresses,
    NV_ARRAY_SIZE(s_VddUartAddresses),
    NvOdmPeripheralClass_Other
},

// SDIO VDD (NV reserved)
{
    NV_VDD_SDIO_ODM_ID,
    s_VddSdioAddresses,
    NV_ARRAY_SIZE(s_VddSdioAddresses),
    NvOdmPeripheralClass_Other
},

// VDAC VDD (NV reserved)
{
    NV_VDD_VDAC_ODM_ID,
    s_VddVdacAddresses,
    NV_ARRAY_SIZE(s_VddVdacAddresses),
    NvOdmPeripheralClass_Other
},

// VI VDD (NV reserved)
{
    NV_VDD_VI_ODM_ID,
    s_VddViAddresses,
    NV_ARRAY_SIZE(s_VddViAddresses),
    NvOdmPeripheralClass_Other
},

// BB VDD (NV reserved)
{
    NV_VDD_BB_ODM_ID,
    s_VddBbAddresses,
    NV_ARRAY_SIZE(s_VddBbAddresses),
    NvOdmPeripheralClass_Other
},

// PEX_CLK (NV reserved)
{
    NV_VDD_PEX_CLK_ODM_ID,
    s_VddPexClkAddresses,
    NV_ARRAY_SIZE(s_VddPexClkAddresses),
    NvOdmPeripheralClass_Other
},

#if 0
//  VBUS
{
    NV_VDD_VBUS_ODM_ID,
    s_VddVBusAddresses,
    NV_ARRAY_SIZE(s_VddVBusAddresses),
    NvOdmPeripheralClass_Other
},
#endif

//SOC
{
    NV_VDD_SoC_ODM_ID,
    s_VddSocAddresses,
    NV_ARRAY_SIZE(s_VddSocAddresses),
    NvOdmPeripheralClass_Other
},

// Power for HDMI Hotplug
{
    NV_VDD_HDMI_INT_ID,
    s_HdmiHotplug,
    NV_ARRAY_SIZE(s_HdmiHotplug),
    NvOdmPeripheralClass_Other
},

//  PMU0
{
    NV_ODM_GUID('t','p','s','6','5','8','6','x'),
    s_Pmu0Addresses,
    NV_ARRAY_SIZE(s_Pmu0Addresses),
    NvOdmPeripheralClass_Other
},
// PMU voltage rails enabled by AP GPIOs
{
    TPS_EXT_GUID(Ext_TPS2051BPmuSupply_VDDIO_VID),
    s_Vddio_Vid_En,
    NV_ARRAY_SIZE(s_Vddio_Vid_En),
    NvOdmPeripheralClass_Other,
},
{
    TPS_EXT_GUID(Ext_SWITCHPmuSupply_VDDIO_SD),
    s_Vddio_Sd_En,
    NV_ARRAY_SIZE(s_Vddio_Sd_En),
    NvOdmPeripheralClass_Other,
},
{
    TPS_EXT_GUID(Ext_SWITCHPmuSupply_VDDIO_SDMMC),
    s_Vddio_Sdmmc_En,
    NV_ARRAY_SIZE(s_Vddio_Sdmmc_En),
    NvOdmPeripheralClass_Other,
},
{
    TPS_EXT_GUID(Ext_SWITCHPmuSupply_VDD_BL),
    s_Vddio_Bl_En,
    NV_ARRAY_SIZE(s_Vddio_Bl_En),
    NvOdmPeripheralClass_Other,
},
{
    TPS_EXT_GUID(Ext_SWITCHPmuSupply_VDD_PNL),
    s_Vddio_Pnl_En,
    NV_ARRAY_SIZE(s_Vddio_Pnl_En),
    NvOdmPeripheralClass_Other,
},

#if 0
//  ENC28J60 SPI Ethernet module
{
    NV_ODM_GUID('e','n','c','2','8','j','6','0'),
    s_SpiEthernetAddresses,
    NV_ARRAY_SIZE(s_SpiEthernetAddresses),
    NvOdmPeripheralClass_Other
},
#endif

#if 0
//  SMSC3317 ULPI USB PHY
{
    NV_ODM_GUID('s','m','s','c','3','3','1','7'),
    s_UlpiUsbAddresses,
    NV_ARRAY_SIZE(s_UlpiUsbAddresses),
    NvOdmPeripheralClass_Other
},
#endif


//  LVDS LCD Display
{
    NV_ODM_GUID('L','V','D','S','W','S','V','G'),   // LVDS WSVGA panel
    s_LvdsDisplayAddresses,
    NV_ARRAY_SIZE(s_LvdsDisplayAddresses),
    NvOdmPeripheralClass_Display
},

// HDMI (based on Concorde 2 design)
{
    NV_ODM_GUID('f','f','a','2','h','d','m','i'),
    s_HdmiAddresses,
    NV_ARRAY_SIZE(s_HdmiAddresses),
    NvOdmPeripheralClass_Display    
},

#if 0
// CRT (based on Concorde 2 design)
{
    NV_ODM_GUID('f','f','a','_','_','c','r','t'),
    s_CrtAddresses,
    NV_ARRAY_SIZE(s_CrtAddresses),
    NvOdmPeripheralClass_Display
},

// TV Out Video Dac
{
    NV_ODM_GUID('f','f','a','t','v','o','u','t'),
    s_ffaVideoDacAddresses,
    NV_ARRAY_SIZE(s_ffaVideoDacAddresses),
    NvOdmPeripheralClass_Display
},
#endif

// Sdio
{
    NV_ODM_GUID('s','d','i','o','_','m','e','m'),
    s_SdioAddresses,
    NV_ARRAY_SIZE(s_SdioAddresses),
    NvOdmPeripheralClass_Other
},

// I2c SmBus transport.
{
    NV_ODM_GUID('I','2','c','S','m','B','u','s'),
    s_I2cSmbusAddresses,
    NV_ARRAY_SIZE(s_I2cSmbusAddresses),
    NvOdmPeripheralClass_Other
},

#if 0
// USB Mux J7A1 and J6A1
{
   NV_ODM_GUID('u','s','b','m','x','J','7','6'),
    s_UsbMuxAddress,
    NV_ARRAY_SIZE(s_UsbMuxAddress),
    NvOdmPeripheralClass_Other

},
#endif

#if 0
// Qwerty key baord for 16x8
{
    NV_ODM_GUID('q','w','e','r','t','y',' ',' '),
    s_QwertyKeyPad16x8Addresses,
    NV_ARRAY_SIZE(s_QwertyKeyPad16x8Addresses),
    NvOdmPeripheralClass_HCI
},
#endif

// Temperature Monitor (TMON)
{
    NV_ODM_GUID('a','d','t','7','4','6','1',' '),
    s_Tmon0Addresses,
    NV_ARRAY_SIZE(s_Tmon0Addresses),
    NvOdmPeripheralClass_Other
},

// Bluetooth
{
    NV_ODM_GUID('l','b','e','e','9','q','m','b'),
    s_p1162BluetoothAddresses,
    NV_ARRAY_SIZE(s_p1162BluetoothAddresses),
    NvOdmPeripheralClass_Other
},

// Sdio wlan  on COMMs Module
{
    NV_ODM_GUID('s','d','i','o','w','l','a','n'),
    s_WlanAddresses,
    NV_ARRAY_SIZE(s_WlanAddresses),
    NvOdmPeripheralClass_Other
},

// Audio codec (I2C_PMU edition)
{
    NV_ODM_GUID('w','o','l','f','8','9','0','3'),
    s_AudioCodecAddresses,
    NV_ARRAY_SIZE(s_AudioCodecAddresses),
    NvOdmPeripheralClass_Other
},

// Audio codec (I2C_1 edition)
{
    NV_ODM_GUID('w','o','8','9','0','3','_','1'),
    s_AudioCodecAddressesI2C_1,
    NV_ARRAY_SIZE(s_AudioCodecAddressesI2C_1),
    NvOdmPeripheralClass_Other
},

#if 0
// Touch panel
{
    NV_ODM_GUID('p','a','n','j','i','t','_','0'),
    s_TouchPanelAddresses,
    NV_ARRAY_SIZE(s_TouchPanelAddresses),
    NvOdmPeripheralClass_HCI
},

// Accelerometer Module
{
    NV_ODM_GUID('b','m','a','1','5','0','a','c'),
    s_AcceleroAddresses,
    NV_ARRAY_SIZE(s_AcceleroAddresses),
    NvOdmPeripheralClass_Other,
},

// Kionix Accelerometer on Tango
{
    NV_ODM_GUID('k','x','t','9','-','0','0','0'),
    s_KXTFAcceleroAddresses,
    NV_ARRAY_SIZE(s_KXTFAcceleroAddresses),
    NvOdmPeripheralClass_Other,
},
#endif
// NOTE: This list *must* end with a trailing comma.
// AK4183 touch screen
{
	NV_ODM_GUID('a','k','4','1','8','3','t','s'),
	s_AK4183Address,
	NV_ARRAY_SIZE(s_AK4183Address),
	NvOdmPeripheralClass_HCI
},

// AT168 touch screen
{
	NV_ODM_GUID('a','t','e','1','6','8','t','s'),
	s_AT168Address,
	NV_ARRAY_SIZE(s_AT168Address),
	NvOdmPeripheralClass_HCI
},

// Bq24745 Battery.
{
	NV_ODM_GUID('b','q','2','4','7','4','5',' '),
	s_Bq24745Address,
	NV_ARRAY_SIZE(s_Bq24745Address),
	NvOdmPeripheralClass_Other
},
