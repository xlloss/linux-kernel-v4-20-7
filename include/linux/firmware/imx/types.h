/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017~2018 NXP
 *
 * Header file containing types used across multiple service APIs.
 */

#ifndef _SC_TYPES_H
#define _SC_TYPES_H

/*
 * This type is used to indicate a resource. Resources include peripherals
 * and bus masters (but not memory regions). Note items from list should
 * never be changed or removed (only added to at the end of the list).
 */
enum imx_sc_rsrc {
	IMX_SC_R_A53 = 0,
	IMX_SC_R_A53_0 = 1,
	IMX_SC_R_A53_1 = 2,
	IMX_SC_R_A53_2 = 3,
	IMX_SC_R_A53_3 = 4,
	IMX_SC_R_A72 = 5,
	IMX_SC_R_A72_0 = 6,
	IMX_SC_R_A72_1 = 7,
	IMX_SC_R_A72_2 = 8,
	IMX_SC_R_A72_3 = 9,
	IMX_SC_R_CCI = 10,
	IMX_SC_R_DB = 11,
	IMX_SC_R_DRC_0 = 12,
	IMX_SC_R_DRC_1 = 13,
	IMX_SC_R_GIC_SMMU = 14,
	IMX_SC_R_IRQSTR_M4_0 = 15,
	IMX_SC_R_IRQSTR_M4_1 = 16,
	IMX_SC_R_SMMU = 17,
	IMX_SC_R_GIC = 18,
	IMX_SC_R_DC_0_BLIT0 = 19,
	IMX_SC_R_DC_0_BLIT1 = 20,
	IMX_SC_R_DC_0_BLIT2 = 21,
	IMX_SC_R_DC_0_BLIT_OUT = 22,
	IMX_SC_R_DC_0_CAPTURE0 = 23,
	IMX_SC_R_DC_0_CAPTURE1 = 24,
	IMX_SC_R_DC_0_WARP = 25,
	IMX_SC_R_DC_0_INTEGRAL0 = 26,
	IMX_SC_R_DC_0_INTEGRAL1 = 27,
	IMX_SC_R_DC_0_VIDEO0 = 28,
	IMX_SC_R_DC_0_VIDEO1 = 29,
	IMX_SC_R_DC_0_FRAC0 = 30,
	IMX_SC_R_DC_0_FRAC1 = 31,
	IMX_SC_R_DC_0 = 32,
	IMX_SC_R_GPU_2_PID0 = 33,
	IMX_SC_R_DC_0_PLL_0 = 34,
	IMX_SC_R_DC_0_PLL_1 = 35,
	IMX_SC_R_DC_1_BLIT0 = 36,
	IMX_SC_R_DC_1_BLIT1 = 37,
	IMX_SC_R_DC_1_BLIT2 = 38,
	IMX_SC_R_DC_1_BLIT_OUT = 39,
	IMX_SC_R_DC_1_CAPTURE0 = 40,
	IMX_SC_R_DC_1_CAPTURE1 = 41,
	IMX_SC_R_DC_1_WARP = 42,
	IMX_SC_R_DC_1_INTEGRAL0 = 43,
	IMX_SC_R_DC_1_INTEGRAL1 = 44,
	IMX_SC_R_DC_1_VIDEO0 = 45,
	IMX_SC_R_DC_1_VIDEO1 = 46,
	IMX_SC_R_DC_1_FRAC0 = 47,
	IMX_SC_R_DC_1_FRAC1 = 48,
	IMX_SC_R_DC_1 = 49,
	IMX_SC_R_GPU_3_PID0 = 50,
	IMX_SC_R_DC_1_PLL_0 = 51,
	IMX_SC_R_DC_1_PLL_1 = 52,
	IMX_SC_R_SPI_0 = 53,
	IMX_SC_R_SPI_1 = 54,
	IMX_SC_R_SPI_2 = 55,
	IMX_SC_R_SPI_3 = 56,
	IMX_SC_R_UART_0 = 57,
	IMX_SC_R_UART_1 = 58,
	IMX_SC_R_UART_2 = 59,
	IMX_SC_R_UART_3 = 60,
	IMX_SC_R_UART_4 = 61,
	IMX_SC_R_EMVSIM_0 = 62,
	IMX_SC_R_EMVSIM_1 = 63,
	IMX_SC_R_DMA_0_CH0 = 64,
	IMX_SC_R_DMA_0_CH1 = 65,
	IMX_SC_R_DMA_0_CH2 = 66,
	IMX_SC_R_DMA_0_CH3 = 67,
	IMX_SC_R_DMA_0_CH4 = 68,
	IMX_SC_R_DMA_0_CH5 = 69,
	IMX_SC_R_DMA_0_CH6 = 70,
	IMX_SC_R_DMA_0_CH7 = 71,
	IMX_SC_R_DMA_0_CH8 = 72,
	IMX_SC_R_DMA_0_CH9 = 73,
	IMX_SC_R_DMA_0_CH10 = 74,
	IMX_SC_R_DMA_0_CH11 = 75,
	IMX_SC_R_DMA_0_CH12 = 76,
	IMX_SC_R_DMA_0_CH13 = 77,
	IMX_SC_R_DMA_0_CH14 = 78,
	IMX_SC_R_DMA_0_CH15 = 79,
	IMX_SC_R_DMA_0_CH16 = 80,
	IMX_SC_R_DMA_0_CH17 = 81,
	IMX_SC_R_DMA_0_CH18 = 82,
	IMX_SC_R_DMA_0_CH19 = 83,
	IMX_SC_R_DMA_0_CH20 = 84,
	IMX_SC_R_DMA_0_CH21 = 85,
	IMX_SC_R_DMA_0_CH22 = 86,
	IMX_SC_R_DMA_0_CH23 = 87,
	IMX_SC_R_DMA_0_CH24 = 88,
	IMX_SC_R_DMA_0_CH25 = 89,
	IMX_SC_R_DMA_0_CH26 = 90,
	IMX_SC_R_DMA_0_CH27 = 91,
	IMX_SC_R_DMA_0_CH28 = 92,
	IMX_SC_R_DMA_0_CH29 = 93,
	IMX_SC_R_DMA_0_CH30 = 94,
	IMX_SC_R_DMA_0_CH31 = 95,
	IMX_SC_R_I2C_0 = 96,
	IMX_SC_R_I2C_1 = 97,
	IMX_SC_R_I2C_2 = 98,
	IMX_SC_R_I2C_3 = 99,
	IMX_SC_R_I2C_4 = 100,
	IMX_SC_R_ADC_0 = 101,
	IMX_SC_R_ADC_1 = 102,
	IMX_SC_R_FTM_0 = 103,
	IMX_SC_R_FTM_1 = 104,
	IMX_SC_R_CAN_0 = 105,
	IMX_SC_R_CAN_1 = 106,
	IMX_SC_R_CAN_2 = 107,
	IMX_SC_R_DMA_1_CH0 = 108,
	IMX_SC_R_DMA_1_CH1 = 109,
	IMX_SC_R_DMA_1_CH2 = 110,
	IMX_SC_R_DMA_1_CH3 = 111,
	IMX_SC_R_DMA_1_CH4 = 112,
	IMX_SC_R_DMA_1_CH5 = 113,
	IMX_SC_R_DMA_1_CH6 = 114,
	IMX_SC_R_DMA_1_CH7 = 115,
	IMX_SC_R_DMA_1_CH8 = 116,
	IMX_SC_R_DMA_1_CH9 = 117,
	IMX_SC_R_DMA_1_CH10 = 118,
	IMX_SC_R_DMA_1_CH11 = 119,
	IMX_SC_R_DMA_1_CH12 = 120,
	IMX_SC_R_DMA_1_CH13 = 121,
	IMX_SC_R_DMA_1_CH14 = 122,
	IMX_SC_R_DMA_1_CH15 = 123,
	IMX_SC_R_DMA_1_CH16 = 124,
	IMX_SC_R_DMA_1_CH17 = 125,
	IMX_SC_R_DMA_1_CH18 = 126,
	IMX_SC_R_DMA_1_CH19 = 127,
	IMX_SC_R_DMA_1_CH20 = 128,
	IMX_SC_R_DMA_1_CH21 = 129,
	IMX_SC_R_DMA_1_CH22 = 130,
	IMX_SC_R_DMA_1_CH23 = 131,
	IMX_SC_R_DMA_1_CH24 = 132,
	IMX_SC_R_DMA_1_CH25 = 133,
	IMX_SC_R_DMA_1_CH26 = 134,
	IMX_SC_R_DMA_1_CH27 = 135,
	IMX_SC_R_DMA_1_CH28 = 136,
	IMX_SC_R_DMA_1_CH29 = 137,
	IMX_SC_R_DMA_1_CH30 = 138,
	IMX_SC_R_DMA_1_CH31 = 139,
	IMX_SC_R_UNUSED1 = 140,
	IMX_SC_R_UNUSED2 = 141,
	IMX_SC_R_UNUSED3 = 142,
	IMX_SC_R_UNUSED4 = 143,
	IMX_SC_R_GPU_0_PID0 = 144,
	IMX_SC_R_GPU_0_PID1 = 145,
	IMX_SC_R_GPU_0_PID2 = 146,
	IMX_SC_R_GPU_0_PID3 = 147,
	IMX_SC_R_GPU_1_PID0 = 148,
	IMX_SC_R_GPU_1_PID1 = 149,
	IMX_SC_R_GPU_1_PID2 = 150,
	IMX_SC_R_GPU_1_PID3 = 151,
	IMX_SC_R_PCIE_A = 152,
	IMX_SC_R_SERDES_0 = 153,
	IMX_SC_R_MATCH_0 = 154,
	IMX_SC_R_MATCH_1 = 155,
	IMX_SC_R_MATCH_2 = 156,
	IMX_SC_R_MATCH_3 = 157,
	IMX_SC_R_MATCH_4 = 158,
	IMX_SC_R_MATCH_5 = 159,
	IMX_SC_R_MATCH_6 = 160,
	IMX_SC_R_MATCH_7 = 161,
	IMX_SC_R_MATCH_8 = 162,
	IMX_SC_R_MATCH_9 = 163,
	IMX_SC_R_MATCH_10 = 164,
	IMX_SC_R_MATCH_11 = 165,
	IMX_SC_R_MATCH_12 = 166,
	IMX_SC_R_MATCH_13 = 167,
	IMX_SC_R_MATCH_14 = 168,
	IMX_SC_R_PCIE_B = 169,
	IMX_SC_R_SATA_0 = 170,
	IMX_SC_R_SERDES_1 = 171,
	IMX_SC_R_HSIO_GPIO = 172,
	IMX_SC_R_MATCH_15 = 173,
	IMX_SC_R_MATCH_16 = 174,
	IMX_SC_R_MATCH_17 = 175,
	IMX_SC_R_MATCH_18 = 176,
	IMX_SC_R_MATCH_19 = 177,
	IMX_SC_R_MATCH_20 = 178,
	IMX_SC_R_MATCH_21 = 179,
	IMX_SC_R_MATCH_22 = 180,
	IMX_SC_R_MATCH_23 = 181,
	IMX_SC_R_MATCH_24 = 182,
	IMX_SC_R_MATCH_25 = 183,
	IMX_SC_R_MATCH_26 = 184,
	IMX_SC_R_MATCH_27 = 185,
	IMX_SC_R_MATCH_28 = 186,
	IMX_SC_R_LCD_0 = 187,
	IMX_SC_R_LCD_0_PWM_0 = 188,
	IMX_SC_R_LCD_0_I2C_0 = 189,
	IMX_SC_R_LCD_0_I2C_1 = 190,
	IMX_SC_R_PWM_0 = 191,
	IMX_SC_R_PWM_1 = 192,
	IMX_SC_R_PWM_2 = 193,
	IMX_SC_R_PWM_3 = 194,
	IMX_SC_R_PWM_4 = 195,
	IMX_SC_R_PWM_5 = 196,
	IMX_SC_R_PWM_6 = 197,
	IMX_SC_R_PWM_7 = 198,
	IMX_SC_R_GPIO_0 = 199,
	IMX_SC_R_GPIO_1 = 200,
	IMX_SC_R_GPIO_2 = 201,
	IMX_SC_R_GPIO_3 = 202,
	IMX_SC_R_GPIO_4 = 203,
	IMX_SC_R_GPIO_5 = 204,
	IMX_SC_R_GPIO_6 = 205,
	IMX_SC_R_GPIO_7 = 206,
	IMX_SC_R_GPT_0 = 207,
	IMX_SC_R_GPT_1 = 208,
	IMX_SC_R_GPT_2 = 209,
	IMX_SC_R_GPT_3 = 210,
	IMX_SC_R_GPT_4 = 211,
	IMX_SC_R_KPP = 212,
	IMX_SC_R_MU_0A = 213,
	IMX_SC_R_MU_1A = 214,
	IMX_SC_R_MU_2A = 215,
	IMX_SC_R_MU_3A = 216,
	IMX_SC_R_MU_4A = 217,
	IMX_SC_R_MU_5A = 218,
	IMX_SC_R_MU_6A = 219,
	IMX_SC_R_MU_7A = 220,
	IMX_SC_R_MU_8A = 221,
	IMX_SC_R_MU_9A = 222,
	IMX_SC_R_MU_10A = 223,
	IMX_SC_R_MU_11A = 224,
	IMX_SC_R_MU_12A = 225,
	IMX_SC_R_MU_13A = 226,
	IMX_SC_R_MU_5B = 227,
	IMX_SC_R_MU_6B = 228,
	IMX_SC_R_MU_7B = 229,
	IMX_SC_R_MU_8B = 230,
	IMX_SC_R_MU_9B = 231,
	IMX_SC_R_MU_10B = 232,
	IMX_SC_R_MU_11B = 233,
	IMX_SC_R_MU_12B = 234,
	IMX_SC_R_MU_13B = 235,
	IMX_SC_R_ROM_0 = 236,
	IMX_SC_R_FSPI_0 = 237,
	IMX_SC_R_FSPI_1 = 238,
	IMX_SC_R_IEE = 239,
	IMX_SC_R_IEE_R0 = 240,
	IMX_SC_R_IEE_R1 = 241,
	IMX_SC_R_IEE_R2 = 242,
	IMX_SC_R_IEE_R3 = 243,
	IMX_SC_R_IEE_R4 = 244,
	IMX_SC_R_IEE_R5 = 245,
	IMX_SC_R_IEE_R6 = 246,
	IMX_SC_R_IEE_R7 = 247,
	IMX_SC_R_SDHC_0 = 248,
	IMX_SC_R_SDHC_1 = 249,
	IMX_SC_R_SDHC_2 = 250,
	IMX_SC_R_ENET_0 = 251,
	IMX_SC_R_ENET_1 = 252,
	IMX_SC_R_MLB_0 = 253,
	IMX_SC_R_DMA_2_CH0 = 254,
	IMX_SC_R_DMA_2_CH1 = 255,
	IMX_SC_R_DMA_2_CH2 = 256,
	IMX_SC_R_DMA_2_CH3 = 257,
	IMX_SC_R_DMA_2_CH4 = 258,
	IMX_SC_R_USB_0 = 259,
	IMX_SC_R_USB_1 = 260,
	IMX_SC_R_USB_0_PHY = 261,
	IMX_SC_R_USB_2 = 262,
	IMX_SC_R_USB_2_PHY = 263,
	IMX_SC_R_DTCP = 264,
	IMX_SC_R_NAND = 265,
	IMX_SC_R_LVDS_0 = 266,
	IMX_SC_R_LVDS_0_PWM_0 = 267,
	IMX_SC_R_LVDS_0_I2C_0 = 268,
	IMX_SC_R_LVDS_0_I2C_1 = 269,
	IMX_SC_R_LVDS_1 = 270,
	IMX_SC_R_LVDS_1_PWM_0 = 271,
	IMX_SC_R_LVDS_1_I2C_0 = 272,
	IMX_SC_R_LVDS_1_I2C_1 = 273,
	IMX_SC_R_LVDS_2 = 274,
	IMX_SC_R_LVDS_2_PWM_0 = 275,
	IMX_SC_R_LVDS_2_I2C_0 = 276,
	IMX_SC_R_LVDS_2_I2C_1 = 277,
	IMX_SC_R_M4_0_PID0 = 278,
	IMX_SC_R_M4_0_PID1 = 279,
	IMX_SC_R_M4_0_PID2 = 280,
	IMX_SC_R_M4_0_PID3 = 281,
	IMX_SC_R_M4_0_PID4 = 282,
	IMX_SC_R_M4_0_RGPIO = 283,
	IMX_SC_R_M4_0_SEMA42 = 284,
	IMX_SC_R_M4_0_TPM = 285,
	IMX_SC_R_M4_0_PIT = 286,
	IMX_SC_R_M4_0_UART = 287,
	IMX_SC_R_M4_0_I2C = 288,
	IMX_SC_R_M4_0_INTMUX = 289,
	IMX_SC_R_M4_0_SIM = 290,
	IMX_SC_R_M4_0_WDOG = 291,
	IMX_SC_R_M4_0_MU_0B = 292,
	IMX_SC_R_M4_0_MU_0A0 = 293,
	IMX_SC_R_M4_0_MU_0A1 = 294,
	IMX_SC_R_M4_0_MU_0A2 = 295,
	IMX_SC_R_M4_0_MU_0A3 = 296,
	IMX_SC_R_M4_0_MU_1A = 297,
	IMX_SC_R_M4_1_PID0 = 298,
	IMX_SC_R_M4_1_PID1 = 299,
	IMX_SC_R_M4_1_PID2 = 300,
	IMX_SC_R_M4_1_PID3 = 301,
	IMX_SC_R_M4_1_PID4 = 302,
	IMX_SC_R_M4_1_RGPIO = 303,
	IMX_SC_R_M4_1_SEMA42 = 304,
	IMX_SC_R_M4_1_TPM = 305,
	IMX_SC_R_M4_1_PIT = 306,
	IMX_SC_R_M4_1_UART = 307,
	IMX_SC_R_M4_1_I2C = 308,
	IMX_SC_R_M4_1_INTMUX = 309,
	IMX_SC_R_M4_1_SIM = 310,
	IMX_SC_R_M4_1_WDOG = 311,
	IMX_SC_R_M4_1_MU_0B = 312,
	IMX_SC_R_M4_1_MU_0A0 = 313,
	IMX_SC_R_M4_1_MU_0A1 = 314,
	IMX_SC_R_M4_1_MU_0A2 = 315,
	IMX_SC_R_M4_1_MU_0A3 = 316,
	IMX_SC_R_M4_1_MU_1A = 317,
	IMX_SC_R_SAI_0 = 318,
	IMX_SC_R_SAI_1 = 319,
	IMX_SC_R_SAI_2 = 320,
	IMX_SC_R_IRQSTR_SCU2 = 321,
	IMX_SC_R_IRQSTR_DSP = 322,
	IMX_SC_R_UNUSED5 = 323,
	IMX_SC_R_UNUSED6 = 324,
	IMX_SC_R_AUDIO_PLL_0 = 325,
	IMX_SC_R_PI_0 = 326,
	IMX_SC_R_PI_0_PWM_0 = 327,
	IMX_SC_R_PI_0_PWM_1 = 328,
	IMX_SC_R_PI_0_I2C_0 = 329,
	IMX_SC_R_PI_0_PLL = 330,
	IMX_SC_R_PI_1 = 331,
	IMX_SC_R_PI_1_PWM_0 = 332,
	IMX_SC_R_PI_1_PWM_1 = 333,
	IMX_SC_R_PI_1_I2C_0 = 334,
	IMX_SC_R_PI_1_PLL = 335,
	IMX_SC_R_SC_PID0 = 336,
	IMX_SC_R_SC_PID1 = 337,
	IMX_SC_R_SC_PID2 = 338,
	IMX_SC_R_SC_PID3 = 339,
	IMX_SC_R_SC_PID4 = 340,
	IMX_SC_R_SC_SEMA42 = 341,
	IMX_SC_R_SC_TPM = 342,
	IMX_SC_R_SC_PIT = 343,
	IMX_SC_R_SC_UART = 344,
	IMX_SC_R_SC_I2C = 345,
	IMX_SC_R_SC_MU_0B = 346,
	IMX_SC_R_SC_MU_0A0 = 347,
	IMX_SC_R_SC_MU_0A1 = 348,
	IMX_SC_R_SC_MU_0A2 = 349,
	IMX_SC_R_SC_MU_0A3 = 350,
	IMX_SC_R_SC_MU_1A = 351,
	IMX_SC_R_SYSCNT_RD = 352,
	IMX_SC_R_SYSCNT_CMP = 353,
	IMX_SC_R_DEBUG = 354,
	IMX_SC_R_SYSTEM = 355,
	IMX_SC_R_SNVS = 356,
	IMX_SC_R_OTP = 357,
	IMX_SC_R_VPU_PID0 = 358,
	IMX_SC_R_VPU_PID1 = 359,
	IMX_SC_R_VPU_PID2 = 360,
	IMX_SC_R_VPU_PID3 = 361,
	IMX_SC_R_VPU_PID4 = 362,
	IMX_SC_R_VPU_PID5 = 363,
	IMX_SC_R_VPU_PID6 = 364,
	IMX_SC_R_VPU_PID7 = 365,
	IMX_SC_R_VPU_UART = 366,
	IMX_SC_R_VPUCORE = 367,
	IMX_SC_R_VPUCORE_0 = 368,
	IMX_SC_R_VPUCORE_1 = 369,
	IMX_SC_R_VPUCORE_2 = 370,
	IMX_SC_R_VPUCORE_3 = 371,
	IMX_SC_R_DMA_4_CH0 = 372,
	IMX_SC_R_DMA_4_CH1 = 373,
	IMX_SC_R_DMA_4_CH2 = 374,
	IMX_SC_R_DMA_4_CH3 = 375,
	IMX_SC_R_DMA_4_CH4 = 376,
	IMX_SC_R_ISI_CH0 = 377,
	IMX_SC_R_ISI_CH1 = 378,
	IMX_SC_R_ISI_CH2 = 379,
	IMX_SC_R_ISI_CH3 = 380,
	IMX_SC_R_ISI_CH4 = 381,
	IMX_SC_R_ISI_CH5 = 382,
	IMX_SC_R_ISI_CH6 = 383,
	IMX_SC_R_ISI_CH7 = 384,
	IMX_SC_R_MJPEG_DEC_S0 = 385,
	IMX_SC_R_MJPEG_DEC_S1 = 386,
	IMX_SC_R_MJPEG_DEC_S2 = 387,
	IMX_SC_R_MJPEG_DEC_S3 = 388,
	IMX_SC_R_MJPEG_ENC_S0 = 389,
	IMX_SC_R_MJPEG_ENC_S1 = 390,
	IMX_SC_R_MJPEG_ENC_S2 = 391,
	IMX_SC_R_MJPEG_ENC_S3 = 392,
	IMX_SC_R_MIPI_0 = 393,
	IMX_SC_R_MIPI_0_PWM_0 = 394,
	IMX_SC_R_MIPI_0_I2C_0 = 395,
	IMX_SC_R_MIPI_0_I2C_1 = 396,
	IMX_SC_R_MIPI_1 = 397,
	IMX_SC_R_MIPI_1_PWM_0 = 398,
	IMX_SC_R_MIPI_1_I2C_0 = 399,
	IMX_SC_R_MIPI_1_I2C_1 = 400,
	IMX_SC_R_CSI_0 = 401,
	IMX_SC_R_CSI_0_PWM_0 = 402,
	IMX_SC_R_CSI_0_I2C_0 = 403,
	IMX_SC_R_CSI_1 = 404,
	IMX_SC_R_CSI_1_PWM_0 = 405,
	IMX_SC_R_CSI_1_I2C_0 = 406,
	IMX_SC_R_HDMI = 407,
	IMX_SC_R_HDMI_I2S = 408,
	IMX_SC_R_HDMI_I2C_0 = 409,
	IMX_SC_R_HDMI_PLL_0 = 410,
	IMX_SC_R_HDMI_RX = 411,
	IMX_SC_R_HDMI_RX_BYPASS = 412,
	IMX_SC_R_HDMI_RX_I2C_0 = 413,
	IMX_SC_R_ASRC_0 = 414,
	IMX_SC_R_ESAI_0 = 415,
	IMX_SC_R_SPDIF_0 = 416,
	IMX_SC_R_SPDIF_1 = 417,
	IMX_SC_R_SAI_3 = 418,
	IMX_SC_R_SAI_4 = 419,
	IMX_SC_R_SAI_5 = 420,
	IMX_SC_R_GPT_5 = 421,
	IMX_SC_R_GPT_6 = 422,
	IMX_SC_R_GPT_7 = 423,
	IMX_SC_R_GPT_8 = 424,
	IMX_SC_R_GPT_9 = 425,
	IMX_SC_R_GPT_10 = 426,
	IMX_SC_R_DMA_2_CH5 = 427,
	IMX_SC_R_DMA_2_CH6 = 428,
	IMX_SC_R_DMA_2_CH7 = 429,
	IMX_SC_R_DMA_2_CH8 = 430,
	IMX_SC_R_DMA_2_CH9 = 431,
	IMX_SC_R_DMA_2_CH10 = 432,
	IMX_SC_R_DMA_2_CH11 = 433,
	IMX_SC_R_DMA_2_CH12 = 434,
	IMX_SC_R_DMA_2_CH13 = 435,
	IMX_SC_R_DMA_2_CH14 = 436,
	IMX_SC_R_DMA_2_CH15 = 437,
	IMX_SC_R_DMA_2_CH16 = 438,
	IMX_SC_R_DMA_2_CH17 = 439,
	IMX_SC_R_DMA_2_CH18 = 440,
	IMX_SC_R_DMA_2_CH19 = 441,
	IMX_SC_R_DMA_2_CH20 = 442,
	IMX_SC_R_DMA_2_CH21 = 443,
	IMX_SC_R_DMA_2_CH22 = 444,
	IMX_SC_R_DMA_2_CH23 = 445,
	IMX_SC_R_DMA_2_CH24 = 446,
	IMX_SC_R_DMA_2_CH25 = 447,
	IMX_SC_R_DMA_2_CH26 = 448,
	IMX_SC_R_DMA_2_CH27 = 449,
	IMX_SC_R_DMA_2_CH28 = 450,
	IMX_SC_R_DMA_2_CH29 = 451,
	IMX_SC_R_DMA_2_CH30 = 452,
	IMX_SC_R_DMA_2_CH31 = 453,
	IMX_SC_R_ASRC_1 = 454,
	IMX_SC_R_ESAI_1 = 455,
	IMX_SC_R_SAI_6 = 456,
	IMX_SC_R_SAI_7 = 457,
	IMX_SC_R_AMIX = 458,
	IMX_SC_R_MQS_0 = 459,
	IMX_SC_R_DMA_3_CH0 = 460,
	IMX_SC_R_DMA_3_CH1 = 461,
	IMX_SC_R_DMA_3_CH2 = 462,
	IMX_SC_R_DMA_3_CH3 = 463,
	IMX_SC_R_DMA_3_CH4 = 464,
	IMX_SC_R_DMA_3_CH5 = 465,
	IMX_SC_R_DMA_3_CH6 = 466,
	IMX_SC_R_DMA_3_CH7 = 467,
	IMX_SC_R_DMA_3_CH8 = 468,
	IMX_SC_R_DMA_3_CH9 = 469,
	IMX_SC_R_DMA_3_CH10 = 470,
	IMX_SC_R_DMA_3_CH11 = 471,
	IMX_SC_R_DMA_3_CH12 = 472,
	IMX_SC_R_DMA_3_CH13 = 473,
	IMX_SC_R_DMA_3_CH14 = 474,
	IMX_SC_R_DMA_3_CH15 = 475,
	IMX_SC_R_DMA_3_CH16 = 476,
	IMX_SC_R_DMA_3_CH17 = 477,
	IMX_SC_R_DMA_3_CH18 = 478,
	IMX_SC_R_DMA_3_CH19 = 479,
	IMX_SC_R_DMA_3_CH20 = 480,
	IMX_SC_R_DMA_3_CH21 = 481,
	IMX_SC_R_DMA_3_CH22 = 482,
	IMX_SC_R_DMA_3_CH23 = 483,
	IMX_SC_R_DMA_3_CH24 = 484,
	IMX_SC_R_DMA_3_CH25 = 485,
	IMX_SC_R_DMA_3_CH26 = 486,
	IMX_SC_R_DMA_3_CH27 = 487,
	IMX_SC_R_DMA_3_CH28 = 488,
	IMX_SC_R_DMA_3_CH29 = 489,
	IMX_SC_R_DMA_3_CH30 = 490,
	IMX_SC_R_DMA_3_CH31 = 491,
	IMX_SC_R_AUDIO_PLL_1 = 492,
	IMX_SC_R_AUDIO_CLK_0 = 493,
	IMX_SC_R_AUDIO_CLK_1 = 494,
	IMX_SC_R_MCLK_OUT_0 = 495,
	IMX_SC_R_MCLK_OUT_1 = 496,
	IMX_SC_R_PMIC_0 = 497,
	IMX_SC_R_PMIC_1 = 498,
	IMX_SC_R_SECO = 499,
	IMX_SC_R_CAAM_JR1 = 500,
	IMX_SC_R_CAAM_JR2 = 501,
	IMX_SC_R_CAAM_JR3 = 502,
	IMX_SC_R_SECO_MU_2 = 503,
	IMX_SC_R_SECO_MU_3 = 504,
	IMX_SC_R_SECO_MU_4 = 505,
	IMX_SC_R_HDMI_RX_PWM_0 = 506,
	IMX_SC_R_A35 = 507,
	IMX_SC_R_A35_0 = 508,
	IMX_SC_R_A35_1 = 509,
	IMX_SC_R_A35_2 = 510,
	IMX_SC_R_A35_3 = 511,
	IMX_SC_R_DSP = 512,
	IMX_SC_R_DSP_RAM = 513,
	IMX_SC_R_CAAM_JR1_OUT = 514,
	IMX_SC_R_CAAM_JR2_OUT = 515,
	IMX_SC_R_CAAM_JR3_OUT = 516,
	IMX_SC_R_VPU_DEC_0 = 517,
	IMX_SC_R_VPU_ENC_0 = 518,
	IMX_SC_R_CAAM_JR0 = 519,
	IMX_SC_R_CAAM_JR0_OUT = 520,
	IMX_SC_R_PMIC_2 = 521,
	IMX_SC_R_DBLOGIC = 522,
	IMX_SC_R_HDMI_PLL_1 = 523,
	IMX_SC_R_BOARD_R0 = 524,
	IMX_SC_R_BOARD_R1 = 525,
	IMX_SC_R_BOARD_R2 = 526,
	IMX_SC_R_BOARD_R3 = 527,
	IMX_SC_R_BOARD_R4 = 528,
	IMX_SC_R_BOARD_R5 = 529,
	IMX_SC_R_BOARD_R6 = 530,
	IMX_SC_R_BOARD_R7 = 531,
	IMX_SC_R_MJPEG_DEC_MP = 532,
	IMX_SC_R_MJPEG_ENC_MP = 533,
	IMX_SC_R_VPU_TS_0 = 534,
	IMX_SC_R_VPU_MU_0 = 535,
	IMX_SC_R_VPU_MU_1 = 536,
	IMX_SC_R_VPU_MU_2 = 537,
	IMX_SC_R_VPU_MU_3 = 538,
	IMX_SC_R_VPU_ENC_1 = 539,
	IMX_SC_R_VPU = 540,
	IMX_SC_R_LAST
};

/* NOTE - please add by replacing some of the UNUSED from above! */

/*
 * This type is used to indicate a control.
 */
enum imx_sc_ctrl {
	IMX_SC_C_TEMP = 0,
	IMX_SC_C_TEMP_HI = 1,
	IMX_SC_C_TEMP_LOW = 2,
	IMX_SC_C_PXL_LINK_MST1_ADDR = 3,
	IMX_SC_C_PXL_LINK_MST2_ADDR = 4,
	IMX_SC_C_PXL_LINK_MST_ENB = 5,
	IMX_SC_C_PXL_LINK_MST1_ENB = 6,
	IMX_SC_C_PXL_LINK_MST2_ENB = 7,
	IMX_SC_C_PXL_LINK_SLV1_ADDR = 8,
	IMX_SC_C_PXL_LINK_SLV2_ADDR = 9,
	IMX_SC_C_PXL_LINK_MST_VLD = 10,
	IMX_SC_C_PXL_LINK_MST1_VLD = 11,
	IMX_SC_C_PXL_LINK_MST2_VLD = 12,
	IMX_SC_C_SINGLE_MODE = 13,
	IMX_SC_C_ID = 14,
	IMX_SC_C_PXL_CLK_POLARITY = 15,
	IMX_SC_C_LINESTATE = 16,
	IMX_SC_C_PCIE_G_RST = 17,
	IMX_SC_C_PCIE_BUTTON_RST = 18,
	IMX_SC_C_PCIE_PERST = 19,
	IMX_SC_C_PHY_RESET = 20,
	IMX_SC_C_PXL_LINK_RATE_CORRECTION = 21,
	IMX_SC_C_PANIC = 22,
	IMX_SC_C_PRIORITY_GROUP = 23,
	IMX_SC_C_TXCLK = 24,
	IMX_SC_C_CLKDIV = 25,
	IMX_SC_C_DISABLE_50 = 26,
	IMX_SC_C_DISABLE_125 = 27,
	IMX_SC_C_SEL_125 = 28,
	IMX_SC_C_MODE = 29,
	IMX_SC_C_SYNC_CTRL0 = 30,
	IMX_SC_C_KACHUNK_CNT = 31,
	IMX_SC_C_KACHUNK_SEL = 32,
	IMX_SC_C_SYNC_CTRL1 = 33,
	IMX_SC_C_DPI_RESET = 34,
	IMX_SC_C_MIPI_RESET = 35,
	IMX_SC_C_DUAL_MODE = 36,
	IMX_SC_C_VOLTAGE = 37,
	IMX_SC_C_PXL_LINK_SEL = 38,
	IMX_SC_C_OFS_SEL = 39,
	IMX_SC_C_OFS_AUDIO = 40,
	IMX_SC_C_OFS_PERIPH = 41,
	IMX_SC_C_OFS_IRQ = 42,
	IMX_SC_C_RST0 = 43,
	IMX_SC_C_RST1 = 44,
	IMX_SC_C_SEL0 = 45,
	IMX_SC_C_LAST
};

#endif /* _SC_TYPES_H */
