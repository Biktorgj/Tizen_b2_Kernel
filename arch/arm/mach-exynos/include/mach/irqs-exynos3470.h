/*
 * Copyright (c) 2010-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - IRQ definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define EXYNOS4_NO_COMBINER
/* BLK_CPU */
#define EXYNOS4_IRQ_PMU_CPU0		IRQ_SPI(18)
#define EXYNOS4_IRQ_PMU_CPU1		IRQ_SPI(19)
#define EXYNOS4_IRQ_PMU_CPU2		IRQ_SPI(20)
#define EXYNOS4_IRQ_PMU_CPU3		IRQ_SPI(21)
#define EXYNOS4_IRQ_CTI_CPU0		IRQ_SPI(22)
#define EXYNOS4_IRQ_CTI_CPU1		IRQ_SPI(23)
#define EXYNOS4_IRQ_CTI_CPU2		IRQ_SPI(24)
#define EXYNOS4_IRQ_CTI_CPU3		IRQ_SPI(25)
#define EXYNOS4_IRQ_AXI_ERR		IRQ_SPI(26)
/* External Interrupt */
#define EXYNOS4_IRQ_EINT0		IRQ_SPI(32)
#define EXYNOS4_IRQ_EINT1		IRQ_SPI(33)
#define EXYNOS4_IRQ_EINT2		IRQ_SPI(34)
#define EXYNOS4_IRQ_EINT3		IRQ_SPI(35)
#define EXYNOS4_IRQ_EINT4		IRQ_SPI(36)
#define EXYNOS4_IRQ_EINT5		IRQ_SPI(37)
#define EXYNOS4_IRQ_EINT6		IRQ_SPI(38)
#define EXYNOS4_IRQ_EINT7		IRQ_SPI(39)
#define EXYNOS4_IRQ_EINT8		IRQ_SPI(40)
#define EXYNOS4_IRQ_EINT9		IRQ_SPI(41)
#define EXYNOS4_IRQ_EINT10		IRQ_SPI(42)
#define EXYNOS4_IRQ_EINT11		IRQ_SPI(43)
#define EXYNOS4_IRQ_EINT12		IRQ_SPI(44)
#define EXYNOS4_IRQ_EINT13		IRQ_SPI(45)
#define EXYNOS4_IRQ_EINT14		IRQ_SPI(46)
#define EXYNOS4_IRQ_EINT15		IRQ_SPI(47)
#define EXYNOS_IRQ_EINT16_31		IRQ_SPI(48)
/* BLK_ALIVE */
#define EXYNOS4_IRQ_CEC			IRQ_SPI(49)
#define EXYNOS4_IRQ_PMU			IRQ_SPI(50)
/* BLK_CP */
#define EXYNOS3470_IRQ_CP_WDT		IRQ_SPI(52)
#define EXYNOS3470_IRQ_CP_ACTIVE	IRQ_SPI(53)
/* BLK_MIF */
#define EXYNOS4_IRQ_SYSMMU_SSS_0	IRQ_SPI(60)
#define EXYNOS4_IRQ_SYSMMU_2D_0		IRQ_SPI(61)
#define EXYNOS4_IRQ_SYSMMU_SSS_1	IRQ_SPI(62)
#define EXYNOS4_IRQ_SYSMMU_2D_1		IRQ_SPI(63)
#define EXYNOS4_IRQ_MCU_IPC		IRQ_SPI(72)
#define EXYNOS4_IRQ_RTC_ALARM		IRQ_SPI(73)
#define EXYNOS4_IRQ_RTC_TIC		IRQ_SPI(74)
#define EXYNOS4_IRQ_INTFEEDCTRL_SSS	IRQ_SPI(75)
#define EXYNOS4_IRQ_2D			IRQ_SPI(76)
/* BLK_LCD */
#define EXYNOS4_IRQ_SYSMMU_LCD0_M0_0	IRQ_SPI(80)
#define EXYNOS4_IRQ_SYSMMU_LCD1_M1_0	IRQ_SPI(81)
#define EXYNOS4_IRQ_MIPIDSI0		IRQ_SPI(83)
#define EXYNOS4_IRQ_FIMD0_FIFO		IRQ_SPI(84)
#define EXYNOS4_IRQ_FIMD0_VSYNC		IRQ_SPI(85)
#define EXYNOS4_IRQ_FIMD0_SYSTEM	IRQ_SPI(86)
/* BLK_TV */
#define EXYNOS4_IRQ_SYSMMU_TV_M0_0	IRQ_SPI(88)
#define EXYNOS4_IRQ_MIXER		IRQ_SPI(91)
#define EXYNOS4_IRQ_HDMI		IRQ_SPI(92)
/* BLK_MFC */
#define EXYNOS4_IRQ_SYSMMU_MFC_M0_0	IRQ_SPI(96)
#define EXYNOS4_IRQ_SYSMMU_MFC_M1_0	IRQ_SPI(97)
#define EXYNOS4_IRQ_MFC			IRQ_SPI(102)
/* BLK_PERIL */
#define EXYNOS4_IRQ_TIMER0_VIC		IRQ_SPI(104)
#define EXYNOS4_IRQ_TIMER1_VIC		IRQ_SPI(105)
#define EXYNOS4_IRQ_TIMER2_VIC		IRQ_SPI(106)
#define EXYNOS4_IRQ_TIMER3_VIC		IRQ_SPI(107)
#define EXYNOS4_IRQ_TIMER4_VIC		IRQ_SPI(108)
#define EXYNOS4_IRQ_UART0		IRQ_SPI(109)
#define EXYNOS4_IRQ_UART1		IRQ_SPI(110)
#define EXYNOS4_IRQ_UART2		IRQ_SPI(111)
#define EXYNOS4_IRQ_UART3		IRQ_SPI(112)
#define EXYNOS4_IRQ_IIC			IRQ_SPI(113)
#define EXYNOS4_IRQ_IIC1		IRQ_SPI(114)
#define EXYNOS4_IRQ_IIC2		IRQ_SPI(115)
#define EXYNOS4_IRQ_IIC3		IRQ_SPI(116)
#define EXYNOS4_IRQ_IIC4		IRQ_SPI(117)
#define EXYNOS4_IRQ_IIC5		IRQ_SPI(118)
#define EXYNOS4_IRQ_IIC6		IRQ_SPI(119)
#define EXYNOS4_IRQ_IIC7		IRQ_SPI(120)
#define EXYNOS4_IRQ_SPI0		IRQ_SPI(121)
#define EXYNOS4_IRQ_SPI1		IRQ_SPI(122)
#define EXYNOS4_IRQ_SPI2		IRQ_SPI(123)
#define EXYNOS4_IRQ_I2S0		IRQ_SPI(124)
#define EXYNOS4_IRQ_I2S1		IRQ_SPI(125)
#define EXYNOS4_IRQ_PCM0		IRQ_SPI(128)
#define EXYNOS4_IRQ_PCM1		IRQ_SPI(129)
#define EXYNOS4_IRQ_PCM2		IRQ_SPI(130)
#define EXYNOS4_IRQ_SPDIF		IRQ_SPI(131)
#define EXYNOS4X12_IRQ_ADC0		IRQ_SPI(137)
#define EXYNOS4_IRQ_PDMA0		IRQ_SPI(138)
#define EXYNOS4_IRQ_PDMA1		IRQ_SPI(139)
#define EXYNOS4_IRQ_USB_HOST		IRQ_SPI(140)
#define EXYNOS4_IRQ_OTG			IRQ_SPI(141)
#define EXYNOS4_IRQ_HSMMC0		IRQ_SPI(142)
#define EXYNOS4_IRQ_HSMMC1		IRQ_SPI(143)
#define EXYNOS4_IRQ_HSMMC2		IRQ_SPI(144)
#define EXYNOS4_IRQ_TSI			IRQ_SPI(146)
#define EXYNOS4_IRQ_UFS			IRQ_SPI(147)
/* BLK_CAM */
#define EXYNOS4_IRQ_SYSMMU_FIMC0_0	IRQ_SPI(152)
#define EXYNOS4_IRQ_SYSMMU_FIMC1_0	IRQ_SPI(153)
#define EXYNOS4_IRQ_SYSMMU_FIMC2_0	IRQ_SPI(154)
#define EXYNOS4_IRQ_SYSMMU_FIMC3_0	IRQ_SPI(155)
#define EXYNOS4_IRQ_SYSMMU_JPEG_0	IRQ_SPI(156)
#define EXYNOS4_IRQ_SYSMMU_FIMC_LITE2_0	IRQ_SPI(163)
#define EXYNOS4_IRQ_MIPI_CSIS0		IRQ_SPI(165)
#define EXYNOS4_IRQ_MIPI_CSIS1		IRQ_SPI(166)
#define EXYNOS4_IRQ_FIMC0		IRQ_SPI(167)
#define EXYNOS4_IRQ_FIMC1		IRQ_SPI(168)
#define EXYNOS4_IRQ_FIMC2		IRQ_SPI(169)
#define EXYNOS4_IRQ_FIMC3		IRQ_SPI(170)
#define EXYNOS4_IRQ_JPEG		IRQ_SPI(171)
/* BLK_G3D */
#define EXYNOS4_IRQ_PMU_3D		IRQ_SPI(177)
#define EXYNOS4_IRQ_PPMMU0_3D		IRQ_SPI(178)
#define EXYNOS4_IRQ_PPMMU1_3D		IRQ_SPI(179)
#define EXYNOS4_IRQ_PPMMU2_3D		IRQ_SPI(180)
#define EXYNOS4_IRQ_PPMMU3_3D		IRQ_SPI(181)
#define EXYNOS4_IRQ_GPMMU_3D		IRQ_SPI(182)
#define EXYNOS4_IRQ_PP0_3D		IRQ_SPI(183)
#define EXYNOS4_IRQ_PP1_3D		IRQ_SPI(184)
#define EXYNOS4_IRQ_PP2_3D		IRQ_SPI(185)
#define EXYNOS4_IRQ_PP3_3D		IRQ_SPI(186)
#define EXYNOS4_IRQ_GP_3D		IRQ_SPI(187)
/* BLK_ISP */
#define EXYNOS4_IRQ_ARMISP_GIC		IRQ_SPI(196)
#define EXYNOS4_IRQ_ISP_GIC		IRQ_SPI(197)
#define EXYNOS4_IRQ_FIMC_LITE0		IRQ_SPI(198)
#define EXYNOS4_IRQ_FIMC_LITE1		IRQ_SPI(199)
#define EXYNOS4_IRQ_SYSMMU_FIMC_LITE0_0	IRQ_SPI(200)
#define EXYNOS4_IRQ_SYSMMU_FIMC_LITE1_0	IRQ_SPI(201)
#define EXYNOS4_IRQ_SYSMMU_FIMC_ISP_0	IRQ_SPI(202)
#define EXYNOS4_IRQ_SYSMMU_FIMC_DRC_0	IRQ_SPI(203)
#define EXYNOS4_IRQ_SYSMMU_FIMC_FD_0	IRQ_SPI(204)
#define EXYNOS4_IRQ_SYSMMU_FIMC_CX_0	IRQ_SPI(205)
#define EXYNOS4_IRQ_SYSMMU_FIMC_SCALERP_0	IRQ_SPI(206)
#define EXYNOS4_IRQ_SYSMMU_FIMC_SCALERC_0	IRQ_SPI(207)
#define EXYNOS4_IRQ_TMU			IRQ_SPI(216)
/* BLK_PERIR */
#define EXYNOS4_IRQ_MCT_G0		IRQ_SPI(218)
#define EXYNOS4_IRQ_MCT_L0		IRQ_SPI(223)
#define EXYNOS4_IRQ_WDT			IRQ_SPI(224)
#define EXYNOS4_IRQ_GPIO_XB		IRQ_SPI(225)
#define EXYNOS4_IRQ_MCT_L1		IRQ_SPI(226)
#define EXYNOS4_IRQ_MCT_L2		IRQ_SPI(227)
#define EXYNOS4_IRQ_MCT_L3		IRQ_SPI(228)
#define EXYNOS4_IRQ_KEYPAD		IRQ_SPI(229)
#define EXYNOS4_IRQ_SYSMMU_MDMA0_0	IRQ_SPI(232)
#define EXYNOS4_IRQ_SYSMMU_ROTATOR_0	IRQ_SPI(233)
#define EXYNOS4_IRQ_ROTATOR		IRQ_SPI(236)
#define EXYNOS4_IRQ_MDMA1		IRQ_SPI(239)
#define EXYNOS4_IRQ_GPIO_XA		IRQ_SPI(240)
#define EXYNOS4_IRQ_AUDIO_SS		IRQ_SPI(241)

#define EXYNOS4_IRQ_MDMA0		IRQ_SPI(33)
#define EXYNOS4_IRQ_COMBINER_G19	IRQ_SPI(42)
#define EXYNOS4_IRQ_COMBINER_G18	IRQ_SPI(48)
#define EXYNOS4_IRQ_UART4		IRQ_SPI(56)
#define EXYNOS4_IRQ_HSMMC3		IRQ_SPI(76)
#define EXYNOS4_IRQ_DWMCI		IRQ_SPI(77)
#define EXYNOS4_IRQ_ONENAND_AUDI	IRQ_SPI(82)
#define EXYNOS4_IRQ_PCIE		IRQ_SPI(90)
#define EXYNOS4_IRQ_IIC_HDMIPHY		IRQ_SPI(93)
#define EXYNOS4_IRQ_SDO			IRQ_SPI(95)
#define EXYNOS4_IRQ_I2S2		IRQ_SPI(99)
#define EXYNOS4_IRQ_AC97		IRQ_SPI(100)
#define EXYNOS4_IRQ_ADC0		IRQ_SPI(105)
#define EXYNOS4_IRQ_PEN0		IRQ_SPI(106)
#define EXYNOS4_IRQ_ADC1		IRQ_SPI(107)
#define EXYNOS4_IRQ_COMBINER_G16	IRQ_SPI(107)
#define EXYNOS4_IRQ_PEN1		IRQ_SPI(108)
#define EXYNOS4_IRQ_COMBINER_G17	IRQ_SPI(108)
#define EXYNOS4_IRQ_GPS			IRQ_SPI(111)
#define EXYNOS4_IRQ_SLIMBUS		IRQ_SPI(113)
#define EXYNOS4_IRQ_SATA		IRQ_SPI(116)
