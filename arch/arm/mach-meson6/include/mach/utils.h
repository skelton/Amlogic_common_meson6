/*
 *
 * arch/arm/mach-meson3/include/mach/am_regs.h
 *
 *  Copyright (C) 2011 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Basic register address definitions in physical memory and
 * some block defintions for core devices like the timer.
 */

#ifndef __MACH_MESSON6_UTILS_H
#define __MACH_MESSON6_UTILS_H
/**
 * Some driver use different name or set a alias name of MACRO
 * Please put that into this file
 */


/**
 * UART Section
 */

#define UART_AO    0
#define UART_A     1
#define UART_B     2
#define UART_C     3
#define UART_D     4

#define MAX_PORT_NUM 5
#define UART_FIFO_CNTS	64,128,64,64,64
#define UART_ADDRS      ((void *)P_AO_UART_WFIFO),((void *)P_UART0_WFIFO), \
						((void *)P_UART1_WFIFO),((void *)P_UART2_WFIFO),   \
						((void *)P_UART3_WFIFO)
#define UART_INTS		INT_UART_AO, INT_UART_0, INT_UART_1, INT_UART_2    \
						, INT_UART_3

/*canvas MACRO */

#define CANVAS_ADDR_LMASK       0x1fffffff
#define CANVAS_WIDTH_LMASK      0x7
#define CANVAS_WIDTH_LWID       3
#define CANVAS_WIDTH_LBIT       29
//#define DC_CAV_LUT_DATAH          0x1328
#define CANVAS_WIDTH_HMASK      0x1ff
#define CANVAS_WIDTH_HBIT       0
#define CANVAS_HEIGHT_MASK      0x1fff
#define CANVAS_HEIGHT_BIT       9
#define CANVAS_YWRAP            (1<<23)
#define CANVAS_XWRAP            (1<<22)
#define CANVAS_ADDR_NOWRAP      0x00
#define CANVAS_ADDR_WRAPX       0x01
#define CANVAS_ADDR_WRAPY       0x02
#define CANVAS_BLKMODE_MASK     3
#define CANVAS_BLKMODE_BIT      24
#define CANVAS_BLKMODE_LINEAR   0x00
#define CANVAS_BLKMODE_32X32    0x01
#define CANVAS_BLKMODE_64X32    0x02
//#define DC_CAV_LUT_ADDR           0x132c
#define CANVAS_LUT_INDEX_BIT    0
#define CANVAS_LUT_INDEX_MASK   0x7
#define CANVAS_LUT_WR_EN        (0x2 << 8)
#define CANVAS_LUT_RD_EN        (0x1 << 8)
//#define DC_CAV_LVL3_MODE          0x1330
/*OSD relative MACRO*/
// Bit 28   color management enable
// Bit 27,  if true, vd2 use viu2 output as the input, otherwise use normal vd2 from memory 
// Bit 26:18, vd2 alpha
// Bit 17, osd2 enable for preblend
// Bit 16, osd1 enable for preblend
// Bit 15, vd2 enable for preblend
// Bit 14, vd1 enable for preblend
// Bit 13, osd2 enable for postblend
// Bit 12, osd1 enable for postblend
// Bit 11, vd2 enable for postblend
// Bit 10, vd1 enable for postblend
// Bit 9,  if true, osd1 is alpha premultipiled 
// Bit 8,  if true, osd2 is alpha premultipiled 
// Bit 7,  postblend module enable
// Bit 6,  preblend module enable
// Bit 5,  if true, osd2 foreground compared with osd1 in preblend
// Bit 4,  if true, osd2 foreground compared with osd1 in postblend
// Bit 3,  
// Bit 2,  if true, disable resetting async fifo every vsync, otherwise every vsync
//			 the aync fifo will be reseted.
// Bit 1,	  
// Bit 0	if true, the output result of VPP is saturated
    #define VPP_VD2_ALPHA_WID           9
    #define VPP_VD2_ALPHA_MASK          0x1ff
    #define VPP_VD2_ALPHA_BIT           18
    #define VPP_OSD2_PREBLEND           (1 << 17)
    #define VPP_OSD1_PREBLEND           (1 << 16)
    #define VPP_VD2_PREBLEND            (1 << 15)
    #define VPP_VD1_PREBLEND            (1 << 14)
    #define VPP_OSD2_POSTBLEND          (1 << 13)
    #define VPP_OSD1_POSTBLEND          (1 << 12)
    #define VPP_VD2_POSTBLEND           (1 << 11)
    #define VPP_VD1_POSTBLEND           (1 << 10)
    #define VPP_OSD1_ALPHA              (1 << 9)
    #define VPP_OSD2_ALPHA              (1 << 8)
    #define VPP_POSTBLEND_EN            (1 << 7)
    #define VPP_PREBLEND_EN             (1 << 6)
    #define VPP_PRE_FG_SEL_MASK         (1 << 5)
    #define VPP_PRE_FG_OSD2             (1 << 5)
    #define VPP_PRE_FG_OSD1             (0 << 5)
    #define VPP_POST_FG_SEL_MASK        (1 << 4)
    #define VPP_POST_FG_OSD2            (1 << 4)
    #define VPP_POST_FG_OSD1            (0 << 4)
    #define VPP_FIFO_RESET_DE           (1 << 2)
    #define VPP_OUT_SATURATE            (1 << 0)
/*tv relative part*/
#define VFIFO2VD_CTL                          0x1b58
#define VFIFO2VD_PIXEL_START                  0x1b59
#define VFIFO2VD_PIXEL_END                    0x1b5a
#define VFIFO2VD_LINE_TOP_START               0x1b5b
#define VFIFO2VD_LINE_TOP_END                 0x1b5c
#define VFIFO2VD_LINE_BOT_START               0x1b5d
#define VFIFO2VD_LINE_BOT_END                 0x1b5e

/* AUDIO */
	#define SPDIF_EN                31
    #define SPDIF_INT_EN            30
    #define SPDIF_BURST_PRE_INT_EN  29
    #define SPDIF_TIE_0             24
    #define SPDIF_SAMPLE_SEL        23
    #define SPDIF_REVERSE_EN        22
    #define SPDIF_BIT_ORDER         20
    #define SPDIF_CHNL_ORDER        19
    #define SPDIF_DATA_TYPE_SEL     18
    #define SPDIF_XTDCLK_UPD_ITVL   14   //16:14
    #define SPDIF_CLKNUM_54U        0     //13:0 
    
	#define SPDIF_CLKNUM_192K  24     //29:24 
    #define SPDIF_CLKNUM_96K   18     //23:18 
    #define SPDIF_CLKNUM_48K   12     //17:12 
    #define SPDIF_CLKNUM_44K   6     // 11:6
    #define SPDIF_CLKNUM_32K   0     // 5:0
    
    #define I2SIN_DIR       0    // I2S CLK and LRCLK direction. 0 : input 1 : output.
    #define I2SIN_CLK_SEL    1    // I2S clk selection : 0 : from pad input. 1 : from AIU.
    #define I2SIN_LRCLK_SEL 2
    #define I2SIN_POS_SYNC  3
    #define I2SIN_LRCLK_SKEW 4    // 6:4
    #define I2SIN_LRCLK_INVT 7
    #define I2SIN_SIZE       8    //9:8 : 0 16 bit. 1 : 18 bits 2 : 20 bits 3 : 24bits.
    #define I2SIN_CHAN_EN   10    //13:10. 
    #define I2SIN_EN        15
    
    #define AUDIN_FIFO0_EN       0
    #define AUDIN_FIFO0_RST      1
    #define AUDIN_FIFO0_LOAD     2    //write 1 to load address to AUDIN_FIFO0.
         
    #define AUDIN_FIFO0_DIN_SEL  3
            // 0     spdifIN
            // 1     i2Sin
            // 2     PCMIN
            // 3     HDMI in
            // 4     DEMODULATOR IN
    #define AUDIN_FIFO0_ENDIAN   8    //10:8   data endian control.
    #define AUDIN_FIFO0_CHAN     11    //14:11   channel number.  in M1 suppose there's only 1 channel and 2 channel.
    #define AUDIN_FIFO0_UG       15    // urgent request enable.
    #define AUDIN_FIFO0_HOLD0_EN  19  
    #define AUDIN_FIFO0_HOLD1_EN  20
    #define AUDIN_FIFO0_HOLD2_EN  21
    #define AUDIN_FIFO0_HOLD0_SEL 22   // 23:22
    #define AUDIN_FIFO0_HOLD1_SEL 24   // 25:24
    #define AUDIN_FIFO0_HOLD2_SEL 26   // 27:26
    #define AUDIN_FIFO0_HOLD_LVL  28   // 27:26
    
    #define AUDIN_FIFO1_EN       0
    #define AUDIN_FIFO1_RST      1
    #define AUDIN_FIFO1_LOAD     2    //write 1 to load address to AUDIN_FIFO0.
         
    #define AUDIN_FIFO1_DIN_SEL  3
            // 0     spdifIN
            // 1     i2Sin
            // 2     PCMIN
            // 3     HDMI in
            // 4     DEMODULATOR IN
    #define AUDIN_FIFO1_ENDIAN   8    //10:8   data endian control.
    #define AUDIN_FIFO1_CHAN     11    //14:11   channel number.  in M1 suppose there's only 1 channel and 2 channel.
    #define AUDIN_FIFO1_UG       15    // urgent request enable.
    #define AUDIN_FIFO1_HOLD0_EN  19  
    #define AUDIN_FIFO1_HOLD1_EN  20
    #define AUDIN_FIFO1_HOLD2_EN  21
    #define AUDIN_FIFO1_HOLD0_SEL 22   // 23:22
    #define AUDIN_FIFO1_HOLD1_SEL 24   // 25:24
    #define AUDIN_FIFO1_HOLD2_SEL 26   // 27:26
    #define AUDIN_FIFO1_HOLD_LVL  28   // 27:26
    

/*BT656 MACRO */
//#define BT_CTRL 0x2240 	///../ucode/register.h
    #define BT_SYSCLOCK_RESET    30      //Sync fifo soft  reset_n at system clock domain.     Level reset. 0 = reset. 1 : normal mode.
    #define BT_656CLOCK_RESET    29      //Sync fifo soft reset_n at bt656 clock domain.   Level reset.  0 = reset.  1 : normal mode.
    #define BT_VSYNC_SEL              25      //25:26 VDIN VS selection.   00 :  SOF.  01: EOF.   10: vbi start point.  11 : vbi end point.
    #define BT_HSYNC_SEL              23      //24:23 VDIN HS selection.  00 : EAV.  01: SAV.    10:  EOL.  11: SOL
    #define BT_CAMERA_MODE        22      // Camera_mode
    #define BT_CLOCK_ENABLE        7	// 1: enable bt656 clock. 0: disable bt656 clock.

//#define BT_PORT_CTRL 0x2249 	///../ucode/register.h
    #define BT_VSYNC_MODE      23  //1: use  vsync  as the VBI start point. 0: use the regular vref.
    #define BT_HSYNC_MODE      22  //1: use hsync as the active video start point.  0. Use regular sav and eav. 
    #define BT_SOFT_RESET           31	// Soft reset
    #define BT_JPEG_START           30
    #define BT_JPEG_IGNORE_BYTES    18	//20:18
    #define BT_JPEG_IGNORE_LAST     17
    #define BT_UPDATE_ST_SEL        16
    #define BT_COLOR_REPEAT         15
    #define BT_VIDEO_MODE           13	// 14:13
    #define BT_AUTO_FMT             12
    #define BT_PROG_MODE            11
    #define BT_JPEG_MODE            10
    #define BT_XCLK27_EN_BIT        9	// 1 : xclk27 is input.     0 : xclk27 is output.
    #define BT_FID_EN_BIT           8	// 1 : enable use FID port.
    #define BT_CLK27_SEL_BIT        7	// 1 : external xclk27      0 : internal clk27.
    #define BT_CLK27_PHASE_BIT      6	// 1 : no inverted          0 : inverted.
    #define BT_ACE_MODE_BIT         5	// 1 : auto cover error by hardware.
    #define BT_SLICE_MODE_BIT       4	// 1 : no ancillay flag     0 : with ancillay flag.
    #define BT_FMT_MODE_BIT         3	// 1 : ntsc                 0 : pal.
    #define BT_REF_MODE_BIT         2	// 1 : from bit stream.     0 : from ports.
    #define BT_MODE_BIT             1	// 1 : BT656 model          0 : SAA7118 mode.
    #define BT_EN_BIT               0	// 1 : enable.
    #define BT_VSYNC_PHASE      0
    #define BT_HSYNC_PHASE      1
    #define BT_VSYNC_PULSE      2
    #define BT_HSYNC_PULSE      3
    #define BT_FID_PHASE        4
    #define BT_FID_HSVS         5
    #define BT_IDQ_EN           6
    #define BT_IDQ_PHASE        7
    #define BT_D8B              8
    #define BT_10BTO8B          9
    #define BT_FID_DELAY       10	//12:10
    #define BT_VSYNC_DELAY     13	//
    #define BT_HSYNC_DELAY     16
/***
 * HDMI
 */
#define  P_HDMI_CNTL_PORT P_HDMI_CTRL_PORT
/**
 * LVDS
 */
   #define     LVDS_blank_data_reserved 30  // 31:30
   #define     LVDS_blank_data_r        20  // 29:20
   #define     LVDS_blank_data_g        10  // 19:10
   #define     LVDS_blank_data_b         0  //  9:0
   #define     LVDS_USE_TCON    7
   #define     LVDS_DUAL        6
   #define     PN_SWP           5
   #define     LSB_FIRST        4
   #define     LVDS_RESV        3
   #define     ODD_EVEN_SWP     2
   #define     LVDS_REPACK      0


#endif //__MACH_MESSON3_REGS_H
