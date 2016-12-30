/*******************************************************************************
 * File Name: ov9712.h
 * Author: Matthew Fonken
 ******************************************************************************/

#ifndef _OV9712_H    /* Guard against multiple inclusion */
#define _OV9712_H
    
#define OV9712_ADDR  0x60
   
//<editor-fold defaultstate="collapsed" desc="OV9712 Registers">  
#define GAIN		 0x00
#define BLUE		 0x01
#define RED          0x02
#define REG03		 0x03
#define REG04		 0x04
#define GREEN		 0x05
#define BRL          0x06
#define GL           0x07
#define REG08		 0x08
#define COM2		 0x09
#define PIDH		 0x0a
#define PIDL		 0x0b
#define COM3		 0x0c
#define COM4		 0x0d
#define COM5		 0x0e
#define COM6		 0x0f
#define AEC          0x10
#define CLK          0x11
#define COM7		 0x12
#define COM8		 0x13
#define COM9		 0x14
#define COM10		 0x15
#define AECH		 0x16
#define HSTART		 0x17
#define AHSIZE		 0x18
#define VSTART		 0x19
#define AVSIZE		 0x1a
#define PSHIFT		 0x1b
#define MIDH		 0x1c
#define MIDL		 0x1d
#define COM11		 0x1e
#define REG1F		 0x1f
#define COM12		 0x20
#define COM13		 0x21
#define COM14		 0x22
#define COM15		 0x23
#define WPT          0x24
#define BPT          0x25
#define VPT          0x26
#define COM16		 0x27
#define COM17		 0x28
#define COM18		 0x29
#define REG2A		 0x2a
#define REG2B		 0x2b
#define COM19		 0x2c
#define REG2D		 0x2d
#define REG2E		 0x2e
#define YAVG		 0x2f
#define REG30		 0x30
#define REG31		 0x31
#define REG32		 0x32
#define COM24		 0x37
#define COM25		 0x38
#define COM28		 0x3c
#define RENDL		 0x3d
#define RENDH		 0x3e
#define REG41		 0x41
#define REG42		 0x42
#define REG43		 0x43
#define ROFFS		 0x44
#define BOFFS		 0x45
#define GROFFS		 0x46
#define GBOFFS		 0x47
#define HOFFS		 0x48
#define REG49		 0x49
#define REG4A		 0x4a
#define REG4E		 0x4e
#define REG4F		 0x4f
#define REG50		 0x50
#define REG51		 0x51
#define REG55		 0x55
#define REG56		 0x56
#define REG57		 0x57
#define REG58		 0x58
#define REG59		 0x59
#define REG5C		 0x5c
#define REG5D		 0x5d
#define REG72		 0x72
#define REG73		 0x73
#define REG74		 0x74
#define REG75		 0x75
#define REG76		 0x76
#define REG77		 0x77
#define REG78		 0x78
#define REG79		 0x79
#define DSP_CTRL_0 	 0x96
#define DSP_CTRL_1 	 0x97
#define DSP_CTRL_2 	 0x98
#define DSP_CTRL_3 	 0x99
#define DSP_CTRL_4 	 0x9a
#define DSP_CTRL_5 	 0x9b
#define DSP_CTRL_6	 0x9c
#define DSP_CTRL_7	 0x9d
#define LENC_CTRL_0	 0x9e
#define LENC_CTRL_1	 0x9f
#define LENC_CTRL_2	 0xa0
#define LENC_CTRL_3	 0xa1
#define LENC_CTRL_4	 0xa2
#define LENC_CTRL_5	 0xa3
#define LENC_CTRL_6	 0xa4
#define LENC_CTRL_7	 0xa5
#define LENC_CTRL_8	 0xa6
#define LENC_CTRL_9	 0xa7
#define LENC_CTRL_10 0xa8
#define LENC_CTRL_11 0xa9
#define LENC_CTRL_12 0xaa
#define LENC_CTRL_13 0xab
#define LENC_CTRL_14 0xac
#define LENC_CTRL_15 0xad
#define LENC_CTRL_16 0xae
#define LENC_CTRL_17 0xaf
#define LENC_CTRL_18 0xb0
#define LENC_CTRL_19 0xb1
#define LENC_CTRL_20 0xb2
#define LENC_CTRL_21 0xb3
#define LENC_CTRL_22 0xb4
#define LENC_CTRL_23 0xb5
#define AWB_CTRL_0	 0xb6
#define AWB_CTRL_1	 0xb7
#define AWB_CTRL_2	 0xb8
#define AWB_CTRL_3	 0xb9
#define AWB_CTRL_4	 0xba
#define AWB_CTRL_5	 0xbb
#define WBC_CTRL_0	 0xbc
#define YAVG_CTRL_0	 0xbd
#define YAVG_CTRL_1	 0xbe
#define YAVG_CTRL_2	 0xbf
#define YAVG_CTRL_3	 0xc0
#define YAVG_CTRL_4	 0xc1
#define DVP_CTRL_00	 0xc2
#define DVP_CTRL_01	 0xc3
#define DVP_CTRL_02	 0xc4
#define DVP_CTRL_03	 0xc5
#define DVP_CTRL_04	 0xc6
#define DVP_CTRL_05	 0xc7
#define DVP_CTRL_06	 0xc8
#define DVP_CTRL_07	 0xc9
#define DVP_CTRL_08	 0xca
#define DVP_CTRL_09	 0xcb
#define DVP_CTRL_0a	 0xcc
#define DVP_CTRL_0b	 0xcd
#define DVP_CTRL_0c	 0xce
#define DVP_CTRL_0d	 0xcf
#define DVP_CTRL_0e	 0xd0
#define DVP_CTRL_0f	 0xd1
#define DVP_CTRL_10	 0xd2
#define DVP_CTRL_11	 0xd3
#define DVP_CTRL_12	 0xd4
#define SC_CTRL_0	 0xd6
#define ENDR		 0xff
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Important Terms">  
/*	TERMS *
* AEC - Auto Exposure Control
* AGC - Auto Gain Control
* BLC - Black level Calibration
**/
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Type Definitions">
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef struct _cam_register_t
{
    uint8_t id;
    uint8_t value;
} cam_register_t;
     
cam_register_t OV9712_regs[] =
{
/* Clock Selection */
	//{REG5C,			0x79}, // [6:5]PLL Pre-divider - /1(b0x), /2(b10), or /4(b11)|[4:0]Pll-multiplier CLK2=CLK1 x (32-[4:0])
	//{REG5D,         0xf4}, // [5:4]Output drive capability - 1x(b00), 2x(b01), 3x(b10), or 4x(b11)
	{DVP_CTRL_00,	0xb0}, // [7:6]VSYNC - vsync_old(b00), vsync_new(b01), or vsync3(b10)|[5]pclk_gate_en|[4]vsync_gate|[3]vsync3_w_sel|[2]pclk reverse|[1]href reverse|[0]vsync reverse
    //{COM7,          0x40}, // [1]Color bar with pixel overlay
    //{DSP_CTRL_1,	0x2a}, // [7]SMPH Mean enable|[3]Color bar without pixel overlay|[1:0]Patterns
    {ENDR}
};

cam_register_t dummy[] =
{
/* Clock Selection */
	{REG5C,			0x59}, // [6:5]PLL Pre-divider - /1(b0x), /2(b10), or /4(b11)|[4:0]Pll-multiplier CLK2=CLK1 x (32-[4:0])

/* Driving Capability and Direction Control for I/O Pins */
    {REG5D,         0xd4}, // [5:4]Output drive capability - 1x(b00), 2x(b01), 3x(b10), or 4x(b11)
    {REG56,         0x1f}, // [7:5]Reserved|[4]HREF|[3]VSYNC|[2]PCLK|[1]Y9|[0]Y8

/* Histogram-Based AEC Algorithm Adjustment Controls */
    {REG72,         0x60}, // BP Level	  - Upper limit for B pixels
    {REG73,         0x49}, // WP Level 	  - Lower limit for W pixels
    {REG74,         0xe0}, // VBP Level	  - Upper limit for VB pixels
    {REG75,         0xe0}, // VWP Level	  - Lower limit for VW pixels
    {REG76,         0xc0}, // BP Number	  - Lower probability for B pixels to trigger auto AEC/AGC
    {REG77,         0xc0}, // WP Number	  - Lower probability for W pixels to trigger auto AEC/AGC
    {REG78,         0xff}, // VBP Number  - Lower probability for B pixels to trigger fast AEC/AGC
    {REG79,			0xff}, // VWP Number  - Lower probability for W pixels to trigger fast AEC/AGC
    
/* Test Pattern Selection Control */
    {COM7,          0x42}, // [1]Color bar with pixel overlay
    
/* Histogram-Based AEC/AGC Reference Area Option */
    {COM8,          0x85}, // [5]Banding on/off|[3]LAEC on/off
    {COM9,          0x10}, // [3]Enable AEC below banding value (0 - 1/100s<exposure<1/120s, 1 - allow shorter/longer exposure under strong light conditions)
    {COM5,          0x40}, // [3]VAEC on/off (Add frame)
    {REG03,         0x02}, // [7:5]VAEC Ceiling - 1(b001),2(b010),3(b011), or 7(b1XX)
    {COM13,         0x03}, // [5:4]Auto trigger VAEC when gain is greater than 2x(b00),4x(b01),8x(b10), or 16x(b11)
    {COM14,         0x00}, // [5:0]Maximum banding steps in terms of row exposure
    {REG4A,         0xd0}, // [1:0]Banding filter value BDst[9:8]
    {REG49,         0x00}, // [7:0]Banding filter value BDst[7:0]

/* BLC Control Functions */
    {REG41,         0x82}, // [3:0]Target black level for algorithm
    {REG43,         0xc0}, // [7:6]BLC line selection - R/Gr offset(bX0), R/Gb offset(b01), or all offsets(b11)
    {COM24,         0x02}, // [3]Manual BLC trigger for 64fr|[2]BLC always on
    {HOFFS,         0x00}, // [7:6]BLC offset for Gb[9:8]|[5:4]BLC offset for Gr[9:8]|[3:2]BLC offset for B[9:8]|[1:0]BLC offset for R[9:8]
    {ROFFS,         0x00}, // [7:0]BLC offset for R[7:0]
    {BOFFS,         0x00}, // [7:0]BLC offset for B[7:0]
    {GROFFS,        0x00}, // [7:0]BLC offset for Gr[7:0]
    {GBOFFS,        0x00}, // [7:0]BLC offset for Gb[7:0]

/* Digital Gain Control */
    {GAIN,          0x00}, // [7]Apply gain if gain >= 2x(b0), Apply gain if gain >= 4x(b1)

/* Pre-DSP Registers */
    {DSP_CTRL_1,	0x8a}, // [7]SMPH Mean enable|[3]Color bar without pixel overlay|[1:0]Patterns
    {DSP_CTRL_2,	0x00}, // [7:0]pre_out_hoff[7:0]
    {DSP_CTRL_3,	0x00}, // [7:0]pre_out_voff[7:0]
    {DSP_CTRL_4,	0x00}, // [5:4]pre_out_voff[9:8]|[2:0]pre_out_hoff[10:8]
    {DSP_CTRL_5,	0x04}, // [7:4]pre_pad_vrgt|[3:0]pre_pad_hrgt

/* LEN Registers */
    {DSP_CTRL_0, 	0xf9}, // [7]black_en|[6]white_en|[5]AWB gain enable|[4]AWB enable|[3]LENC correction enable|[2]lenc_gain_coef_en|[0]ISP Enable
    {LENC_CTRL_0,	0x00}, // def_coef_man
    {LENC_CTRL_1,	0x00}, // def_coef_th
    {LENC_CTRL_2,	0xf1}, // [7:4]lenc_gain_thre2[3:0]|[3:0]lenc_gain_thre1[3:0]
    {LENC_CTRL_3,	0x6a}, // [4]Manual center of RGB circle|[3]Round off last two bits of LENC input data|[2]def_coef_man_en|[1]lenc_gain_thre2[4]|[0]lenc_gain_thre1[4]
    {LENC_CTRL_4,	0x00}, // Black level manual setting

    {LENC_CTRL_7,	0x12}, // [5:4]LCC_Y[9:8] for R|[2:0]LCC_X[9:8] for R
    {LENC_CTRL_5,	0x80}, // LCC_X[7:0] for R
    {LENC_CTRL_6,	0x90}, // LCC_Y[7:0] for R
    {LENC_CTRL_8, 	0x22}, // [6:0]First group of factors for R
    {LENC_CTRL_9, 	0xc2}, // [6:0]Second group of factors for R
    {LENC_CTRL_10, 	0x87}, // [7:4]Second group of factors for R|[3:0]First group of factors for R

    {LENC_CTRL_13,	0x12}, // [5:4]LCC_Y[9:8] for G|[2:0]LCC_X[9:8] for G
    {LENC_CTRL_11,	0x80}, // LCC_X[7:0] for G
    {LENC_CTRL_12,	0x90}, // LCC_Y[7:0] for G
    {LENC_CTRL_14, 	0x22}, // [6:0]First group of factors for G
    {LENC_CTRL_15, 	0xc2}, // [6:0]Second group of factors for G
    {LENC_CTRL_16, 	0x87}, // [7:4]Second group of factors for G|[3:0]First group of factors for G

    {LENC_CTRL_19,	0x12}, // [5:4]LCC_Y[9:8] for B|[2:0]LCC_X[9:8] for B
    {LENC_CTRL_17,	0x80}, // LCC_X[7:0] for B
    {LENC_CTRL_18,	0x90}, // LCC_Y[7:0] for B
    {LENC_CTRL_20, 	0x22}, // [6:0]First group of factors for B
    {LENC_CTRL_21, 	0xc2}, // [6:0]Second group of factors for B
    {LENC_CTRL_22, 	0x87}, // [7:4]Second group of factors for B|[3:0]First group of factors for B

    {LENC_CTRL_23,	0x00}, // [2]V_skip|[0]H_skip - Normal image output(b0) or Sub-sampling output(b1)

/* White Balance Function */
    {COM25,         0x10}, // [4]Manual WB control(b0)| or ISP auto WB(b1)
    {BLUE,          0x40}, // Blue[11:4]
    {GREEN,         0x00}, // Green[11:4]
    {RED,           0x40}, // Red[11:4]
    {BRL,           0x00}, // [7:4]Blue[3:0]|[3:0]Red[3:0]
    {GL,            0x00}, // [7:4]Green[3:0]
    {AWB_CTRL_0,	0x07}, // [2]AWB fast mode enable|[1]AWB bias enable|[0]AWB bias plus enable - add bias after AWB gain is calculated
    {AWB_CTRL_2,	0x04}, // AWB stable range 1
    {AWB_CTRL_3,	0x08}, // AWB stable range 2
    {AWB_CTRL_4,	0x00}, // AWB frame count
    {AWB_CTRL_5,	0x20}, // AWB delta - steps toward stable range

/* YAVG Registers */
    {DSP_CTRL_6,	0x00}, // blk_thresh
    {DSP_CTRL_7,	0xff}, // wht_thresh
    {YAVG_CTRL_0,	0xa0}, // yavg_winh
    {YAVG_CTRL_1,	0xc8}, // yavg_winv
    {YAVG_CTRL_2,	0x00}, // yavg_winofh[7:0]
    {YAVG_CTRL_3,	0x00}, // yavg_winofv[7:0]
    {YAVG_CTRL_4,	0x00}, // [7]yavg_win_man|[5:4]yavg_winofv[9:8]|[2:0]yavg_winh[10:8]

/* System Control Registers */
    {SC_CTRL_0,     0x0c}, // [7]Bypass DVP|[5]snr_rst_ctrl_for DVP|[4]snr_rst_ctrl for DSP|[3]DVP gated clock|[2]DSP gated clock|[1]DVP reset|[0]DSP reset

/* DVP Control Register */
    {DVP_CTRL_00,	0x80}, // [7:6]VSYNC - vsync_old(b00), vsync_new(b01), or vsync3(b10)|[5]pclk_gate_en|[4]vsync_gate|[3]vsync3_w_sel|[2]pclk reverse|[1]href reverse|[0]vsync reverse
    {DVP_CTRL_01,	0x20}, // [7]ccir656_en|[6]sync_code_sel|[5]blk_656|[4]eav_first|[3:1]dvp_data_sequence - See datasheet table 6-2|[0]chg_flag
    {DVP_CTRL_02,	0xab}, // FS for CCIR656
    {DVP_CTRL_03,	0xb6}, // FE for CCIR656
    {DVP_CTRL_04,	0x80}, // FS for CCIR656
    {DVP_CTRL_05,	0x9d}, // LE for CCIR656
    {DVP_CTRL_06,	0x00}, // [7]blk_tog|[6]vsync_width_sel|[5:3]h2v delay|[2:0]v2h delay
    {DVP_CTRL_07,	0x80}, // vsync_width_l
    {DVP_CTRL_08,	0x24}, // [7:6]vsync_width_h|[5:4]sof2h delay offset|[3]ddr_phase|[2]Test pattern 10bit(b0) or 8bit(b1)[1:0]No pattern(b00), 1,2,4,8,?(b01), or 1,1,2,2,?(b10)
    {DVP_CTRL_09,	0xa6}, // [7]sof_rst_en,[6:5]chg_sel for recalculating DVP timing for HSYNC mode|[4]hsync_en|[3]hsync_dvp_en|[2]hsync_1st_en|[1:0]snr(b00), reg(b01), or auto(b10)
    {DVP_CTRL_0a,	0x40}, // [7:4]FIFO buffer size|[3:0]dmy_line_nu
    {DVP_CTRL_0b,	0x00}, // eof2v delay[23:16]
    {DVP_CTRL_0c,	0x01}, // eof2v delay[15:8]
    {DVP_CTRL_0d,	0x00}, // eof2v delay[7:0]
    {DVP_CTRL_0e,	0x00}, // line_length[15:8] 
    {DVP_CTRL_0f,	0x00}, // line_length[7:0]
    {DVP_CTRL_10,	0x00}, // clip_max[9:2]
    {DVP_CTRL_11,	0x00}, // clip_min[9:2]
    {DVP_CTRL_12,	0x00}, // [5]f_tog_en|[4]ddr_en|[3:2]clipmin[1:0]|[1:0]clip_max[1:0]	

	{ENDR}
};
//</editor-fold>

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
