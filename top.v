`timescale 1ns / 1ps
// =============================================================================
// B板视觉控制系统 - Phase 1: Camera → SDRAM → HDMI
// FPGA: Anlogic EG4S20BG256 (HX4S20C开发板)
// 
// 功能：OV5640摄像头采集 → RGB888写入SDRAM → 读取显示到HDMI
// 分辨率：640×480@60Hz
// =============================================================================

module top(
    // --- 系统时钟与复位 ---
    input   wire        clk_50,          // 50MHz系统时钟
    input   wire        rst_n,           // 外部复位（可选，内部POR已足够）
    
    // --- 摄像头接口 (OV5640 DVP + I2C) ---
    input   wire        cam_pclk,        // 摄像头像素时钟
    input   wire        cam_vsync,       // 帧同步
    input   wire        cam_href,        // 行有效
    input   wire [7:0]  cam_data,        // 8位数据 (RGB565)
    output  wire        cam_rst_n,       // 摄像头复位 (低有效)
    output  wire        cam_pwdn,        // 摄像头掉电控制 (高有效)
    output  wire        cam_scl,         // I2C时钟
    inout   wire        cam_sda,         // I2C数据
    
    // --- HDMI输出 (LVDS) ---
    output  wire        HDMI_CLK_P,      // TMDS时钟差分正
    output  wire        HDMI_D0_P,       // TMDS通道0差分正
    output  wire        HDMI_D1_P,       // TMDS通道1差分正
    output  wire        HDMI_D2_P,       // TMDS通道2差分正
    
    // --- LED指示 (简单状态灯) ---
    output  wire        led_cam,         // 摄像头工作指示灯
    output  wire        led_hmi          // HDMI输出指示灯
    
);

// ===========================================================================
// 1. 时钟与复位生成
// ===========================================================================

wire por_reset_n;          // POR发生器输出
wire master_reset_n;       // 主复位 (低有效)
wire master_reset;         // 主复位 (高有效)
wire ext_mem_clk;          // SDRAM时钟 (125MHz)
wire mem_clk_sft;          // SDRAM相位偏移时钟 (125MHz+90°)
wire video_clk;            // HDMI像素时钟 (25MHz)
wire video_clk_5x;         // HDMI序列化器时钟 (125MHz)
wire hdmi_locked;          // HDMI PLL锁定标志

por_generator u_por_gen (
    .clk_50     (clk_50),
    .por_reset_n(por_reset_n)
);

assign master_reset_n = por_reset_n & rst_n;
assign master_reset   = ~master_reset_n;

video_pll u_video_pll (
    .refclk     (clk_50),
    .reset      (master_reset),
    .extlock    (hdmi_locked),
    .clk0_out   (ext_mem_clk),     // 125MHz → SDRAM controller
    .clk1_out   (mem_clk_sft),     // 125MHz+90° → SDRAM IP phase shift
    .clk2_out   (video_clk),       // 25MHz → HDMI pixel clock
    .clk3_out   (video_clk_5x)     // 125MHz → HDMI serializer
);

// ===========================================================================
// 2. 摄像头路径
// ===========================================================================

wire cam_rst_n_int;
wire cam_pwdn_int;
wire cam_scl_int;
wire cam_sda_int;

wire cmos_frame_vsync;
wire cmos_frame_href;
wire cmos_frame_valid;
wire [15:0] cmos_wr_data;

ov5640_dri u_ov5640_dri (
    .clk              (clk_50),
    .rst_n            (master_reset_n),
    .cam_pclk         (cam_pclk),
    .cam_vsync        (cam_vsync),
    .cam_href         (cam_href),
    .cam_data         (cam_data),
    .cam_rst_n        (cam_rst_n_int),
    .cam_pwdn         (cam_pwdn_int),
    .cam_scl          (cam_scl_int),
    .cam_sda          (cam_sda_int),
    .cmos_h_pixel     (13'd640),   // 640x480
    .cmos_v_pixel     (13'd480),
    .total_h_pixel    (13'd1066),  // 640 + 128 + 298 = 1066
    .total_v_pixel    (13'd525),   // 480 + 3 + 42 = 525
    .capture_start    (master_reset_n),
    .cam_init_done    (),
    .cmos_frame_vsync (cmos_frame_vsync),
    .cmos_frame_href  (cmos_frame_href),
    .cmos_frame_valid (cmos_frame_valid),
    .cmos_frame_data  (cmos_wr_data)
);

assign cam_rst_n = master_reset_n ? cam_rst_n_int : 1'b0;
assign cam_pwdn  = master_reset_n ? cam_pwdn_int  : 1'b1;
assign cam_scl   = master_reset_n ? cam_scl_int   : 1'b1;
assign cam_sda   = master_reset_n ? cam_sda_int   : 1'bz;

ov5640_delay u_ov5640_delay (
    .clk                (cam_pclk),
    .rst_n              (master_reset_n),
    .cmos_frame_vsync   (cmos_frame_vsync),
    .cmos_frame_href    (cmos_frame_href),
    .cmos_frame_valid   (cmos_frame_valid),
    .cmos_wr_data       (cmos_wr_data),
    .cam_write_en       (cam_write_en),
    .cam_write_data     (cam_write_data),
    .cam_write_req      (cam_write_req),
    .cam_write_req_ack  (cam_write_req_ack)
);

// ===========================================================================
// 3. SDRAM读写仲裁器
// ===========================================================================

wire App_wr_en;
wire [20:0] App_wr_addr;
wire [31:0] App_wr_din;
wire [3:0]  App_wr_dm;
wire App_rd_en;
wire [20:0] App_rd_addr;
wire Sdr_rd_en;
wire [31:0] Sdr_rd_dout;

frame_read_write #(
    .ADDR_BITS(21),
    .READ_DATA_BITS(32),
    .WRITE_DATA_BITS(32)
) u_frame_read_write (
    .mem_clk            (ext_mem_clk),
    .rst                (master_reset),
    .Sdr_init_done      (sdram_ip.Sdr_init_done),
    .Sdr_init_ref_vld   (1'b0),
    .Sdr_busy           (sdram_ip.Sdr_busy),
    .App_wr_en          (App_wr_en),
    .App_wr_addr        (App_wr_addr),
    .App_wr_din         (App_wr_din),
    .App_wr_dm          (App_wr_dm),
    .App_rd_en          (App_rd_en),
    .App_rd_addr        (App_rd_addr),
    .Sdr_rd_en          (Sdr_rd_en),
    .Sdr_rd_dout        (Sdr_rd_dout),
    .write_clk          (cam_pclk),
    .write_req          (cam_write_req),
    .write_req_ack      (cam_write_req_ack),
    .write_finish       (),
    .write_addr_0       (21'd0),
    .write_addr_index   (2'd0),
    .write_len          (21'd307200),   // 640*480 pixels
    .write_en           (cam_write_en),
    .write_data         (cam_write_data),
    .read_clk           (video_clk),
    .read_req           (video_read_req),
    .read_req_ack       (video_read_req_ack),
    .read_finish        (),
    .read_addr_0        (21'd0),
    .read_addr_index    (2'd0),
    .read_len           (21'd307200),
    .read_en            (video_read_en),
    .read_data          (video_read_data)
);

// ===========================================================================
// 4. SDRAM控制器 (在-package SDRAM via hard macro)
// ===========================================================================

EG_PHY_SDRAM_2M_32 #(
    .FAMILY("EG4"),
    .SELF_REFRESH_OPEN(1'b1)
) sdram_ip (
    .Clk            (ext_mem_clk),
    .Clk_sft        (mem_clk_sft),
    .Rst            (master_reset),
    .Sdr_init_done  (sdram_init_done),
    .Sdr_init_ref_vld(),
    .Sdr_busy       (sdram_busy),
    .App_wr_en      (App_wr_en),
    .App_wr_addr    (App_wr_addr),
    .App_wr_dm      (App_wr_dm),
    .App_wr_din     (App_wr_din),
    .App_rd_en      (App_rd_en),
    .App_rd_addr    (App_rd_addr),
    .Sdr_rd_en      (Sdr_rd_en),
    .Sdr_rd_dout    (Sdr_rd_dout)
);

// ===========================================================================
// 5. HDMI显示路径
// ===========================================================================

wire hs_0, vs_0, de_0;
wire hs, vs, de;
wire [23:0] vout_data;

video_timing_data u_video_timing_data (
    .video_clk      (video_clk),
    .rst            (master_reset),
    .read_req       (video_read_req),
    .read_req_ack   (video_read_req_ack),
    .hs             (hs_0),
    .vs             (vs_0),
    .de             (de_0)
);

video_delay #(
    .DATA_WIDTH(24)
) u_video_delay (
    .video_clk      (video_clk),
    .rst            (master_reset),
    .read_en        (video_read_en),
    .read_data      (video_read_data[31:8]),   // Camera format: {R,G,B,0} → take [31:8]
    .hs             (hs_0),
    .vs             (vs_0),
    .de             (de_0),
    .hs_r           (hs),
    .vs_r           (vs),
    .de_r           (de),
    .vout_data      (vout_data)
);

hdmi_tx #(.FAMILY("EG4")) u_hdmi_tx (
    .PXLCLK_I       (video_clk),
    .PXLCLK_5X_I    (video_clk_5x),
    .RST_N          (~master_reset),
    .VGA_HS         (hs),
    .VGA_VS         (vs),
    .VGA_DE         (de),
    .VGA_RGB        (vout_data),
    .HDMI_CLK_P     (HDMI_CLK_P),
    .HDMI_D0_P      (HDMI_D0_P),
    .HDMI_D1_P      (HDMI_D1_P),
    .HDMI_D2_P      (HDMI_D2_P)
);

// ===========================================================================
// 6. LED指示灯
// ===========================================================================

assign led_cam  = cmos_frame_valid;      // 摄像头有数据时亮
assign led_hmi  = de;                    // HDMI有视频输出时亮

endmodule
