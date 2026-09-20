`timescale 1ns / 1ps
// =============================================================================
// B板视觉控制系统 — Phase 1: OV5640 → SDRAM → HDMI
// Device: Anlogic EG4S20BG256 (HX4S20C)
//
// 数据通路:
//   ov5640_dri (I2C配置 + DVP采集, RGB565)
//     → ov5640_delay (RGB565→RGB888, 打包为 {R,G,B,8'd0})
//     → frame_read_write (写域 cam_pclk / 读域 video_clk 异步仲裁)
//     → sdram (片内 2M×32 SDRAM, EG_PHY_SDRAM_2M_32 hard macro)
//     → video_timing_data + video_delay → hdmi_tx (VHDL) → TMDS
// =============================================================================

module top (
    input  wire        clk_50,        // 50MHz 晶振
    input  wire        key1,          // 板载按键, 低有效, 作为外部复位

    // --- OV5640 ---
    input  wire        cam_pclk,
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_data,
    output wire        cam_rst_n,
    output wire        cam_pwdn,
    output wire        cam_scl,
    inout  wire        cam_sda,

    // --- HDMI (TMDS LVDS) ---
    output wire        HDMI_CLK_P,
    output wire        HDMI_D0_P,
    output wire        HDMI_D1_P,
    output wire        HDMI_D2_P,

    // --- 状态指示 ---
    output wire        led_cam,       // 摄像头出帧
    output wire        led_hdmi       // HDMI 时序运行
);

// ---------------------------------------------------------------------------
// 帧参数 (与参考工程 top_dual_udp.v 一致, 需匹配 i2c_ov5640_rgb565_cfg.v 的
// 寄存器时序, 否则 cmos_capture_data 的帧有效窗口会错位)
// ---------------------------------------------------------------------------
localparam [12:0] H_DISP       = 13'd640;
localparam [12:0] V_DISP       = 13'd480;
localparam [12:0] TOTAL_H      = 13'd1856;   // 640 + 1216
localparam [12:0] TOTAL_V      = 13'd984;    // 480 + 504
localparam        FRAME_WORDS  = 21'd307200; // 640*480

// ---------------------------------------------------------------------------
// 1. 复位与时钟
// ---------------------------------------------------------------------------
wire por_reset_n;
wire sys_rst_n = por_reset_n & key1;
wire sys_rst   = ~sys_rst_n;

wire mem_clk;       // 125MHz  SDRAM 控制器时钟
wire mem_clk_sft;   // 125MHz  90°   SDRAM 相移时钟
wire video_clk;     //  25MHz  HDMI 像素时钟
wire video_clk_5x;  // 125MHz  HDMI 串行化时钟

por_generator u_por_gen (
    .clk_50      (clk_50),
    .por_reset_n (por_reset_n)
);

video_pll u_video_pll (
    .refclk   (clk_50),
    .reset    (sys_rst),
    .extlock  (),
    .clk0_out (mem_clk),
    .clk1_out (mem_clk_sft),
    .clk2_out (video_clk),
    .clk3_out (video_clk_5x)
);

// ---------------------------------------------------------------------------
// 2. 摄像头采集
// ---------------------------------------------------------------------------
wire        cmos_frame_vsync;
wire        cmos_frame_href;
wire        cmos_frame_valid;
wire [15:0] cmos_frame_data;

wire        cam_init_done;
wire        Sdr_init_done;
wire        Sdr_busy;

wire        cam_write_req;
wire        cam_write_req_ack;
wire        cam_write_en;
wire [31:0] cam_write_data;

ov5640_dri u_ov5640_dri (
    .clk              (clk_50),
    .rst_n            (sys_rst_n),
    .cam_pclk         (cam_pclk),
    .cam_vsync        (cam_vsync),
    .cam_href         (cam_href),
    .cam_data         (cam_data),
    .cam_rst_n        (cam_rst_n),
    .cam_pwdn         (cam_pwdn),
    .cam_scl          (cam_scl),
    .cam_sda          (cam_sda),          // inout 直连, 不在顶层二次驱动
    .cmos_h_pixel     (H_DISP),
    .cmos_v_pixel     (V_DISP),
    .total_h_pixel    (TOTAL_H),
    .total_v_pixel    (TOTAL_V),
    .capture_start    (Sdr_init_done),    // SDRAM 就绪后再开始采集
    .cam_init_done    (cam_init_done),
    .cmos_frame_vsync (cmos_frame_vsync),
    .cmos_frame_href  (cmos_frame_href),
    .cmos_frame_valid (cmos_frame_valid),
    .cmos_frame_data  (cmos_frame_data)
);

// RGB565 → RGB888, 写成 {R[31:24], G[23:16], B[15:8], 8'd0}
ov5640_delay u_ov5640_delay (
    .clk              (cam_pclk),
    .rst_n            (sys_rst_n),
    .cmos_frame_vsync (cmos_frame_vsync),
    .cmos_frame_href  (cmos_frame_href),
    .cmos_frame_valid (cmos_frame_valid),
    .cmos_wr_data     (cmos_frame_data),
    .cam_write_en     (cam_write_en),
    .cam_write_data   (cam_write_data),
    .cam_write_req    (cam_write_req),
    .cam_write_req_ack(cam_write_req_ack)
);

// ---------------------------------------------------------------------------
// 3. SDRAM 读写仲裁 (写: cam_pclk 域, 读: video_clk 域)
// ---------------------------------------------------------------------------
wire        App_wr_en;
wire [20:0] App_wr_addr;
wire [31:0] App_wr_din;
wire [3:0]  App_wr_dm;
wire        App_rd_en;
wire [20:0] App_rd_addr;
wire        Sdr_rd_en;
wire [31:0] Sdr_rd_dout;

wire        hdmi_read_req;
wire        hdmi_read_req_ack;
wire        hdmi_read_en;
wire [31:0] hdmi_read_data;

frame_read_write #(
    .ADDR_BITS       (21),
    .MEM_DATA_BITS   (32),
    .READ_DATA_BITS  (32),
    .WRITE_DATA_BITS (32)
) u_frame_rw (
    .rst              (sys_rst),
    .mem_clk          (mem_clk),
    .Sdr_init_done    (Sdr_init_done),
    .Sdr_init_ref_vld (1'b0),
    .Sdr_busy         (Sdr_busy),

    .App_rd_en        (App_rd_en),
    .App_rd_addr      (App_rd_addr),
    .Sdr_rd_en        (Sdr_rd_en),
    .Sdr_rd_dout      (Sdr_rd_dout),

    .App_wr_en        (App_wr_en),
    .App_wr_addr      (App_wr_addr),
    .App_wr_din       (App_wr_din),
    .App_wr_dm        (App_wr_dm),

    // 写通道: 摄像头
    .write_clk        (cam_pclk),
    .write_req        (cam_write_req),
    .write_req_ack    (cam_write_req_ack),
    .write_finish     (),
    .write_addr_0     (21'd0),
    .write_addr_1     (21'd0),
    .write_addr_2     (21'd0),
    .write_addr_3     (21'd0),
    .write_addr_index (2'd0),
    .write_len        (FRAME_WORDS),
    .write_en         (cam_write_en),
    .write_data       (cam_write_data),

    // 读通道: HDMI 显示
    .read_clk         (video_clk),
    .read_req         (hdmi_read_req),
    .read_req_ack     (hdmi_read_req_ack),
    .read_finish      (),
    .read_addr_0      (21'd0),
    .read_addr_1      (21'd0),
    .read_addr_2      (21'd0),
    .read_addr_3      (21'd0),
    .read_addr_index  (2'd0),
    .read_len         (FRAME_WORDS),
    .read_en          (hdmi_read_en),
    .read_data        (hdmi_read_data)
);

// ---------------------------------------------------------------------------
// 4. SDRAM 控制器 (片内 SDRAM, 无需外部引脚约束)
// ---------------------------------------------------------------------------
sdram u_sdram (
    .Clk              (mem_clk),
    .Clk_sft          (mem_clk_sft),
    .Rst              (sys_rst),
    .Sdr_init_done    (Sdr_init_done),
    .Sdr_init_ref_vld (),
    .Sdr_busy         (Sdr_busy),
    .App_wr_en        (App_wr_en),
    .App_wr_addr      (App_wr_addr),
    .App_wr_dm        (App_wr_dm),
    .App_wr_din       (App_wr_din),
    .App_rd_en        (App_rd_en),
    .App_rd_addr      (App_rd_addr),
    .Sdr_rd_en        (Sdr_rd_en),
    .Sdr_rd_dout      (Sdr_rd_dout)
);

// ---------------------------------------------------------------------------
// 5. HDMI 显示输出
// ---------------------------------------------------------------------------
wire hs_0, vs_0, de_0;
wire hs,  vs,  de;
wire [23:0] vout_data;

video_timing_data u_video_timing (
    .video_clk    (video_clk),
    .rst          (sys_rst),
    .read_req     (hdmi_read_req),
    .read_req_ack (hdmi_read_req_ack),
    .hs           (hs_0),
    .vs           (vs_0),
    .de           (de_0)
);

// 摄像头写入格式为 {R,G,B,8'd0}, 故取 [31:8] 而非参考工程 app.v 的 [23:0]
video_delay #(
    .DATA_WIDTH (24)
) u_video_delay (
    .video_clk  (video_clk),
    .rst        (sys_rst),
    .read_en    (hdmi_read_en),
    .read_data  (hdmi_read_data[31:8]),
    .hs         (hs_0),
    .vs         (vs_0),
    .de         (de_0),
    .hs_r       (hs),
    .vs_r       (vs),
    .de_r       (de),
    .vout_data  (vout_data)
);

hdmi_tx #(
    .FAMILY ("EG4")
) u_hdmi_tx (
    .PXLCLK_I    (video_clk),
    .PXLCLK_5X_I (video_clk_5x),
    .RST_N       (sys_rst_n),
    .VGA_HS      (hs),
    .VGA_VS      (vs),
    .VGA_DE      (de),
    .VGA_RGB     (vout_data),
    .HDMI_CLK_P  (HDMI_CLK_P),
    .HDMI_D2_P   (HDMI_D2_P),
    .HDMI_D1_P   (HDMI_D1_P),
    .HDMI_D0_P   (HDMI_D0_P)
);

// ---------------------------------------------------------------------------
// 6. 状态指示 (分频到肉眼可见)
// ---------------------------------------------------------------------------
reg [24:0] blink_cnt;
always @(posedge video_clk or posedge sys_rst) begin
    if (sys_rst) blink_cnt <= 25'd0;
    else         blink_cnt <= blink_cnt + 25'd1;
end

reg [3:0] cam_frame_cnt;
always @(posedge cam_pclk or posedge sys_rst) begin
    if (sys_rst)            cam_frame_cnt <= 4'd0;
    else if (cmos_frame_vsync) cam_frame_cnt <= cam_frame_cnt + 4'd1;  // 每帧翻转低位
end

assign led_hdmi = blink_cnt[24];                              // ~1Hz
assign led_cam  = cam_frame_cnt[3] & cam_init_done;           // 摄像头初始化完成且有帧

endmodule
