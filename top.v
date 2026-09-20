`timescale 1ns / 1ps
// =============================================================================
// B板视觉控制系统 — Phase 2: OV5640 → SDRAM(乒乓双缓冲) → YCbCr → HDMI
// Device: Anlogic EG4S20BG256 (HX4S20C)
//
// 数据通路:
//   ov5640_dri (I2C配置 + DVP采集, RGB565)
//     → ov5640_delay (RGB565→RGB888, 打包为 {R,G,B,8'd0})
//     → frame_read_write (写域 cam_pclk / 读域 video_clk 异步仲裁)
//         ↑ 基址由 frame_buffer_ctrl 按帧提交在 BUF0/BUF1 间交替 (Phase 2 新增)
//     → sdram (片内 2M×32 SDRAM, EG_PHY_SDRAM_2M_32 hard macro)
//     → display_path (rgb_to_ycbcr BT.601 流水线 + 显示模式 mux + video_delay
//         PIPE_LAT=2) → hdmi_tx (VHDL) → TMDS
// =============================================================================

module top (
    input  wire        clk_50,        // 50MHz 晶振
    input  wire        key1,          // 板载按键 A2, 低有效, 作为外部复位
    input  wire        key2,          // 板载按键 B2, 低有效, 切换显示模式

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
    output wire        led_hdmi,      // HDMI 时序运行

    // --- 分段诊断 (板载 4 颗独立 LED, 高电平点亮, 不依赖 SW6) ---
    output wire        led1,          // A4  SCCB 配置完成
    output wire        led2,          // A3  片内 SDRAM 就绪
    output wire        led3,          // C10 读通道心跳 (SDRAM -> HDMI), 闪 = 读通道在跑完帧
    output wire        led4           // B12 写通道心跳 (摄像头 -> SDRAM), 闪 = 乒乓在提交
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

// 乒乓地址划分: 片内 SDRAM 为 2M×32 = 2,097,152 字 (21bit 字地址),
// 两块 307200 字共占 29%, 余量够 Phase 3/5 再加灰度缓存或第 3 块缓冲。
localparam [20:0] BUF0_BASE    = 21'd0;
localparam [20:0] BUF1_BASE    = 21'd307200;

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
wire        hdmi_read_finish;
wire        hdmi_read_en;
wire [31:0] hdmi_read_data;

// 乒乓指针: 全部在 mem_clk 域, 与 frame_read_write 内部采样 *_addr_index 的
// 时钟同域, 因此不需要跨时钟同步。
wire        frame_commit;
wire [1:0]  fb_wr_index;
wire [1:0]  fb_rd_index;

frame_buffer_ctrl u_fb_ctrl (
    .mem_clk    (mem_clk),
    .rst        (sys_rst),
    .wr_finish  (frame_commit),
    .wr_index   (fb_wr_index),
    .rd_index   (fb_rd_index)
);

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

    // 写通道: 摄像头 (基址随 fb_wr_index 在两块缓冲间交替)
    .write_clk        (cam_pclk),
    .write_req        (cam_write_req),
    .write_req_ack    (cam_write_req_ack),
    .write_finish     (frame_commit),
    .write_addr_0     (BUF0_BASE),
    .write_addr_1     (BUF1_BASE),
    .write_addr_2     (BUF0_BASE),
    .write_addr_3     (BUF0_BASE),
    .write_addr_index (fb_wr_index),
    .write_len        (FRAME_WORDS),
    .write_en         (cam_write_en),
    .write_data       (cam_write_data),

    // 读通道: HDMI 显示 (只读最近一次提交完成的那一帧)
    .read_clk         (video_clk),
    .read_req         (hdmi_read_req),
    .read_req_ack     (hdmi_read_req_ack),
    .read_finish      (hdmi_read_finish),
    .read_addr_0      (BUF0_BASE),
    .read_addr_1      (BUF1_BASE),
    .read_addr_2      (BUF0_BASE),
    .read_addr_3      (BUF0_BASE),
    .read_addr_index  (fb_rd_index),
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

// 5.1 key2 (B2) 切换显示模式, 编码见 display_path.v 的 MODE_*:
//     0=原图 1=灰度 2=YCbCr 假彩 3=彩条(调试档, 不读帧缓冲)
wire key2_press;
ax_debounce #(
    .FREQ (25)                      // video_clk 25MHz
) u_key2 (
    .clk           (video_clk),
    .rst           (sys_rst),
    .button_in     (key2),
    .button_posedge(),
    .button_negedge(key2_press),
    .button_out    ()
);

reg [1:0] disp_mode;
always @(posedge video_clk or posedge sys_rst) begin
    if (sys_rst)         disp_mode <= 2'd0;                                  // 原图
    else if (key2_press) disp_mode <= (disp_mode == 2'd3) ? 2'd0 : disp_mode + 2'd1;
end

// 5.2 显示通路: YCbCr 转换 + 模式选择 + 时序对齐 (详见 src/video/display_path.v)
display_path #(
    .DATA_WIDTH (24),
    .PIPE_LAT   (2)                 // = rgb_to_ycbcr 流水线深度
) u_disp (
    .video_clk  (video_clk),
    .rst        (sys_rst),
    .read_data  (hdmi_read_data),
    .hs_i       (hs_0),
    .vs_i       (vs_0),
    .de_i       (de_0),
    .mode       (disp_mode),
    .read_en    (hdmi_read_en),
    .hs_o       (hs),
    .vs_o       (vs),
    .de_o       (de),
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

// cmos_frame_vsync 是持续多拍的电平, 必须边沿计数才是"每帧一次";
// 直接 if(sig) 会让计数器跑在 pclk 速率上, LED 糊成常亮。
reg       cam_vsync_d0;
reg [7:0] cam_frame_cnt;
always @(posedge cam_pclk or posedge sys_rst) begin
    if (sys_rst) begin
        cam_vsync_d0  <= 1'b0;
        cam_frame_cnt <= 8'd0;
    end
    else begin
        cam_vsync_d0 <= cmos_frame_vsync;
        if (cmos_frame_vsync & ~cam_vsync_d0)
            cam_frame_cnt <= cam_frame_cnt + 8'd1;
    end
end

// 两条独立心跳, 一边盯乒乓的一半:
//   led3 = 读通道 (SDRAM -> HDMI): read_finish 每读完一帧一拍 @59.5Hz, /32 翻转 → 0.93Hz
//   led4 = 写通道 (摄像头 -> SDRAM): frame_commit 每提交一帧一拍 @13.1Hz, /8 翻转 → 0.82Hz
// 两个计数器都跑在 mem_clk 上 —— 这两个事件本来就是 mem_clk 域的单拍脉冲, 不存在跨域。
//
// 为什么改成翻转计数而不是展宽脉冲: 上一版把 commit 展宽到 40ms, 而写帧周期只有 76ms,
// 占空比 53% —— 肉眼看是"常亮", 把"乒乓在翻页"这个结论整个说反了。翻转计数亮灭各半, 不会骗人。
//
// 读通道一旦被 SDRAM 仲裁卡住 (frame_fifo_read 只有在 App_wr_busy=0 时才敢发起突发),
// led3 会熄灭而 led4 照闪; 两边都闪则说明像素已经送到 display_path 入口,
// 问题只在转换/HDMI/显示器一侧。
reg [5:0] rd_hb;
reg [3:0] wr_hb;
always @(posedge mem_clk or posedge sys_rst) begin
    if (sys_rst) begin
        rd_hb <= 6'd0;
        wr_hb <= 4'd0;
    end
    else begin
        if (hdmi_read_finish) rd_hb <= rd_hb + 6'd1;
        if (frame_commit)     wr_hb <= wr_hb + 4'd1;
    end
end

// 帧率 = 24MHz/(1856*984) ≈ 13.1fps → bit[3] 每 8 帧翻转 ≈ 0.8Hz 闪
assign led_hdmi = blink_cnt[24];                              // ~0.75Hz
assign led_cam  = cam_frame_cnt[3] & cam_init_done;
assign led1     = cam_init_done;                              // 常亮 = SCCB 表写完
assign led2     = Sdr_init_done;                              // 常亮 = 片内 SDRAM 就绪
assign led3     = rd_hb[5];                                   // 闪   = 读通道在跑完整帧
assign led4     = wr_hb[3];                                   // 闪   = 写通道在提交整帧

endmodule
