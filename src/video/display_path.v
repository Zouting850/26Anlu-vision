`timescale 1ns/1ps
// =============================================================================
// 显示通路 (Phase 2 建立, Phase 3 挂上火焰检测)
//   帧缓冲读出的像素 -> YCbCr -> 显示模式选择 -> 时序对齐 -> 火焰蒙版/HUD 叠加
//
// 抽成独立模块的目的: tb_display_path 可以直接例化这一份 (与综合进 FPGA 的完全
// 相同), 而不是在 testbench 里复刻一遍接线 —— 流水线对齐恰好是最容易被复刻走样
// 的部分。Phase 3 的检测器也挂在这里, 同样是为了让"蒙版落位"这件事能在仿真里被
// 端到端验证, 而不是只在上板时用眼睛对。
//
// 像素序 (PIPE_LAT = 2, 与 video_delay 的节拍编号同一套记法):
//   第18拍 read_en 向读 FIFO 取数 -> 第19拍 dout 有效 (rfifo 为 SHOW_AHEAD=0 /
//   NOREG, 读延迟 1 拍) -> 转换器 S1 第20拍 -> S2 第21拍 -> video_delay 在
//   de_d[21] 捕获, de_r 取 de_d[22]。
// 因此 in_valid 取 read_en 延一拍, 原图通路另加 2 拍延迟与转换结果同列。
//
// ★ 第21拍这一层叫"检测级"(de_cap), 有两条不显然的性质:
//   1) 此刻 read_data 里已经是**后面两个像素**的数据了 (每个 read_en 只把它保留一拍),
//      所以喂给 fire_detector 的 R/G/B 必须另走一条 PIPE_LAT 移位链 (det_rgb_dly),
//      不能直接取 px_r/px_g/px_b —— 直接取会让色域判据看错像素, 且只在边缘露出来。
//   2) 光栅坐标 (cx,cy) 由 de_cap/vs_cap 自己数出来, 与转换结果天然同列, 所以
//      蒙版写地址、读地址、检测坐标是同一套, 不需要任何坐标变换。
// =============================================================================
module display_path
#(
    parameter DATA_WIDTH = 24,
    parameter PIPE_LAT   = 2,           // rgb_to_ycbcr 的流水线深度, 须 >=1 (rgb_dly 至少要一级)

    // 火焰检测的块网格: 640/8 × 480/8。像素宽高必须是 (1<<BLOCK_SHIFT) 的整数倍,
    // 否则帧边角上会有半块永远攒不满 (本项目 640×480、TB 的 128×64 都满足)。
    parameter  HBLKS        = 80,
    parameter  VBLKS        = 60,
    parameter  BLOCK_SHIFT  = 3,
    parameter  MASK_AW      = 13,
    parameter  integer FIRE_BLOCK_MIN = 20,   // 块内候选像素门限 (满块 64)
    parameter  integer FIRE_ALARM_BLK = 6,    // 报警块数门限 (6 块 ≈ 全帧 0.125%)
    parameter  FIRE_HB_DIV    = 32,           // HUD 心跳分频 (0.93Hz)
    parameter  FIRE_ALRM_DIV  = 15            // HUD 报警闪烁分频 (1.98Hz)
)
(
    input  wire                    video_clk,
    input  wire                    rst,

    input  wire [31:0]             read_data,     // 帧缓冲一字: {R,G,B,8'd0}
    input  wire                    hs_i,
    input  wire                    vs_i,
    input  wire                    de_i,
    input  wire [1:0]              mode,          // 见 MODE_* 定义

    output wire                    read_en,       // 向 frame_read_write 取数
    output wire                    hs_o,
    output wire                    vs_o,
    output wire                    de_o,
    output wire [DATA_WIDTH-1:0]   vout_data
);

localparam [1:0] MODE_RAW  = 2'd0,
                 MODE_GRAY = 2'd1,
                 MODE_YCC  = 2'd2,
                 MODE_BAR  = 2'd3;   // 调试档: 8 列彩条, 完全不碰帧缓冲

// 摄像头写入格式为 {R,G,B,8'd0}, 低 8 位是填充, 故取 [31:8]
wire [7:0] px_r = read_data[31:24];
wire [7:0] px_g = read_data[23:16];
wire [7:0] px_b = read_data[15:8];

reg px_vld_d1;
always @(posedge video_clk or posedge rst) begin
    if (rst) px_vld_d1 <= 1'b0;
    else     px_vld_d1 <= read_en;
end

wire [7:0] px_y, px_cb, px_cr, px_gray;
rgb_to_ycbcr u_rgb2ycbcr (
    .clk       (video_clk),
    .rst       (rst),
    .in_valid  (px_vld_d1),
    .in_r      (px_r),
    .in_g      (px_g),
    .in_b      (px_b),
    .y         (px_y),
    .cb        (px_cb),
    .cr        (px_cr),
    .gray      (px_gray),
    .out_valid ()                 // 与 de_cap 同拍, 对齐由本模块统一用 de_cap 描述
);

// 调试彩条: 8 列 × 80px = 640, 与 HDMI 输出列数写死对齐 (见 top.v 的 H_DISP)。
// 目的是把"HDMI/显示器/同步这条路是活的"与"帧缓冲内容对不对"分开判断 ——
// 它不需要摄像头出帧、也不需要 SDRAM 读通, 只要 video_clk 在跑就该有图。
//
// 按 read_en 而不是 de_i 计数: read_en 与"此刻正在被显示的像素"同序, 条带落位严格
// 对齐; 若按 de_i 计数, 计数器会比显示的列超前整条流水线的深度 (19 拍), 行首行尾会
// 各露出一截白, 看起来像"彩条左边缺一条"。
// 640 = 8 × 80 刚好一行, 所以 bar_idx 每条 8 位计数器自然回卷, 每一行都从白重新开始。
reg [6:0] bar_w;      // 条内像素 0..79
reg [2:0] bar_idx;    // 条号 0..7
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        bar_w   <= 7'd0;
        bar_idx <= 3'd0;
    end
    else if (read_en) begin
        if (bar_w == 7'd79) begin
            bar_w   <= 7'd0;
            bar_idx <= bar_idx + 3'd1;
        end
        else begin
            bar_w <= bar_w + 7'd1;
        end
    end
end

reg [DATA_WIDTH-1:0] bar_rgb;
always @* begin
    case (bar_idx)
        3'd0:    bar_rgb = 24'hFFFFFF;   // 白
        3'd1:    bar_rgb = 24'hFFFF00;   // 黄
        3'd2:    bar_rgb = 24'h00FFFF;   // 青
        3'd3:    bar_rgb = 24'h00FF00;   // 绿
        3'd4:    bar_rgb = 24'hFF00FF;   // 品红
        3'd5:    bar_rgb = 24'hFF0000;   // 红
        3'd6:    bar_rgb = 24'h0000FF;   // 蓝
        default: bar_rgb = 24'h000000;   // 黑
    endcase
end

// 原图通路延后 PIPE_LAT 拍, 否则 mode=RAW 时 RGB 与 YCbCr 错列。
// 彩条从这里进同一条移位链, 因此它与 RAW 共用 default 分支, 对齐天然一致。
localparam DLY_W = PIPE_LAT * DATA_WIDTH;
wire [DATA_WIDTH-1:0] pipe_in = (mode == MODE_BAR) ? bar_rgb : {px_r, px_g, px_b};
reg [DLY_W-1:0] rgb_dly;
always @(posedge video_clk or posedge rst) begin
    if (rst) rgb_dly <= {DLY_W{1'b0}};
    else     rgb_dly <= {rgb_dly[DLY_W-DATA_WIDTH-1:0], pipe_in};
end

reg [DATA_WIDTH-1:0] disp_rgb;
always @* begin
    case (mode)
        MODE_GRAY: disp_rgb = {px_gray, px_gray, px_gray};
        MODE_YCC:  disp_rgb = {px_y,    px_cb,   px_cr};
        default:   disp_rgb = rgb_dly[DLY_W-1 -: DATA_WIDTH];   // RAW 与 BAR 都取移位链输出
    endcase
end

// video_delay 里的捕获级节拍 (de_d[CAP_IDX]): 转换结果与 disp_rgb 都在这一列
wire de_cap, vs_cap;
wire [DATA_WIDTH-1:0] vout_raw;
video_delay #(
    .DATA_WIDTH (DATA_WIDTH),
    .PIPE_LAT   (PIPE_LAT)
) u_video_delay (
    .video_clk  (video_clk),
    .rst        (rst),
    .read_en    (read_en),
    .read_data  (disp_rgb),
    .hs         (hs_i),
    .vs         (vs_i),
    .de         (de_i),
    .hs_r       (hs_o),
    .vs_r       (vs_o),
    .de_r       (de_o),
    .de_cap     (de_cap),
    .vs_cap     (vs_cap),
    .vout_data  (vout_raw)
);

// ===========================================================================
// Phase 3: 火焰检测 (分用显示读通道, 见 docs/implementation-plan.md 的 Phase 3)
// ===========================================================================

// ---- 检测级光栅坐标: 只数 de_cap, 与转换结果同列 ----
// cx 必须在 de_cap 那一拍就 +1 (用 de_cap_d 会慢一拍: 第 k 个有效像素处 cx=k-1,
// 整块蒙版右移一个像素 —— 块边界那一列的候选像素会被算进隔壁块, 帧边的像素掉到门外)。
// 消隐期 cx 清零, 所以每行第一个有效像素处 cx==0。
// cy: 行末 +1; vs_cap 的任何一条边沿都落在消隐里且在本帧第一个有效行之前
//     (640×480 时序: vs 脉冲在第 480~481 行, 之后还有 33 行背 porch), 所以任何
//     边沿都能复位 —— 不挑极性, 换显示时序也不用改这里。
reg [9:0] cx;
reg [8:0] cy;
reg       de_cap_d, vs_cap_d;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        cx <= 10'd0;
        cy <= 9'd0;
        de_cap_d <= 1'b0;
        vs_cap_d <= 1'b0;
    end
    else begin
        de_cap_d <= de_cap;
        vs_cap_d <= vs_cap;
        if (vs_cap ^ vs_cap_d)       cy <= 9'd0;
        else if (de_cap_d & ~de_cap) cy <= cy + 9'd1;
        if (~de_cap)                        cx <= 10'd0;
        else                                cx <= cx + 10'd1;
    end
end

wire       px_vld   = de_cap;                       // 本拍像素在有效显示区内
wire [9:0] px_x     = cx;
wire [8:0] px_y_r   = cy;
// 本遍最后一个有效像素 = 帧末。要求 H/V 像素数正好等于网格 × 块边长 (640=80×8,
// 480=60×8), 所以它同时也是一个块的最后一拍 —— 分析器才能在某一拍里同时完成
// "最后一块入账"和"帧统计锁存"。
wire       pass_end = px_vld & (cx == (HBLKS << BLOCK_SHIFT) - 1)
                            & (cy == (VBLKS << BLOCK_SHIFT) - 1);

// 原始 RGB 另走一条 PIPE_LAT 链, 与转换后的 Y/Cb/Cr 同列 (见文件头 ★1)
reg [PIPE_LAT*24-1:0] det_rgb_dly;
always @(posedge video_clk or posedge rst) begin
    if (rst) det_rgb_dly <= {(PIPE_LAT*24){1'b0}};
    else     det_rgb_dly <= {det_rgb_dly[PIPE_LAT*24-24-1:0], px_r, px_g, px_b};
end
wire [7:0] det_r = det_rgb_dly[PIPE_LAT*24-1 -: 8];
wire [7:0] det_g = det_rgb_dly[PIPE_LAT*24-9 -: 8];
wire [7:0] det_b = det_rgb_dly[PIPE_LAT*24-17 -: 8];

wire fire_px;
fire_detector u_fire_det (
    .y       (px_y),
    .cb      (px_cb),
    .cr      (px_cr),
    .r       (det_r),
    .g       (det_g),
    .b       (det_b),
    .fire_px (fire_px)
);

// 蒙版读地址: 与写地址同一套坐标 (块号 × 网格宽 + 块号)
wire [6:0] bx = px_x[9:BLOCK_SHIFT];
wire [5:0] by = px_y_r[8:BLOCK_SHIFT];
wire [MASK_AW-1:0] mask_raddr = by * HBLKS + bx;

wire       mask_rbit;
wire [19:0] stat_px_cnt;
wire [12:0] stat_blk_cnt;
wire [9:0]  stat_cx;
wire [8:0]  stat_cy;
wire        alarm, hb_tgl, alrm_tgl, mask_valid;

fire_region_analyzer #(
    .HBLKS       (HBLKS),
    .VBLKS       (VBLKS),
    .BLOCK_SHIFT (BLOCK_SHIFT),
    .MASK_AW     (MASK_AW),
    .BLOCK_MIN   (FIRE_BLOCK_MIN),
    .ALARM_BLK   (FIRE_ALARM_BLK),
    .HB_DIV      (FIRE_HB_DIV),
    .ALRM_DIV    (FIRE_ALRM_DIV)
) u_fire_region (
    .video_clk    (video_clk),
    .rst          (rst),
    .px_vld       (px_vld),
    .px           (px_x),
    .py           (px_y_r),
    .fire_px      (fire_px),
    .pass_end     (pass_end),
    .stat_px_cnt  (stat_px_cnt),
    .stat_blk_cnt (stat_blk_cnt),
    .stat_cx      (stat_cx),
    .stat_cy      (stat_cy),
    .alarm        (alarm),
    .hb_tgl       (hb_tgl),
    .alrm_tgl     (alrm_tgl),
    .mask_valid   (mask_valid),
    .mask_raddr   (mask_raddr),
    .mask_rbit    (mask_rbit)
);

// ---- 叠加: vout_raw 与 mask_rbit / 延一拍坐标同列, 全部是组合选择 ----
// vout_raw 在 de_cap 的下一拍有效 (video_delay 的 OUT_IDX 级), 所以坐标与蒙版都取
// 延一拍的形式; mask_rbit 本来就是 RAM 的一拍读延迟, 正好对上。
// 这里**不能再加寄存级**: de_o 已经和 vout_raw 同列, 多一拍就会与 HDMI 序列化错位。
reg [9:0] cx_q;
reg [8:0] cy_q;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        cx_q <= 10'd0;
        cy_q <= 9'd0;
    end
    else begin
        cx_q <= px_x;
        cy_q <= px_y_r;
    end
end

wire [6:0] ox = cx_q[9:BLOCK_SHIFT];      // 输出级块列号
wire [5:0] oy = cy_q[8:BLOCK_SHIFT];      // 输出级块行号

// 50% 半透明红: 保留底图轮廓, 又能一眼看出被标了 (方案里写的是"半透明覆盖")
wire [8:0] mix_r_sum = {1'b0, vout_raw[23:16]} + 9'd255;
wire [7:0] mix_r = mix_r_sum[8:1];                       // (R+255)/2
wire [7:0] mix_g = {1'b0, vout_raw[15:8]} >> 1;
wire [7:0] mix_b = {1'b0, vout_raw[7:0]}  >> 1;

// 右上角 4 格 HUD (2 个块高 = 16 行, 每格 8 像素宽):
//   蓝   = 检测遍心跳 (0.93Hz 明暗各半)  -> 本模块在收遍
//   黄   = 本遍有候选像素 (色域判据命中)
//   橙   = 本遍有块过密度门限 (蒙版里确有红块)
//   红闪 = 报警中 (1.98Hz)
// HUD 在四档模式下都画 (含彩条档), 这样切到调试档时检测器状态仍然可读。
localparam [6:0] HUD_X0 = HBLKS - 4;         // 右上角最右 4 个块列
wire [2:0] hud_i   = ox - HUD_X0;
wire       hud_win = (oy < 6'd2) & (ox >= HUD_X0);   // 块行 0..1 (16 行高)
wire       hud_px  = (stat_px_cnt  != 20'd0);
wire       hud_blk = (stat_blk_cnt != 13'd0);

reg        hud_on;
reg [23:0] hud_rgb;
always @* begin
    case (hud_i)
        3'd0:      begin hud_on = hb_tgl;             hud_rgb = 24'h0000FF; end
        3'd1:      begin hud_on = hud_px;             hud_rgb = 24'hFFFF00; end
        3'd2:      begin hud_on = hud_blk;            hud_rgb = 24'hFF8000; end
        default:   begin hud_on = alarm & alrm_tgl;   hud_rgb = 24'hFF0000; end
    endcase
end

wire overlay_en = mask_valid & (mode != MODE_BAR);   // 彩条档不叠蒙版, 保持纯时序基准

// HUD 那 8×16 的窗口内一律由 HUD 接管 (灭着的格子画黑), 否则底色会从格子里透出来,
// 四格判读表就没法读了 —— 这块黑底就是"这一格在闪/在亮"能一眼看清的前提。
assign vout_data = hud_win ? (hud_on ? hud_rgb : {DATA_WIDTH{1'b0}})
                 : (overlay_en & mask_rbit) ? {mix_r, mix_g, mix_b}
                 : vout_raw;

endmodule
