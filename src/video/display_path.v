`timescale 1ns/1ps
// =============================================================================
// 显示通路 (Phase 2): 帧缓冲读出的像素 -> YCbCr -> 显示模式选择 -> 时序对齐
//
// 抽成独立模块的目的: tb_display_path 可以直接例化这一份 (与综合进 FPGA 的完全
// 相同), 而不是在 testbench 里复刻一遍接线 —— 流水线对齐恰好是最容易被复刻走样
// 的部分。
//
// 像素序 (PIPE_LAT = 2):
//   第18拍 read_en 向读 FIFO 取数 -> 第19拍 dout 有效 (rfifo 为 SHOW_AHEAD=0 /
//   NOREG, 读延迟 1 拍) -> 转换器 S1 第20拍 -> S2 第21拍 -> video_delay 在
//   de_d[21] 捕获, de_r 取 de_d[22]。
// 因此 in_valid 取 read_en 延一拍, 原图通路另加 2 拍延迟与转换结果同列。
// =============================================================================
module display_path
#(
    parameter DATA_WIDTH = 24,
    parameter PIPE_LAT   = 2            // rgb_to_ycbcr 的流水线深度, 须 >=1 (rgb_dly 至少要一级)
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
    .out_valid ()
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
// 彩条从这里进同一条移位链, 因此它与 RAW 共用下面的 default 分支, 对齐天然一致。
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
    .vout_data  (vout_data)
);

endmodule
