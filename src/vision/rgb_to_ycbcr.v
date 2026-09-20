`timescale 1ns/1ps
// =============================================================================
// RGB888 -> YCbCr (ITU-R BT.601 限定范围) + 灰度, 2 级流水线, 固定 2 拍延迟
//
// 系数与取整方式沿用已上板验证的参考工程
// 开发案例/.../Main/q3q4/import/udp_cam_ctrl.v:319-322
//   Y  = ( 66R + 129G +  25B + 128) >>> 8 + 16
//   Cb = (-38R -  74G + 112B + 128) >>> 8 + 128
//   Cr = (112R -  94G -  18B + 128) >>> 8 + 128
//   gray = (77R + 150G + 29B + 128) >>> 8      // 0~255 全范围, 供 Sobel/帧间差分
//
// S1 做乘法+加法并寄存中间值, S2 只做移位+偏置, 使 S1 成为唯一的关键路径。
// =============================================================================
module rgb_to_ycbcr (
    input  wire               clk,        // 像素时钟 (本项目为 video_clk)
    input  wire               rst,        // 高有效, 与 clk 同步使用

    input  wire               in_valid,
    input  wire [7:0]         in_r,
    input  wire [7:0]         in_g,
    input  wire [7:0]         in_b,

    output reg  [7:0]         y,
    output reg  [7:0]         cb,
    output reg  [7:0]         cr,
    output reg  [7:0]         gray,
    output reg                out_valid
);

// 输入扩展为 signed, 使常量乘法留在有符号域内 (-38*R 等才不会回卷)
wire signed [8:0] r_s = {1'b0, in_r};
wire signed [8:0] g_s = {1'b0, in_g};
wire signed [8:0] b_s = {1'b0, in_b};

// 中间值范围 (极值代入): y_acc ∈ [128, 56356], gray_acc ∈ [128, 65408],
// cb_acc/cr_acc ∈ [-28432, 28688] —— 18 位有符号足够。
reg signed [17:0] y_acc;
reg signed [17:0] cb_acc;
reg signed [17:0] cr_acc;
reg signed [17:0] gray_acc;
reg               valid_d1;

// ---------------------------------------------------------------------------
// S1: 加权求和 + 四舍五入偏置 (+128)
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        y_acc    <= 18'sd0;
        cb_acc   <= 18'sd0;
        cr_acc   <= 18'sd0;
        gray_acc <= 18'sd0;
        valid_d1 <= 1'b0;
    end
    else begin
        y_acc    <= 66*r_s + 129*g_s +  25*b_s + 18'sd128;
        cb_acc   <= -38*r_s - 74*g_s + 112*b_s + 18'sd128;
        cr_acc   <= 112*r_s - 94*g_s -  18*b_s + 18'sd128;
        gray_acc <= 77*r_s + 150*g_s +  29*b_s + 18'sd128;
        valid_d1 <= in_valid;
    end
end

// ---------------------------------------------------------------------------
// S2: 算术右移取整 + 偏置
//
// 不做钳位: 输入为 8bit 时值域恰好落在 Y[16,235] / Cb,Cr[16,240] / gray[0,255],
// 实施方案里写的钳位边界正是这些极值本身 (R=G=B=255 得 Y=235; B=255 得 Cb=240),
// 比较器永远不会动作, 加上去只是白耗 LUT。
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        y         <= 8'd0;
        cb        <= 8'd0;
        cr        <= 8'd0;
        gray      <= 8'd0;
        out_valid <= 1'b0;
    end
    else begin
        y         <= (y_acc    >>> 8) + 18'sd16;
        cb        <= (cb_acc   >>> 8) + 18'sd128;
        cr        <= (cr_acc   >>> 8) + 18'sd128;
        gray      <= gray_acc  >>> 8;
        out_valid <= valid_d1;
    end
end

endmodule
