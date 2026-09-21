`timescale 1ns/1ps
// =============================================================================
// 火焰逐像素候选判据 (Phase 3) —— 纯组合逻辑, 没有时钟也没有流水线
//
// 输入是 rgb_to_ycbcr 输出的 BT.601 限定量 (Y∈[16,235], Cb/Cr∈[16,240]),
// 与它同拍即可, 因此本模块不引入任何延迟 —— 检测级与显示级共用同一套光栅坐标,
// 蒙版落位不需要再对齐 (延迟一旦错开一拍, 红块就会骑在火焰边缘上)。
//
// 判据 = 实施方案的 5 条 + 1 条切肤色的:
//   Y     >= Y_MIN    (180)  火焰亮度
//   Cr    >= CR_MIN   (155)  红色色度高
//   Cb    <= CB_MAX   (120)  蓝色色度低
//   R > G, R > B             暖色调 (实施方案原有条款)
//   Cr-Cb >= DIFF_MIN (60)   切肤色: 肤色 Cr≈150/Cb≈110 (差≈40),
//                            火焰橙 Cr≈210/Cb≈50 (差≈160)
//
// 顺带把"R>G / R>B 到底是不是废话"算清楚了 (默认门限下它们确实被色度门限蕴含):
//   Cr>=155 即 ((112R-94G-18B+128)>>8)+128>=155, 需要 X=112R-94G-18B >= 6784。
//   若 R<=G: X <= 112R-94R-18B = 18(R-B) <= 4590, 矛盾  => R>G 恒成立。
//   Cr-Cb>=60 需要 150R-20G-130B >= 15360; 若 R<=B: 该式 <= 20(R-G) <= 5100,
//   矛盾                                                              => R>B 恒成立。
// 之所以仍然实现这两条: 四条门限都是参数, 一旦按现场调低 CR_MIN/DIFF_MIN, 蕴含就
// 断了, 而 2 个 8 位比较器只值 2 个 LUT。
//
// 已知无法靠色域区分的误报类: 黄色灯光、铜/黄铜反光、夕阳下的土墙 —— 它们与火焰在
// YCbCr 里就是同一种颜色。本模块**不试图**区分, 交给 fire_region_analyzer 的块内
// 密度 + 全帧面积 + 连续多遍确认三层过滤。
// =============================================================================
module fire_detector
#(
    parameter integer Y_MIN    = 180,
    parameter integer CR_MIN   = 155,
    parameter integer CB_MAX   = 120,
    parameter integer DIFF_MIN = 60
)
(
    input  wire [7:0] y,
    input  wire [7:0] cb,
    input  wire [7:0] cr,
    input  wire [7:0] r,
    input  wire [7:0] g,
    input  wire [7:0] b,
    output wire       fire_px
);

// 参数落成定宽 localparam: 比较两侧的位宽一眼可见, 也不用依赖"无位宽参数"的隐式扩展
localparam [7:0] T_Y  = Y_MIN[7:0];
localparam [7:0] T_CR = CR_MIN[7:0];
localparam [7:0] T_CB = CB_MAX[7:0];
localparam [8:0] T_DF = DIFF_MIN[8:0];

wire [8:0] cr_minus_cb = (cr > cb) ? ({1'b0, cr} - {1'b0, cb}) : 9'd0;

assign fire_px = (y  >= T_Y)
              && (cr >= T_CR)
              && (cb <= T_CB)
              && (r  >  g)
              && (r  >  b)
              && (cr_minus_cb >= T_DF);

endmodule
