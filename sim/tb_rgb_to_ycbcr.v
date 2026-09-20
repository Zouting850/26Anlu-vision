`timescale 1ns/1ps
// =============================================================================
// rgb_to_ycbcr 单元仿真
//   1) 8 个角点 (黑/白/红/绿/蓝/青/品/黄) 对齐 BT.601 已知值
//   2) 256 级灰阶斜坡 (r=g=b)
//   3) 4000 组随机向量, 中间穿插像素间隙
//   4) 固定 2 拍流水线延迟与 valid 对齐
//
// 比对方法: 先把整段激励预生成到数组, 再由单个 posedge 进程“检查第 c-2 拍喂入的
// 像素 + 驱动第 c 拍激励”。延迟关系写死在索引上, 插多少空隙都不会错位。
//
// 运行: iverilog -o sim/out/tb_ycbcr.vvp sim/tb_rgb_to_ycbcr.v src/vision/rgb_to_ycbcr.v
//       vvp sim/out/tb_ycbcr.vvp
// =============================================================================
module tb_rgb_to_ycbcr;

localparam NSTIM = 8192;
localparam NRAND = 4000;

reg         clk = 1'b0;
reg         rst = 1'b1;
reg         in_valid = 1'b0;
reg  [7:0]  in_r = 8'd0, in_g = 8'd0, in_b = 8'd0;
wire [7:0]  y, cb, cr, gray;
wire        out_valid;

always #20 clk = ~clk;          // 25MHz, 与 video_clk 同频

rgb_to_ycbcr dut (
    .clk(clk), .rst(rst),
    .in_valid(in_valid), .in_r(in_r), .in_g(in_g), .in_b(in_b),
    .y(y), .cb(cb), .cr(cr), .gray(gray), .out_valid(out_valid)
);

// ---------------------------------------------------------------------------
// TB 侧独立黄金模型
// 必须先把 8bit 输入搬进 integer(有符号) 再乘: 若直接用 input [7:0] 参与运算,
// Verilog 的表达式定标规则会把整个式子变成无符号, -38*r 回卷、>> 变逻辑移位。
// ---------------------------------------------------------------------------
function automatic integer gold_y (input [7:0] rv, input [7:0] gv, input [7:0] bv);
    integer r, g, b;
    begin
        r = rv; g = gv; b = bv;
        gold_y   = ($signed( 66*r + 129*g +  25*b + 128) >>> 8) + 16;
    end
endfunction
function automatic integer gold_cb (input [7:0] rv, input [7:0] gv, input [7:0] bv);
    integer r, g, b;
    begin
        r = rv; g = gv; b = bv;
        gold_cb  = ($signed(-38*r -  74*g + 112*b + 128) >>> 8) + 128;
    end
endfunction
function automatic integer gold_cr (input [7:0] rv, input [7:0] gv, input [7:0] bv);
    integer r, g, b;
    begin
        r = rv; g = gv; b = bv;
        gold_cr  = ($signed(112*r -  94*g -  18*b + 128) >>> 8) + 128;
    end
endfunction
function automatic integer gold_gr (input [7:0] rv, input [7:0] gv, input [7:0] bv);
    integer r, g, b;
    begin
        r = rv; g = gv; b = bv;
        gold_gr  =  $signed(77*r + 150*g +  29*b + 128) >>> 8;
    end
endfunction

reg        st_v  [0:NSTIM-1];
reg [23:0] st_rgb[0:NSTIM-1];
integer    nstim = 0;

task automatic push_stim(input vld, input [7:0] r, input [7:0] g, input [7:0] b);
    begin
        st_v[nstim]   = vld;
        st_rgb[nstim] = {r, g, b};
        nstim = nstim + 1;
    end
endtask

task automatic push_gap(input integer n);
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) push_stim(1'b0, 8'd0, 8'd0, 8'd0);
    end
endtask

// 角点: 既入激励, 又打印黄金值表。用局部变量做参数是因为 iverilog 对常量实参会走
// 编译期折叠, 该路径不按有符号处理 >>, 打印出来的 Cb/Cr 会是回卷后的大数。
task automatic corner(input [7:0] r, input [7:0] g, input [7:0] b);
    reg [7:0] rr, gg, bb;
    begin
        rr = r; gg = g; bb = b;
        push_stim(1'b1, rr, gg, bb);
        $display("  RGB=%03d,%03d,%03d -> Y=%03d Cb=%03d Cr=%03d gray=%03d",
                 rr, gg, bb, gold_y(rr,gg,bb), gold_cb(rr,gg,bb),
                 gold_cr(rr,gg,bb), gold_gr(rr,gg,bb));
    end
endtask

integer errors = 0, nvec = 0;
reg [7:0] ey, ecb, ecr, egr;
// 实测值域: 用来证明 rgb_to_ycbcr “不加钳位”的论断真的成立
integer y_min = 255, y_max = 0, cb_min = 255, cb_max = 0;
// 初值必须是 0..255 之间的非负数: 与 unsigned [7:0] 比较时 Verilog 会把整个
// 比较拉成无符号, 负初值会被当成 42 亿从而永远不更新 (本 TB 第一版就栽在这)
integer cr_min = 255, cr_max = 0, gr_min = 255, gr_max = 0;

// ---------------------------------------------------------------------------
// 单进程: 每拍检查 (c-2) 的期望值, 并驱动第 c 拍的激励
// ---------------------------------------------------------------------------
integer c = 0;
reg [7:0] r0, g0, b0;
reg       ref_valid;
always @(posedge clk) begin
    #1;
    if (c >= 2) begin
        r0 = st_rgb[c-2][23:16]; g0 = st_rgb[c-2][15:8]; b0 = st_rgb[c-2][7:0];
        ref_valid = st_v[c-2];
        nvec = nvec + ref_valid;
        if (out_valid !== ref_valid) begin
            errors = errors + 1;
            $display("FAIL t=%0t c=%0d out_valid=%b 应为 %b", $time, c, out_valid, ref_valid);
        end
        if (ref_valid) begin
            if (y    < y_min)  y_min  = y;    if (y    > y_max)  y_max  = y;
            if (cb   < cb_min) cb_min = cb;   if (cb   > cb_max) cb_max = cb;
            if (cr   < cr_min) cr_min = cr;   if (cr   > cr_max) cr_max = cr;
            if (gray < gr_min) gr_min = gray; if (gray > gr_max) gr_max = gray;
            ey  = gold_y (r0,g0,b0);
            ecb = gold_cb(r0,g0,b0);
            ecr = gold_cr(r0,g0,b0);
            egr = gold_gr(r0,g0,b0);
            if ({y, cb, cr, gray} !== {ey, ecb, ecr, egr}) begin
                errors = errors + 1;
                $display("FAIL t=%0t c=%0d RGB=%03d,%03d,%03d 期望 Y=%03d Cb=%03d Cr=%03d gray=%03d 实际 Y=%03d Cb=%03d Cr=%03d gray=%03d",
                         $time, c, r0, g0, b0, ey, ecb, ecr, egr, y, cb, cr, gray);
            end
        end
    end
    if (c < nstim) begin
        in_valid          = st_v[c];
        {in_r, in_g, in_b} = st_rgb[c];
    end
    else begin
        in_valid = 1'b0;
    end
    c = c + 1;
end

// ---------------------------------------------------------------------------
// 激励生成
// ---------------------------------------------------------------------------
integer i, j;
reg [7:0] rr, rg, rb;
initial begin
    for (i = 0; i < NSTIM; i = i + 1) begin
        st_v[i] = 1'b0; st_rgb[i] = 24'd0;
    end

    push_gap(4);                                  // 复位期间
    // 1) 角点
    $display("---- BT.601 角点黄金值 ----");
    corner(8'd0,   8'd0,   8'd0  );   // 黑
    corner(8'd255, 8'd255, 8'd255);   // 白
    corner(8'd255, 8'd0,   8'd0  );   // 红  Y=82  Cb=90  Cr=240
    corner(8'd0,   8'd255, 8'd0  );   // 绿  Y=144 Cb=54  Cr=34
    corner(8'd0,   8'd0,   8'd255);   // 蓝  Y=41  Cb=240 Cr=110
    corner(8'd0,   8'd255, 8'd255);   // 青
    corner(8'd255, 8'd0,   8'd255);   // 品
    corner(8'd255, 8'd255, 8'd0  );   // 黄
    push_gap(4);

    // 2) 灰阶斜坡
    for (i = 0; i < 256; i = i + 1)
        push_stim(1'b1, i[7:0], i[7:0], i[7:0]);
    push_gap(4);

    // 3) 随机向量 + 随机间隙
    for (j = 0; j < NRAND; j = j + 1) begin
        rr = {$random} % 256; rg = {$random} % 256; rb = {$random} % 256;
        push_stim(1'b1, rr, rg, rb);
        if ((j % 7) == 3)   push_gap(1);
        if ((j % 53) == 11) push_gap(3);
    end
    push_gap(8);

    // 复位在第一个 posedge (t=20) 之前释放, 之后每拍严格喂一次激励
    #5;
    rst = 1'b0;

    // 收尾: 等剩余激励跑完
    wait (c >= nstim + 4);
    $display("比对像素数 = %0d, 错误数 = %0d", nvec, errors);
    $display("实测值域: Y[%0d,%0d] Cb[%0d,%0d] Cr[%0d,%0d] gray[%0d,%0d]",
             y_min, y_max, cb_min, cb_max, cr_min, cr_max, gr_min, gr_max);
    // 钳位边界正好等于这些极值 => 比较器永不动作, 才敢省掉钳位
    if (y_min != 16 || y_max != 235) begin
        $display("FAIL: Y 值域不是 [16,235], 省钳位的依据不成立"); errors = errors + 1;
    end
    if (cb_min != 16 || cb_max != 240 || cr_min != 16 || cr_max != 240) begin
        $display("FAIL: Cb/Cr 值域不是 [16,240]"); errors = errors + 1;
    end
    if (gr_min != 0 || gr_max != 255) begin
        $display("FAIL: gray 值域不是 [0,255]"); errors = errors + 1;
    end
    if (nvec < 8 + 256 + NRAND - 10) begin
        $display("FAIL: 比对样本数过少 (%0d)", nvec);
        $fatal(1);
    end
    if (errors == 0) $display("== tb_rgb_to_ycbcr PASS ==");
    else             $display("== tb_rgb_to_ycbcr FAIL (%0d errors) ==", errors);
    $finish;
end

endmodule
