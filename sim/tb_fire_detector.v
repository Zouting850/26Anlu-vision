`timescale 1ns/1ps
// =============================================================================
// Phase 3 火焰检测单元仿真
//
// 三段独立证据:
//   A. fire_detector 的色域真值表 —— 含一个"偏红肤色"样本, 它在 DIFF_MIN=0 时会被
//      判成火焰、在默认 DIFF_MIN=60 时不会, 用来证明那条判据不是凑数
//   B. fire_region_analyzer 的块密度 / 帧统计 / 报警时序 / 质心除法器
//   C. 蒙版 RAM 的落位与乒乓 (逐地址回读 128 个块)
//
// 几何缩小成 16×8 块 (128×64 像素, 块边长仍是 8), 这样一遍只有 8192 拍, 跑得快,
// 而块逻辑的行为与 80×60 完全同构。
// =============================================================================
module tb_fire_detector;

localparam HBLKS = 16, VBLKS = 8, BS = 3;
localparam HPX = HBLKS << BS;          // 128
localparam VPX = VBLKS << BS;          // 64
localparam NBLK = HBLKS * VBLKS;       // 128
localparam GAP = 40;                   // 模拟消隐 (除法器要 19 拍)

reg clk = 1'b0, rst = 1'b1;
always #20 clk = ~clk;                 // 25MHz

integer errors = 0;

// ---------------------------------------------------------------------------
// A. 逐像素判据真值表
//    Y/Cb/Cr 用参考公式算 (这条公式已由 tb_rgb_to_ycbcr 对 4264 个像素逐位比对过,
//    并与 BT.601 公布角点值一致), 这里只关心"色域判据命中/漏掉"的取舍。
// ---------------------------------------------------------------------------
reg  [7:0] t_r, t_g, t_b;
reg  [7:0] t_y, t_cb, t_cr;
wire       f_def, f_nodiff;

fire_detector dut_def (                          // 默认门限
    .y(t_y), .cb(t_cb), .cr(t_cr),
    .r(t_r), .g(t_g), .b(t_b), .fire_px(f_def));

fire_detector #(.DIFF_MIN(0)) dut_nodiff (       // 关掉肤色那一刀
    .y(t_y), .cb(t_cb), .cr(t_cr),
    .r(t_r), .g(t_g), .b(t_b), .fire_px(f_nodiff));

// BT.601 参考模型: 与 rgb_to_ycbcr 逐位一致 (写法沿用 tb_display_path,
// 关键是 -38*R 那类负项必须在有符号域里算, 所以整体套 $signed)
reg signed [31:0] acc;
task automatic chip(input integer ri, gi, bi, input integer exp_def, exp_nodiff,
                    input [8*70-1:0] nm);
    begin
        t_r = ri[7:0]; t_g = gi[7:0]; t_b = bi[7:0];
        t_y  = (( 66*ri + 129*gi +  25*bi + 128) >> 8) + 16;
        acc  = -38*ri - 74*gi + 112*bi + 128;  t_cb = (acc >> 8) + 128;
        acc  = 112*ri - 94*gi -  18*bi + 128;  t_cr = (acc >> 8) + 128;
        #1;
        if (f_def !== exp_def[0]) begin
            errors = errors + 1;
            $display("FAIL %0s (%0d,%0d,%0d) -> Y=%0d Cb=%0d Cr=%0d: 默认判据得 %0b 期望 %0b",
                     nm, ri, gi, bi, t_y, t_cb, t_cr, f_def, exp_def[0]);
        end
        if (f_nodiff !== exp_nodiff[0]) begin
            errors = errors + 1;
            $display("FAIL %0s: DIFF_MIN=0 时得 %0b 期望 %0b", nm, f_nodiff, exp_nodiff[0]);
        end
        $display("真值 %0s rgb=(%0d,%0d,%0d) Y=%0d Cb=%0d Cr=%0d Cr-Cb=%0d => 默认%0b 无肤色刀%0b",
                 nm, ri, gi, bi, t_y, t_cb, t_cr, t_cr - t_cb, f_def, f_nodiff);
    end
endtask

// ---------------------------------------------------------------------------
// B/C. 区域分析器
// ---------------------------------------------------------------------------
reg          px_vld = 1'b0;
reg  [9:0]   px = 10'd0;
reg  [8:0]   py = 9'd0;
reg          fire_px = 1'b0;
reg          pass_end = 1'b0;
reg  [6:0]   mask_raddr = 7'd0;
wire         mask_rbit;
wire [19:0]  stat_px_cnt;
wire [12:0]  stat_blk_cnt;
wire [9:0]   stat_cx;
wire [8:0]   stat_cy;
wire         alarm, hb_tgl, alrm_tgl, mask_valid;

fire_region_analyzer #(
    .HBLKS(HBLKS), .VBLKS(VBLKS), .BLOCK_SHIFT(BS), .MASK_AW(7),
    .BLOCK_MIN(20), .ALARM_BLK(6), .SET_N(3), .CLR_N(8)
) dut (
    .video_clk(clk), .rst(rst),
    .px_vld(px_vld), .px(px), .py(py), .fire_px(fire_px), .pass_end(pass_end),
    .stat_px_cnt(stat_px_cnt), .stat_blk_cnt(stat_blk_cnt),
    .stat_cx(stat_cx), .stat_cy(stat_cy),
    .alarm(alarm), .hb_tgl(hb_tgl), .alrm_tgl(alrm_tgl), .mask_valid(mask_valid),
    .mask_raddr(mask_raddr), .mask_rbit(mask_rbit)
);

// 场景: 返回本拍像素是否是候选像素
integer scene = 0;
function automatic bit_px(input integer x, y);
    integer bx, by, k;
    begin
        bx = x >> BS;  by = y >> BS;  k = 0;
        case (scene)
            0: begin                                   // 16 块火焰 + 2 个不够密度的块
                if (bx >= 4 && bx <= 7 && by >= 2 && by <= 5) k = 1;
                else if (bx == 10 && by == 6 && ((x & 7) < 4) && ((y & 7) < 3)) k = 1;   // 12/64
                else if (bx == 2  && by == 2 && ((x & 7) == 0)) k = 1;                    // 8/64
            end
            1: k = 0;                                  // 空场景 (肤色/灰墙: 逐像素已被挡住)
            2: begin                                   // 只有 1 块达标: 测 den=1 的除法
                if (bx == 9 && by == 1) k = 1;
            end
            3: begin                                   // 块内密度边界 19/20
                if (bx == 5 && by == 4 && (((y & 7) * 8 + (x & 7)) < 20)) k = 1;
                else if (bx == 6 && by == 4 && (((y & 7) * 8 + (x & 7)) < 19)) k = 1;
            end
            4: begin                                   // 横跨整帧的条带: 测质心与 6 块门限
                if (by >= 3 && by <= 4) k = 1;
            end
            default: k = 0;
        endcase
        bit_px = k;
    end
endfunction

integer n_pass = 0;
task automatic run_pass;
    integer x, y;
    begin
        for (y = 0; y < VPX; y = y + 1) begin
            for (x = 0; x < HPX; x = x + 1) begin
                @(negedge clk);
                px_vld  = 1'b1;  px = x[9:0];  py = y[8:0];
                fire_px = bit_px(x, y);
                pass_end = (x == HPX-1) && (y == VPX-1);
                @(negedge clk);
                px_vld = 1'b0;  pass_end = 1'b0;  fire_px = 1'b0;
            end
        end
        repeat (GAP) @(negedge clk);          // 消隐: 除法器在这里跑完
        n_pass = n_pass + 1;
    end
endtask

// 蒙版回读: 逐地址扫一遍, 采样点要在地址给出之后的一拍 (RAM 读延迟)
reg [NBLK-1:0] mask_got;
task automatic sweep_mask;
    integer a;
    begin
        for (a = 0; a < NBLK; a = a + 1) begin
            @(negedge clk);  mask_raddr = a[6:0];
            @(negedge clk);  mask_got[a] = mask_rbit;
        end
        @(negedge clk);  mask_raddr = 7'd0;
    end
endtask

// 把期望的块集合拼成向量
reg [NBLK-1:0] mask_exp;
task automatic expect_mask(input integer x0, x1, y0, y1, input integer on);
    integer a, bx, by;
    begin
        for (a = 0; a < NBLK; a = a + 1) begin
            bx = a % HBLKS;  by = a / HBLKS;
            mask_exp[a] = on && (bx >= x0) && (bx <= x1) && (by >= y0) && (by <= y1);
        end
    end
endtask

task automatic chk(input integer cond, input [8*40-1:0] msg);
    begin
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %0s", msg);
        end
    end
endtask

integer first_alarm_pass = -1, clear_pass = -1;
reg had_alarm = 0;

initial begin
    repeat (5) @(negedge clk);
    rst = 1'b0;
    repeat (2) @(negedge clk);

    // ---- A. 真值表 ----
    // Y>=180 是硬坎: 纯橙 (255,140,0) 的 Y 只有 152, 暗红 (200,30,10) 只有 84 ——
    // 这套判据圈住的是火焰**发光的外套层**, 不包暗红外晕, 也不包过白的焰心。
    chip(255, 190,  30, 1, 1, "火焰亮橙");
    chip(255, 200,  60, 1, 1, "火焰黄橙-与黄光灯同色-已知误报类");
    chip(255, 240, 180, 0, 0, "近白焰心-Cr被G拉平-靠外圈成块");
    chip(255, 140,   0, 0, 0, "纯橙-Y不到180");
    chip(250, 180, 150, 0, 1, "偏红肤色");
    chip(245, 195, 165, 0, 0, "一般肤色");
    chip(255, 250, 230, 0, 0, "白炽灯");
    chip(128, 128, 128, 0, 0, "灰墙");
    chip(  0,   0, 255, 0, 0, "蓝");
    chip(  0, 255,   0, 0, 0, "绿");
    chip(255,   0, 255, 0, 0, "品红");
    chip(200,  30,  10, 0, 0, "暗红(亮度不够)");

    // ---- B/C. 场景 0: 16 块火焰 ----
    scene = 0;
    run_pass;
    chk(stat_blk_cnt == 16, "场景0: 标红块数应为 16");
    chk(stat_px_cnt  == 16*64 + 12 + 8, "场景0: 候选像素数应为 1044");
    chk(mask_valid === 1'b1, "场景0: 第一遍后蒙版应可用");
    chk(alarm === 1'b0, "场景0: 第 1 遍还不该报警 (SET_N=3)");
    sweep_mask;
    expect_mask(4, 7, 2, 5, 1);
    chk(mask_got === mask_exp, "场景0: 蒙版落位与 16 块不吻合");

    run_pass;  chk(alarm === 1'b0, "场景0: 第 2 遍还不该报警");
    run_pass;  chk(alarm === 1'b1, "场景0: 第 3 遍应报警");
    first_alarm_pass = n_pass;
    // 质心: sum_bx=(4+5+6+7)*4=88, 88/16=5 (向下取整); sum_by=(2+3+4+5)*4=56, 56/16=3
    chk(stat_cx == 5*8 + 4, "场景0: 质心 X 应为块 5 的中心 (44)");
    chk(stat_cy == 3*8 + 4, "场景0: 质心 Y 应为块 3 的中心 (28)");

    // 撤警: 空场景要连续 8 遍低于 ALARM_BLK/2=3
    scene = 1;
    clear_pass = -1;
    repeat (10) begin
        run_pass;
        if (alarm === 1'b0 && clear_pass < 0) clear_pass = n_pass;
    end
    chk(clear_pass > 0, "场景1: 空场景 10 遍内应撤警");
    chk(clear_pass - first_alarm_pass >= 8, "场景1: 撤警不该早于连续 8 遍无火");
    chk(stat_blk_cnt == 0, "场景1: 不该有标红块");
    chk(stat_cx == 0 && stat_cy == 0, "场景1: 无块时质心应清零");
    sweep_mask;
    chk(mask_got === {NBLK{1'b0}}, "场景1: 蒙版应被整遍重写为全 0");

    // ---- 单块 (除数=1) 与密度边界 ----
    scene = 2;
    repeat (3) run_pass;
    chk(stat_blk_cnt == 1, "场景2: 只有 1 块达标");
    chk(stat_cx == 9*8 + 4 && stat_cy == 1*8 + 4, "场景2: 除数=1 时质心应正好落在该块中心");

    scene = 3;
    run_pass;
    chk(stat_blk_cnt == 1, "场景3: 20/64 应达标, 19/64 不该达标");
    sweep_mask;
    expect_mask(5, 5, 4, 4, 1);
    chk(mask_got === mask_exp, "场景3: 密度边界块的落位不对");

    // ---- 条带: 16 块 (2 行 × 8? 实为整行 16 块 × 2 行 = 32 块) 质心居中 ----
    scene = 4;
    repeat (4) run_pass;
    chk(stat_blk_cnt == 32, "场景4: 两整行应为 32 块");
    chk(stat_px_cnt == 32*64, "场景4: 每块满 64 候选");
    // sum_by = (3+4)*16 = 112, /32 = 3 ; sum_bx = (0+..+15)*2 = 240, /32 = 7
    chk(stat_cx == 7*8 + 4 && stat_cy == 3*8 + 4, "场景4: 满行条带质心应在两行交界处");
    chk(alarm === 1'b1, "场景4: 32 块应维持报警");

    // ---- 心跳指示器: HB_DIV=32 遍翻一次 ----
    chk(hb_tgl !== 1'bx, "心跳指示器被驱动");

    $display("---- tb_fire_detector 统计 ----");
    $display("检测遍数 = %0d, 首次报警在第 %0d 遍, 撤警在第 %0d 遍", n_pass, first_alarm_pass, clear_pass);
    $display("场景4 末尾: 块数=%0d 候选像素=%0d 质心=(%0d,%0d)", stat_blk_cnt, stat_px_cnt, stat_cx, stat_cy);
    if (errors == 0) $display("== tb_fire_detector PASS ==");
    else             $display("== tb_fire_detector FAIL (%0d) ==", errors);
    $finish;
end

initial begin
    #20_000_000;
    $display("TIMEOUT tb_fire_detector (errors=%0d)", errors);
    $finish;
end

endmodule
