`timescale 1ns/1ps
// =============================================================================
// 火焰区域分析 (Phase 3): 候选像素 -> 8×8 块密度 -> 帧统计 -> 报警
//
// 输入像素流来自 display_path 的**显示读通道** (Phase 3 决定不再为视觉另开 SDRAM
// 读通道, 理由写在 docs/implementation-plan.md 的 Phase 3 一节)。所以这里的 px/py
// 与 HDMI 正在显示的像素是同一份、同一拍 —— 蒙版落位天然对齐, 不需要坐标变换。
//
// 三层过滤 (逐像素色域判据在 fire_detector, 纯组合, 所以全部状态在这个模块):
//   1) 块内密度: 一个 8×8 块里候选像素 >= BLOCK_MIN(默认 20/64≈31%) 才标红
//      —— 孤立亮点、噪点、反光点死在这一层
//   2) 全帧面积: 标红块数 >= ALARM_BLK(默认 6 块 = 384 像素 ≈ 全帧 0.125%)
//      —— 一小团火苗级别的颜色块不足以报警
//   3) 连续多遍确认: 上一条连续满足 SET_N(3) 遍才置位; 低于 ALARM_BLK/2 连续
//      CLR_N(8) 遍才撤销。中间态用漏桶 (单遍抖动只把计数减 1, 不清零), 避免
//      火焰本身的闪烁把确认计数一遍遍打回 0。
//
// ★ 计数的时间单位是"检测遍"(display pass), 不是相机帧: 显示读 640×480@59.5Hz,
//   相机写 1856×984@13.1fps, 同一相机帧会被连续判约 4.55 遍。SET_N=3 遍 ≈ 50ms
//   ≈ 0.66 个相机帧 —— 火焰要响应快, 这个量级合适; 换算成"帧"记得除以 4.55。
//   两遍之间像素内容完全相同, 重复判定结果也相同, 不会因此抖动。
//
// 蒙版为什么要两块 RAM 乒乓: 蒙位是"整块统计完才写"的, 而显示在**同一遍**里就按
// 同样的光栅顺序读它 —— 读 (bx,by) 的那 64 拍里正好包含写 (bx,by) 的那一拍。ERAM
// 的读穿写在交叉地址上是不定值, 显示会在块边界闪噪点。乒乓后显示读"上一遍写完"的
// 那块、本遍写另一块, 读写永不落在同一块 RAM, 代价是蒙版比画面晚一遍 (≤16.8ms,
// 而这段时间里内容本来就被重复显示 4.55 次)。
//
// 质心要除法 (sum_bx/blk_cnt), 不用组合除法器: 帧末之后有行+帧消隐 ≈15000 拍空闲,
// 19 拍移位相减做得完, 省掉一整条 19 级比较器链。质心按**块**加权 (每个标红块计 1)
// 而不是按候选像素加权 —— 位宽小一半, 而 Phase 7 的坐标帧 (cmd 0x03) 只要块级精度。
// =============================================================================
module fire_region_analyzer
#(
    parameter  HBLKS       = 80,        // 块网格宽 = 640/8
    parameter  VBLKS       = 60,        // 块网格高 = 480/8
    parameter  BLOCK_SHIFT = 3,         // 块边长 2^3=8 像素; H/V 像素数必须是它的整数倍
    parameter  MASK_AW     = 13,        // 80*60=4800 块 → 13 位地址 = 8192 项 = 1 块 ERAM9K(x1)
    parameter integer BLOCK_MIN = 20,
    parameter integer ALARM_BLK = 6,
    parameter integer SET_N     = 3,
    parameter integer CLR_N     = 8,
    parameter  HB_DIV    = 32,          // 心跳分频: 59.5/(2*32)=0.93Hz, 与 led3 同口径
    parameter  ALRM_DIV  = 15           // 报警闪烁分频: 59.5/(2*15)=1.98Hz
)
(
    input  wire               video_clk,
    input  wire               rst,           // 高有效, 与 video_clk 同步

    // ---- 检测级像素流: fire_px 与 px/py 必须同拍 ----
    input  wire               px_vld,        // = de_cap, 本拍像素在有效显示区内
    input  wire [9:0]         px,            // 0..639
    input  wire [8:0]         py,            // 0..479
    input  wire               fire_px,       // 本像素是火焰候选
    input  wire               pass_end,      // 本遍最后一个有效像素 (含 px_vld)

    // ---- 帧末锁存, 两遍之间保持不变 ----
    output reg  [19:0]        stat_px_cnt,   // 本遍候选像素数
    output reg  [12:0]        stat_blk_cnt,  // 本遍过密度门限的块数
    output reg  [9:0]         stat_cx,       // 标红块质心 (像素坐标)
    output reg  [8:0]         stat_cy,
    output reg                alarm,
    output reg                hb_tgl,        // 检测遍心跳 (证明本模块在收遍)
    output reg                alrm_tgl,      // 定速翻转, 由 display_path 与 alarm 相与
    output reg                mask_valid,    // 已写过一整遍, 蒙版可用 (第一遍不叠加)

    // ---- 蒙版读口: 地址由 display_path 用同一套坐标给出, 下一拍出数据 ----
    input  wire [MASK_AW-1:0] mask_raddr,
    output wire               mask_rbit
);

// 门限参数落成定宽 localparam: 比较两侧的位宽一眼可见, 也避免在表达式里对参数做位选
localparam [6:0]  BM     = BLOCK_MIN[6:0];      // 块内候选像素门限, ≤64
localparam [12:0] AB     = ALARM_BLK[12:0];     // 报警块数门限 (SET_N/CLR_N 是 4 位计数, ≤15)
localparam [12:0] AB_LO  = AB >> 1;             // 撤警用滞后门限
localparam [3:0]  SN     = SET_N[3:0];
localparam [3:0]  CN     = CLR_N[3:0];
localparam [7:0]  HBD    = HB_DIV[7:0];
localparam [7:0]  ALD    = ALRM_DIV[7:0];
localparam [BLOCK_SHIFT-1:0] EDGE_MASK = {BLOCK_SHIFT{1'b1}};   // 块内最后一列/行

// ---------------------------------------------------------------------------
// 声明
// ---------------------------------------------------------------------------
wire [6:0] bx       = px[9:BLOCK_SHIFT];                 // 块列号
wire [5:0] by       = py[8:BLOCK_SHIFT];                 // 块行号
wire       blk_end_x = (px[BLOCK_SHIFT-1:0] == EDGE_MASK);
wire       blk_end_y = (py[BLOCK_SHIFT-1:0] == EDGE_MASK);
wire       blk_last  = px_vld & blk_end_x & blk_end_y;
wire [MASK_AW-1:0] blk_addr = by * HBLKS + bx;          // 常数乘 → 移位相加 (80=64+16)

reg  [6:0]  row_cnt [0:HBLKS-1]; // 当前"块行"(8 条扫描行) 内, 每个块列的候选像素数
reg  [19:0] cnt_px;       // 本遍候选像素
reg  [12:0] cnt_blk;      // 本遍标红块
reg  [18:0] sum_bx, sum_by;  // 标红块坐标之和 (质心分子)
reg  [3:0]  set_c, clr_c;    // 报警置位/撤销漏桶
reg  [7:0]  hb_c, al_c;      // 指示器分频
reg         cur_bank;        // 1 = 显示在读 mask1
reg  [0:0]  mask0 [0:(1<<MASK_AW)-1];
reg  [0:0]  mask1 [0:(1<<MASK_AW)-1];
reg  [0:0]  q0, q1;          // 两块 RAM 各自的读寄存器

// 移位相减除法器 (帧末启动, 19 拍完成)
reg         div_run;
reg  [4:0]  div_i;
reg  [18:0] div_num_x, div_num_y, div_qx, div_qy;
reg  [13:0] div_rx, div_ry;
reg  [12:0] div_den;

// ---------------------------------------------------------------------------
// 含当前拍的瞬时总量 —— 帧末那一拍也是块末, 用它们把最后一块算进统计
// ---------------------------------------------------------------------------
wire [6:0]  blk_total        = row_cnt[bx] + {6'b0, fire_px};
wire        blk_fire         = (blk_total >= BM);
wire [19:0] px_total         = cnt_px  + {19'b0, fire_px};
wire [12:0] blk_total_cnt    = cnt_blk + {12'b0, blk_fire};
wire [18:0] sbx_tot          = sum_bx + (blk_fire ? {12'b0, bx} : 19'd0);
wire [18:0] sby_tot          = sum_by + (blk_fire ? {13'b0, by} : 19'd0);

assign mask_rbit = cur_bank ? q1 : q0;

// 移位相减除法器的一拍: 被除数从 bit[18] 起逐位收下, 商 ≤ max(HBLKS,VBLKS) < 128
wire [13:0] rx = {div_rx[12:0], div_num_x[18]};
wire [13:0] ry = {div_ry[12:0], div_num_y[18]};
wire        gx = (rx >= {1'b0, div_den});
wire        gy = (ry >= {1'b0, div_den});
wire [7:0]  q_x = {div_qx[6:0], gx};      // 收满 19 拍时的商, 最低位是本拍的 gx
wire [7:0]  q_y = {div_qy[6:0], gy};

// ---------------------------------------------------------------------------
// 逐像素累计 -> 逐块 (像素流)
//
// 为什么是 HBLKS 个计数器而不是一个: 光栅扫描下, 一个 8×8 块的 64 个像素**不连续**
// —— 同一行里 8 个像素之后要走过同行其它块, 8 行后才回到这一块。单一累加器会把
// 同一行上相邻块的像素混在一起 (第一版就是这么错的: 16 块的场景只数出 4 块)。
// 所以按块列号 bx 开 HBLKS 个 7bit 计数器 (640 下 = 80×7 = 560 位), 每列在自己
// 那一块攒满 (blk_last) 时清零, 于是下一块行天然从 0 开始, 不需要整排清操作。
// ---------------------------------------------------------------------------
integer i;               // 只给复位里的展开循环用 (常量边界, 综合时展开)
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        for (i = 0; i < HBLKS; i = i + 1) row_cnt[i] <= 7'd0;
        cnt_px  <= 20'd0;
        cnt_blk <= 13'd0;
        sum_bx  <= 19'd0;
        sum_by  <= 19'd0;
    end
    else if (px_vld) begin
        row_cnt[bx] <= blk_last ? 7'd0 : (row_cnt[bx] + {6'b0, fire_px});
        cnt_px  <= pass_end ? 20'd0 : (cnt_px + {19'b0, fire_px});
        if (pass_end) begin
            cnt_blk <= 13'd0;                 // pass_end 那拍同时是 blk_last
            sum_bx  <= 19'd0;
            sum_by  <= 19'd0;
        end
        else if (blk_last) begin
            cnt_blk <= blk_total_cnt;
            sum_bx  <= sbx_tot;
            sum_by  <= sby_tot;
        end
    end
end

// 报警漏桶的下一拍值 (帧末更新)
wire [3:0] set_n = (blk_total_cnt >= AB)
                 ? ((set_c < SN) ? set_c + 4'd1 : set_c)
                 : ((set_c > 4'd0)       ? set_c - 4'd1 : set_c);
wire [3:0] clr_n = (blk_total_cnt < AB_LO)
                 ? ((clr_c < CN) ? clr_c + 4'd1 : clr_c)
                 : 4'd0;

// ---------------------------------------------------------------------------
// 帧末: 锁存统计 / 换蒙版银行 / 报警判决 / 启动除法 / 指示器分频
// 19 拍移位相减的迭代也放在同一个过程里 —— 除法器的状态只能有一个驱动者, 拆成两个
// always 会撞车 (stat_cx 被两处赋值, 综合直接报错、仿真取最后一个驱动)。
// pass_end 与 div_run 不会同拍: 帧末之后是消隐, 除法 19 拍就跑完, 下一遍的像素还没来。
// ---------------------------------------------------------------------------
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        stat_px_cnt  <= 20'd0;
        stat_blk_cnt <= 13'd0;
        stat_cx      <= 10'd0;
        stat_cy      <= 9'd0;
        alarm        <= 1'b0;
        mask_valid   <= 1'b0;
        set_c        <= 4'd0;
        clr_c        <= 4'd0;
        hb_c         <= 8'd0;
        al_c         <= 8'd0;
        hb_tgl       <= 1'b0;
        alrm_tgl     <= 1'b0;
        cur_bank     <= 1'b0;
        div_run      <= 1'b0;
        div_i        <= 5'd0;
        div_qx       <= 19'd0;
        div_qy       <= 19'd0;
        div_num_x    <= 19'd0;
        div_num_y    <= 19'd0;
        div_den      <= 13'd0;
        div_rx       <= 14'd0;
        div_ry       <= 14'd0;
    end
    else if (pass_end) begin
        stat_px_cnt  <= px_total;
        stat_blk_cnt <= blk_total_cnt;
        mask_valid   <= 1'b1;
        cur_bank     <= ~cur_bank;                    // 显示改读刚写完的那一块
        set_c        <= set_n;
        clr_c        <= clr_n;
        if (!alarm)       alarm <= (set_n >= SN);
        else if (clr_n >= CN) alarm <= 1'b0;
        if (blk_total_cnt == 13'd0) begin             // 没有标红块就没有质心
            stat_cx <= 10'd0;
            stat_cy <= 9'd0;
        end
        div_num_x <= sbx_tot;                          // 装载被除数, 帧末起 19 拍出商
        div_num_y <= sby_tot;
        div_den   <= blk_total_cnt;
        div_run   <= (blk_total_cnt != 13'd0);
        div_i     <= 5'd0;
        div_qx    <= 19'd0;
        div_qy    <= 19'd0;
        div_rx    <= 14'd0;
        div_ry    <= 14'd0;
        // 指示器: 事件率 59.5 遍/s, 分频后翻转才落在肉眼可判的 0.5~2Hz 区间
        hb_c <= (hb_c == HBD - 8'd1) ? 8'd0 : hb_c + 8'd1;
        if (hb_c == HBD - 8'd1) hb_tgl   <= ~hb_tgl;
        al_c <= (al_c == ALD - 8'd1) ? 8'd0 : al_c + 8'd1;
        if (al_c == ALD - 8'd1) alrm_tgl <= ~alrm_tgl;
    end
    else if (div_run) begin
        div_rx    <= gx ? (rx - {1'b0, div_den}) : rx;
        div_ry    <= gy ? (ry - {1'b0, div_den}) : ry;
        div_qx    <= {div_qx[17:0], gx};
        div_qy    <= {div_qy[17:0], gy};
        div_num_x <= {div_num_x[17:0], 1'b0};
        div_num_y <= {div_num_y[17:0], 1'b0};
        div_i     <= div_i + 5'd1;
        if (div_i == 5'd18) begin
            div_run <= 1'b0;
            // 块号 → 像素: 块起点 + 块中心, 640/480 下分别 ≤636 / ≤476
            stat_cx <= {q_x[6:0], {BLOCK_SHIFT{1'b0}}} + (1 << (BLOCK_SHIFT-1));
            stat_cy <= {q_y[5:0], {BLOCK_SHIFT{1'b0}}} + (1 << (BLOCK_SHIFT-1));
        end
    end
end

// ---------------------------------------------------------------------------
// 蒙版 RAM: 4800 块 × 1bit × 2 银行。每遍把全部块重写一遍, 所以第二遍起内容一定
// 对应"上一遍的画面"。读口每拍都走 (显示要按像素查), 写口只在 blk_last 那一拍,
// 且**一定写进没在被读的那一块** —— 这就是上面说的乒乓, 写条件的 cur_bank 极性
// 必须与 mask_rbit 的选择极性相反, 写反了就退化成交叉读写 (不定值)。
// ---------------------------------------------------------------------------
always @(posedge video_clk) begin
    if (blk_last & ~cur_bank) mask1[blk_addr] <= {blk_fire};
    q1 <= mask1[mask_raddr];
end

always @(posedge video_clk) begin
    if (blk_last & cur_bank) mask0[blk_addr] <= {blk_fire};
    q0 <= mask0[mask_raddr];
end

endmodule
