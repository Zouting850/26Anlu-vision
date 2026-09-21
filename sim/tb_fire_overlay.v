`timescale 1ns/1ps
// =============================================================================
// Phase 3 蒙版落位端到端仿真
//
// 例化的是**真实**链路 (与 tb_display_path 同一套骨架): frame_read_write(真实异步
// FIFO) + frame_buffer_ctrl + display_path(内含 rgb_to_ycbcr / fire_detector /
// fire_region_analyzer / video_delay)。只有片内 SDRAM 控制器(加密)与摄像头/显示时序
// 用模型顶替, 几何缩到 96×48 = 12×6 块 (块边长仍是 8 像素)。
//
// 这一段要证明的是"红块落在火焰上" —— 上板之后这件事只能靠眼睛看, 所以拆成六条:
//   1) 该标红的块 (亮橙区, 4×3=12 块) 显示成半透明红, 且逐像素位置正确
//   2) 不该标红的区域原样显示: 暗橙块 (Y 不到 180)、肤色块 (Cr-Cb=57 < 60)、灰墙
//   3) 右上角 4 格 HUD 只出现在那 4 列, 灭着的格子是黑底, 别处不许被 HUD 染色,
//      且四格在判读窗口里都真的亮过 (心跳/候选/成块/报警各自被驱动过)
//   4) 切到彩条档 (mode=3) 后输出仍严格落在 8 色表内 —— 蒙版若漏进调试档就会变成表外
//      的混色 (条带本身的落位由 tb_display_path 逐像素验, 这里不重复)
//   5) 读写突发不同拍 (Phase 2 的仲裁不变量, 加了检测负载后必须仍成立)
//   6) 每个判读窗口前把最近提交的整块缓冲和场景对账一遍 (verify_mem), 用于把
//      "写侧写坏了"和"读侧读错位"分开
//
// 实测通过量: 46080 个像素逐像素判读 (约 10 遍), 底图/落位/HUD/撞拍/空取/显存对账全零。
//
// 显示档取 mode=0 (原图): 这一档底图就是显存里的 RGB, 没有 YCbCr 往返, 判读是逐位
// 相等不留容差 (灰度/假彩两档的对齐由 tb_display_path 负责)。
// 蒙版比画面晚一遍 (乒乓), 所以每段先跑几遍让统计与蒙版稳定, 再开始逐像素判读。
// =============================================================================
module tb_fire_overlay;

localparam H_ACT = 96,  H_BL = 32, HT = H_ACT + H_BL;    // 128
localparam V_ACT = 48,  V_BL = 8,  VT = V_ACT + V_BL;    // 56
localparam VSP   = 2;
localparam FWORDS = H_ACT * V_ACT;                       // 4608 = 18×256 ✓ 突发的整数倍
localparam BUF0 = 0, BUF1 = FWORDS;
// 显存模型必须容得下**两块**缓冲, 地址位宽按帧长算而不是抄常数: 这里曾经写死 13 位
// (8192 字, 128×64 几何时代的数), 缩几何后 2×4608 = 9216 越过 8192, 越界部分被
// `App_wr_addr[SDM_AW-1:0]` 折回 buf0 头部 —— 于是 buf1 的帧尾永远读成 X, 而 buf0 头部
// 被帧尾内容污染。现象是"底图大面积错位 + 蒙版落位错", 长得和真正的对齐 bug 一模一样,
// 但和被测 RTL 完全无关。verify_mem() 就是为了让这类事在十秒内暴露出来。
localparam SDM_AW = $clog2(2*FWORDS);                     // 14 → 16384 字
localparam HBLKS = H_ACT/8, VBLKS = V_ACT/8, NBLK = HBLKS*VBLKS;

reg mem_clk = 1'b0, video_clk = 1'b0, cam_clk = 1'b0;
reg rst = 1'b1;
always #4    mem_clk    = ~mem_clk;    // 125MHz
always #20   video_clk  = ~video_clk;  //  25MHz
always #20.5 cam_clk    = ~cam_clk;   // 写帧 378us > 读帧 224us, 与真机同向 (读比写快)

integer errors = 0;

// ---------------------------------------------------------------------------
// 片内 SDRAM 控制器的行为替身
// ---------------------------------------------------------------------------
reg [31:0] sdm [0:(1<<SDM_AW)-1];
reg        Sdr_rd_en_r = 1'b0;
reg [31:0] Sdr_rd_dout_r = 32'd0;
wire       Sdr_init_done = ~rst;

wire        App_wr_en;
wire [20:0] App_wr_addr;
wire [31:0] App_wr_din;
wire [3:0]  App_wr_dm;
wire        App_rd_en;
wire [20:0] App_rd_addr;
wire        Sdr_rd_en;
wire [31:0] Sdr_rd_dout;
assign Sdr_rd_en = Sdr_rd_en_r;   assign Sdr_rd_dout = Sdr_rd_dout_r;

always @(posedge mem_clk) begin
    if (App_wr_en) sdm[App_wr_addr[SDM_AW-1:0]] = App_wr_din;
    Sdr_rd_en_r   <= App_rd_en;
    Sdr_rd_dout_r <= sdm[App_rd_addr[SDM_AW-1:0]];
end

// 显存模型必须先清零: 第一遍显示读会在摄像头写满之前就去读, 读到 X 会让 fire_px 变 X,
// 而 X 一旦进累加器就永久粘住 (cnt_blk <= cnt_blk + X 永远是 X)。真机上电后 SDRAM 里
// 是"杂乱但有定义"的电平, 不存在 X, 所以这是 TB 的事, 不是 RTL 的事。
integer sdm_i;
initial for (sdm_i = 0; sdm_i < (1<<SDM_AW); sdm_i = sdm_i + 1) sdm[sdm_i] = 32'd0;

// Phase 2 的仲裁不变量: 行为级存储模型对撞拍照单全收, 只有这条断言能报
integer collide = 0;
always @(posedge mem_clk) begin
    if (!rst && App_wr_en && App_rd_en) begin
        collide = collide + 1;
        if (collide < 5)
            $display("FAIL t=%0t 读写突发撞在同一拍: wr=%0d rd=%0d", $time, App_wr_addr, App_rd_addr);
    end
end

// ---------------------------------------------------------------------------
// 被测链路
// ---------------------------------------------------------------------------
reg         cam_write_req = 1'b0;
reg         cam_write_en  = 1'b0;
reg  [31:0] cam_write_data = 32'd0;
wire        cam_write_req_ack;
wire        frame_commit;
wire [1:0]  fb_wr_index, fb_rd_index;

reg         disp_read_req = 1'b0;
wire        disp_read_req_ack;
wire        disp_read_en;
wire [31:0] disp_read_data;
reg  [1:0]  mode = 2'd0;
reg  [1:0]  chk_mode = 2'd0;             // 本遍开始时锁存 (帧中途换模式会混两种输出)

wire hs0, vs0, de0;
wire hs_o, vs_o, de_o;
wire [23:0] vout;

frame_read_write #(
    .ADDR_BITS(21), .MEM_DATA_BITS(32), .READ_DATA_BITS(32), .WRITE_DATA_BITS(32)
) u_frw (
    .rst(rst), .mem_clk(mem_clk),
    .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(1'b0), .Sdr_busy(1'b0),
    .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
    .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
    .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
    .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
    .write_clk(cam_clk), .write_req(cam_write_req), .write_req_ack(cam_write_req_ack),
    .write_finish(frame_commit),
    .write_addr_0(BUF0), .write_addr_1(BUF1), .write_addr_2(BUF0), .write_addr_3(BUF0),
    .write_addr_index(fb_wr_index), .write_len(FWORDS),
    .write_en(cam_write_en), .write_data(cam_write_data),
    .read_clk(video_clk), .read_req(disp_read_req), .read_req_ack(disp_read_req_ack),
    .read_finish(),
    .read_addr_0(BUF0), .read_addr_1(BUF1), .read_addr_2(BUF0), .read_addr_3(BUF0),
    .read_addr_index(fb_rd_index), .read_len(FWORDS),
    .read_en(disp_read_en), .read_data(disp_read_data)
);

frame_buffer_ctrl u_fb (
    .mem_clk(mem_clk), .rst(rst), .wr_finish(frame_commit),
    .wr_index(fb_wr_index), .rd_index(fb_rd_index)
);

display_path #(
    .DATA_WIDTH(24), .PIPE_LAT(2),
    .HBLKS(HBLKS), .VBLKS(VBLKS), .BLOCK_SHIFT(3), .MASK_AW(7),
    // HUD 两格指示器的心跳/闪烁分频在真机上是 32/15 遍 (0.93Hz / 1.98Hz), 本 TB 的
    // 判读窗口只有几遍, 撞不上那个周期; 分频本来就是给"人眼看得清"用的, 与落位无关,
    // 所以这里缩到 4/3 遍, 让四格在判读窗口里真的各亮过一次 (分频本身的正确性由
    // tb_fire_detector 在分析器层面查)。
    .FIRE_HB_DIV(4), .FIRE_ALRM_DIV(3)
) u_dp (
    .video_clk(video_clk), .rst(rst),
    .read_data(disp_read_data),
    .hs_i(hs0), .vs_i(vs0), .de_i(de0), .mode(mode),
    .read_en(disp_read_en),
    .hs_o(hs_o), .vs_o(vs_o), .de_o(de_o), .vout_data(vout)
);

// ---------------------------------------------------------------------------
// 场景: 0=墙 1=亮橙(该标红, 块 2..5 × 1..3) 2=暗橙(不该) 3=肤色带(不该)
// 区域函数同时驱动激励和判读期望, 避免两张表各写一遍互相漂移。
// HUD 占块列 8..11 的块行 0..1, 与火焰的块列 2..5 不重叠。
// ---------------------------------------------------------------------------
integer scene = 0;
function automatic integer area_of(input integer x, input integer y);
    begin
        if (scene == 1)                                    area_of = 0;   // 全灰墙
        else if (x >= 16 && x < 48 && y >= 8  && y < 32)   area_of = 1;   // 亮橙火焰
        else if (x >= 56 && x < 80 && y >= 8  && y < 32)   area_of = 2;   // 暗橙
        else if (y >= 40)                                  area_of = 3;   // 肤色带
        else                                               area_of = 0;   // 灰墙
    end
endfunction
function automatic [23:0] area_rgb(input integer a);
    case (a)
        1:       area_rgb = 24'hFEBE1E;      // (255,190,30) 亮橙: Y=180 Cr=168 Cb=48
        2:       area_rgb = 24'hFF8C00;      // (255,140,0)  暗橙: Y=152 不到亮度坎
        3:       area_rgb = 24'hFAB496;      // (250,180,150) 偏红肤色: Cr-Cb=57 < 60
        default: area_rgb = 24'h808080;      // 灰墙
    endcase
endfunction
// 显存第 idx 个字"应该"是什么 —— 判读和对账共用这一个函数, 不写两张表
function automatic [31:0] exp_word(input integer idx);
    begin
        if (idx < 0 || idx >= FWORDS) exp_word = 32'hDEAD_BEAD;
        else exp_word = {area_rgb(area_of(idx % H_ACT, idx / H_ACT)), 8'd0};
    end
endfunction
function automatic [23:0] blend_of(input [23:0] c);
    integer rr, gg, bb;
    begin
        rr = (c[23:16] + 255) / 2;          // 先落整数再拼接: 除法结果位宽不定,
        gg = c[15:8] / 2;                   // 直接进 {} 会报 indefinite width
        bb = c[7:0] / 2;
        blend_of = {rr[7:0], gg[7:0], bb[7:0]};
    end
endfunction
// 读通道供给检查: 显示每拍都要一个字, 若读 FIFO 空了还去取 (rd_en_s = ~empty & re),
// 那一拍就被丢掉 —— 整帧从此错位一个字, 块网格也跟着平移。这是"蒙版比火焰偏一列"
// 这类现象的根因所在, 所以把它做成断言而不是靠猜。
wire rf_empty = u_frw.read_buf.empty_flag;
integer underrun = 0, underrun_max = 0;
always @(posedge video_clk) begin
    if (!rst && disp_read_en && rf_empty) begin
        underrun = underrun + 1;
        if (underrun == underrun_max + 1) begin
            underrun_max = underrun;
            $display("FAIL t=%0t 显示取数遇到空 FIFO (第 %0d 次), 本帧从此错位", $time, underrun);
        end
    end
end

// ---------------------------------------------------------------------------
// 摄像头写入进程
//   两条节拍约束, 都是"看起来像 RTL bug 其实是 TB 节拍不对"的坑:
//   1) 写完一帧后必须等 frame_commit 再发下一个 write_req: frame_fifo_write 只有攒够
//      一整轮突发 (256 字) 才向 SDRAM 发突发, 帧尾那一轮是在最后一个像素进 FIFO 之后
//      才发出去的 (≈2~5us)。只留 120 个 cam_clk 的话, 写 FSM 在 S_CHECK_FIFO /
//      S_WRITE_BURST 时被新的 write_req 拽回 S_ACK —— 那一帧永不提交, 乒乓不换手。
//   2) 帧周期必须**大于一个显示遍周期**。双缓冲没有任何"写头别追上读头"的保护
//      (见 frame_buffer_ctrl 头注): 读侧在请求那拍锁到第 k 块, 写侧要到第 k+2 帧才会
//      回到这块, 所以条件就是 T_frame > T_pass。真机是 76ms vs 16.8ms (读快 4.55 倍),
//      缩几何到 96×48 后写一帧只要 189us 而一遍要 287us —— 反过来写头每 1.5 遍就追上
//      读头一次, 显示读到正在被覆写的块, 现象是"底图大面积错位 + 蒙版落位错",
//      而 collide/underrun 两条断言都抓不到 (数据一直都在, 只是内容在半途被人改了)。
//   所以帧间消隐按"显示遍"来留, 不是拍一个 cam_clk 计数。
// ---------------------------------------------------------------------------
integer wx, wy, c_prev;
integer wdone = 0, cdone = 0;
always @(posedge mem_clk) if (frame_commit) cdone = cdone + 1;
initial begin
    wait (rst === 1'b0);
    forever begin
        @(negedge cam_clk); cam_write_req <= 1'b1;
        do @(negedge cam_clk); while (cam_write_req_ack !== 1'b1);
        cam_write_req <= 1'b0;
        // 仿真里的"行/场消隐": 必须给 fifo_aclr 留出释放时间。S_ACK 期间 fifo_aclr 是
        // 拉高的, 头 2 个字若在它撤掉前进 FIFO 就会被清掉 —— 帧长是 256 的整数倍时,
        // 这 2 个字会让最后一轮突发永远差 2 个字攒不齐 (实测 rdusedw 卡在 254, 而
        // burst_need=256), 于是 write_finish 永远不来、乒乓永不换手、显示和写抢同一块。
        // 真机不用管: ov5640_delay 在 vsync 沿发 write_req, 第一个有效像素在几十行之后。
        repeat (16) @(negedge cam_clk);
        for (wy = 0; wy < V_ACT; wy = wy + 1)
            for (wx = 0; wx < H_ACT; wx = wx + 1) begin
                cam_write_data <= {area_rgb(area_of(wx, wy)), 8'd0};
                cam_write_en   <= 1'b1;
                @(negedge cam_clk);
                cam_write_en   <= 1'b0;
            end
        wdone = wdone + 1;                      // 本帧像素已全部进写 FIFO
        c_prev = cdone;                         // 等这一帧真的提交掉
        while (cdone == c_prev) @(negedge cam_clk);
        repeat (VT*HT) @(negedge video_clk);    // 帧间消隐 = 整整一个显示遍 (≈287us)
    end
end

// ---------------------------------------------------------------------------
// 显示时序发生器 + 读请求 (与 tb_display_path 同法)
// ---------------------------------------------------------------------------
reg [7:0] hx;
reg [6:0] vy;
reg vs0_r = 1'b0, de0_r = 1'b0, hs0_r = 1'b0;
assign vs0 = vs0_r; assign de0 = de0_r; assign hs0 = hs0_r;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        vs0_r <= 0; de0_r <= 0; hs0_r <= 0; hx <= 0; vy <= 0;
    end
    else begin
        hx <= (hx == HT-1) ? 8'd0 : hx + 8'd1;
        if (hx == HT-1) vy <= (vy == VT-1) ? 7'd0 : vy + 7'd1;
        vs0_r <= (vy < VSP);
        de0_r <= (vy >= V_BL) && (vy < V_BL + V_ACT) && (hx < H_ACT);
        hs0_r <= (hx >= 4) && (hx < 12);
    end
end
reg vs0_d;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin vs0_d <= 1'b0; disp_read_req <= 1'b0; end
    else begin
        vs0_d <= vs0_r;
        if (vs0_d & ~vs0_r)          disp_read_req <= 1'b1;
        else if (disp_read_req_ack)  disp_read_req <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// 逐像素判读
// ---------------------------------------------------------------------------
integer n_out = 0, n_chk = 0, err_base = 0, err_blk = 0, err_hud = 0;
integer n_prt = 0, seg = 0;               // 打印额度 / 第几个判读窗口
integer gx, gy, gbx, gby, gslot, area;
reg [23:0] exp_c;
reg        checking = 1'b0, hit, bar_seen;
reg [7:0]  bar_bits = 8'd0;               // 彩条档: 8 种颜色是否都出现过
reg [3:0]  kind_seen = 4'b0000;
reg [3:0]  hud_seen  = 4'b0000;            // HUD 四格各自至少亮过一次

always @(posedge video_clk) if (!rst && (vs0_d & ~vs0_r)) chk_mode = mode;

// 判读一律在 negedge 采样: vout / cx / mask_rbit 都是 posedge 更新的寄存器, 在
// posedge 上跨模块读它们会撞上调度顺序 (读到的是上一拍的值), 逐像素对位这种事
// 差一拍就全错, 所以判读点挪到时钟中点, 所有信号都稳定。
always @(negedge video_clk) begin
    if (!rst) begin
        if (vs_o && !vs_o_d2) n_out = 0;
        if (de_o) begin
            n_out = n_out + 1;
            if (checking && n_out <= FWORDS) begin
                gx = (n_out-1) % H_ACT;
                gy = (n_out-1) / H_ACT;
                gbx = gx / 8;  gby = gy / 8;
                n_chk = n_chk + 1;
                if (gy < 16 && gbx >= HBLKS-4) begin
                    // ---- HUD 窗口: 亮着必须是本格颜色, 灭着必须是黑底 ----
                    kind_seen[3] = 1'b1;
                    gslot = gbx - (HBLKS-4);
                    case (gslot)
                        0:       hit = (vout === 24'h0000FF);
                        1:       hit = (vout === 24'hFFFF00);
                        2:       hit = (vout === 24'hFF8000);
                        default: hit = (vout === 24'hFF0000);
                    endcase
                    if (hit) hud_seen[gslot] = 1'b1;
                    else if (vout !== 24'h000000) begin
                        err_hud = err_hud + 1;
                        if (err_hud < 6)
                            $display("FAIL t=%0t HUD 格%0d (%0d,%0d) = %h 既不是黑也不是本格颜色",
                                     $time, gslot, gx, gy, vout);
                    end
                end
                else if (chk_mode == 2'd3) begin
                    // 彩条档: 条带落位由 tb_display_path 负责逐像素验, 这里只要求输出
                    // **一定落在 8 色表内**。蒙版若在彩条档漏出来, 颜色就变成表外的混色
                    // (半透明红), 这一条就是"彩条档蒙版整体消失"的判据。
                    bar_seen = 1'b0;
                    case (vout)
                        24'hFFFFFF: begin bar_seen = 1'b1; bar_bits[0] = 1'b1; end
                        24'hFFFF00: begin bar_seen = 1'b1; bar_bits[1] = 1'b1; end
                        24'h00FFFF: begin bar_seen = 1'b1; bar_bits[2] = 1'b1; end
                        24'h00FF00: begin bar_seen = 1'b1; bar_bits[3] = 1'b1; end
                        24'hFF00FF: begin bar_seen = 1'b1; bar_bits[4] = 1'b1; end
                        24'hFF0000: begin bar_seen = 1'b1; bar_bits[5] = 1'b1; end
                        24'h0000FF: begin bar_seen = 1'b1; bar_bits[6] = 1'b1; end
                        24'h000000: begin bar_seen = 1'b1; bar_bits[7] = 1'b1; end
                        default:    begin bar_seen = 1'b0; end
                    endcase
                    if (!bar_seen) begin
                        err_base = err_base + 1;
                        if (n_prt < 8)
                            $display("FAIL t=%0t [%0d] 彩条档 (%0d,%0d) 出现表外颜色 %h (蒙版漏进调试档?)",
                                     $time, seg, gx, gy, vout);
                    end
                end
                else begin
                    area = area_of(gx, gy);
                    exp_c = area_rgb(area);
                    if (area == 1) exp_c = blend_of(area_rgb(1));          // 该被标红
                    if (vout !== exp_c) begin
                        if (area == 1) err_blk = err_blk + 1;
                        else           err_base = err_base + 1;
                        // 两类各给 4 条打印额度: 混用一个额定时, 先爆的那类会把另一类
                        // 的现场全挤掉, 上一轮就是这样只看到 7 条底图错、看不到落位错。
                        if (n_prt < 8) begin
                            n_prt = n_prt + 1;
                            $display("FAIL t=%0t [%0d] (%0d,%0d) area=%0d got=%h 期望=%h | DUT: cx_q=%0d cy_q=%0d rbit=%b mrab=%0d mvalid=%b blk=%0d px=%0d alarm=%b",
                                     $time, seg, gx, gy, area, vout, exp_c,
                                     u_dp.cx_q, u_dp.cy_q,
                                     u_dp.mask_rbit, u_dp.mask_raddr, u_dp.mask_valid,
                                     u_dp.u_fire_region.stat_blk_cnt,
                                     u_dp.u_fire_region.stat_px_cnt,
                                     u_dp.u_fire_region.alarm);
                        end
                    end
                    kind_seen[area] = 1'b1;
                end
            end
        end
    end
end
reg vs_o_d2 = 1'b0;
always @(posedge video_clk) vs_o_d2 <= vs_o;

// ---------------------------------------------------------------------------
// 显存对账: 每个判读窗口开始前, 把"最近提交的那一块缓冲"整个和场景比一遍。
//   逐像素判读只能看"显示出来的对不对", 分不清"写侧没写对"和"读侧读错位置"; 这一遍
//   比的是显存本身, 出问题时第一时间把两件事分开。上一轮的底图大面积错位就是这么定位
//   到 TB 自己的显存数组越界上的 (见 SDM_AW 的注释), 而当时的现象看起来完全像 RTL bug。
//   只在场景稳定期调用 (run_passes 之前 scene/mode 都不再变)。
// ---------------------------------------------------------------------------
integer last_committed = 0;
integer mem_err = 0;
always @(posedge mem_clk) if (frame_commit) last_committed = fb_wr_index;

task automatic verify_mem(input [8*8-1:0] tag);
    integer k, bad, fb;
    begin
        bad = 0; fb = -1;
        for (k = 0; k < FWORDS; k = k + 1)
            if (sdm[last_committed*FWORDS + k] !== exp_word(k)) begin
                bad = bad + 1;
                if (fb < 0) fb = k;
            end
        if (bad != 0) mem_err = mem_err + 1;
        $display("[mem] %0s buf%0d: 与场景不符的字 = %0d 首个 idx=%0d(行%0d 列%0d) got=%h exp=%h",
                 tag, last_committed, bad, fb, fb/H_ACT, fb%H_ACT,
                 sdm[last_committed*FWORDS + (fb < 0 ? 0 : fb)], exp_word(fb));
    end
endtask

// ---------------------------------------------------------------------------
// 流程控制
//   门槛一律用 cdone (提交次数), 不用"推完像素"的计数 —— 乒乓只在提交时换手,
//   显示看到的永远是最近一次提交完成的那一帧。
//   每个判读窗口结束后立刻报本窗口的增量, 不等最后汇总 —— 一次性看总数只能知道
//   "错了", 分段才看得出是哪一段、哪一类错的。
// ---------------------------------------------------------------------------
integer b0, k0, h0;
task automatic run_passes(input integer np, input [8*24-1:0] tag);
    begin
        checking = 1'b0;
        b0 = err_base; k0 = err_blk; h0 = err_hud;
        repeat (np * VT * HT + 8) @(negedge video_clk);      // 先让统计与蒙版稳定
        verify_mem(tag);
        $display("[info] t=%0t seg%0d %0s: 稳定 %0d 遍后开始判读 (commit=%0d)",
                 $time, seg + 1, tag, np, cdone);
        seg = seg + 1;
        checking = 1'b1;
        repeat (2 * VT * HT)      @(negedge video_clk);      // 判读两遍
        checking = 1'b0;
        $display("[info] t=%0t seg%0d %0s: 本窗口增量 底图=%0d 落位=%0d HUD=%0d",
                 $time, seg, tag, err_base - b0, err_blk - k0, err_hud - h0);
    end
endtask

// 换场景: 先等当前这一帧提交掉 (改 scene 时帧可能写到一半, 那一帧是混合内容),
// 再等 3 帧干净内容提交, 最后放过两个显示遍让读侧切到新块。
task automatic set_scene(input integer s);
    integer n0;
    begin
        checking = 1'b0;
        n0 = cdone;
        while (cdone == n0) @(negedge cam_clk);
        scene = s;
        n0 = cdone;
        while (cdone < n0 + 3) @(negedge cam_clk);
        repeat (2 * VT * HT + 16) @(negedge video_clk);
    end
endtask

initial begin
    repeat (40) @(negedge video_clk);
    rst = 1'b0;
    scene = 0;  mode = 2'd0;
    while (cdone < 2) @(negedge cam_clk);
    run_passes(4, "场景0/mode0");
    run_passes(2, "报警稳态");
    mode = 2'd3;
    run_passes(2, "彩条档");
    mode = 2'd0;
    run_passes(2, "回原图");
    set_scene(1);
    run_passes(4, "全灰墙");
    repeat (4 * VT * HT) @(negedge video_clk);   // 等判读段落彻底结束
    finish_run;
end

initial begin
    #22_000_000;
    $display("TIMEOUT tb_fire_overlay (判读到 t=%0t, 已 check %0d 像素)", $time, n_chk);
    finish_run;
end

task automatic finish_run;
    begin
        $display("---- tb_fire_overlay 统计 ----");
        $display("逐像素判读 = %0d 个 (约 %0d 遍), 写侧推完帧数 = %0d, 提交 = %0d",
                 n_chk, n_chk/FWORDS, wdone, cdone);
        $display("底图/彩条错 = %0d, 蒙版落位错 = %0d, HUD 错 = %0d, 读写撞拍 = %0d, 读通道空取 = %0d, 显存对账不合格 = %0d/5",
                 err_base, err_blk, err_hud, collide, underrun, mem_err);
        $display("覆盖区域类别 (墙/火/暗橙/HUD窗) = %b, HUD 亮过的格 = %b, 彩条档见过的颜色 = %b", kind_seen, hud_seen, bar_bits);
        errors = (collide != 0) + (err_base != 0) + (err_blk != 0) + (err_hud != 0) + (underrun != 0)
               + (mem_err != 0) + (bar_bits != 8'hFF)
               + (kind_seen != 4'b1111) + (hud_seen != 4'b1111) + (n_chk < 3*FWORDS);
        if (collide != 0)         $display("FAIL: 读写突发撞在同一拍, 帧内容会错位");
        if (underrun != 0)        $display("FAIL: 显示取数撞上空 FIFO %0d 次, 帧与块网格整体错位", underrun);
        if (mem_err != 0)         $display("FAIL: 有 %0d 个窗口的显存内容与场景不符 —— 写侧或 TB 模型的问题, 先别查检测器", mem_err);
        if (bar_bits != 8'hFF)    $display("FAIL: 彩条档只见过 %b, 彩条计数器没跑满 8 档", bar_bits);
        if (err_base != 0)        $display("FAIL: 不该改色的像素被改了 (误报或对齐错)");
        if (err_blk  != 0)        $display("FAIL: 该标红的像素没标红 (漏检或蒙版落位错)");
        if (err_hud  != 0)        $display("FAIL: HUD 窗内出现了黑/本格颜色以外的值");
        if (kind_seen != 4'b1111) $display("FAIL: 有四类区域没被逐像素判读到, 覆盖不足");
        if (hud_seen  != 4'b1111) $display("FAIL: HUD 有四格从没亮过, 指示器没被验证");
        if (n_chk < 3*FWORDS)     $display("FAIL: 判读像素太少 (%0d), 读通路可能没跑起来", n_chk);
        if (errors == 0) $display("== tb_fire_overlay PASS ==");
        else             $display("== tb_fire_overlay FAIL (%0d) ==", errors);
        $finish;
    end
endtask


endmodule
