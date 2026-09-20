`timescale 1ns/1ps
// =============================================================================
// 显示通路端到端仿真 (Phase 2 最关键的一条证据链)
//
// 例化的是**真实**模块: frame_read_write(含真实异步 FIFO RTL) + frame_buffer_ctrl
// + display_path(内含 rgb_to_ycbcr 与 video_delay #(.PIPE_LAT(2)))。只有片内
// SDRAM 控制器(加密)和摄像头/显示时序用模型顶替, 并把几何缩小到 64x8 帧。
//
// 要证明的事: 像素经过 “写进帧缓冲 -> 乒乓换 bank -> 突发读出 -> 跨时钟 FIFO ->
// YCbCr 流水线 -> 时序对齐” 之后没有错列、错行、通道 skew。PIPE_LAT 只要差一拍,
// 下面的位置自洽检查就会立刻报错。
//
// 每个像素自带位置指纹:
//   R = {x[5:0], gen[1:0]}    -> x = R[7:2], gen = R[1:0]
//   G = {5'b0,    y[2:0]}     -> y = G[2:0]
//   B = {x[5:0], y[2:0], 1'b}
// 三通道必须互相自洽 (B 要等于用 R、G 解出的 x、y 拼出来的值) —— 这一条专抓
// “某一路比别的路早/晚几拍”; gen 字段用来确认读到的是完整一帧。
// =============================================================================
module tb_display_path;

localparam H_ACT = 64, H_BL = 16, HT = H_ACT + H_BL;      // 80
localparam V_ACT = 8,  V_BL = 4,  VT = V_ACT + V_BL;      // 12
localparam VSP   = 2;                                      // vs 高电平行数
localparam FWORDS = H_ACT * V_ACT;                        // 512 = 一帧字
localparam BUF0 = 0, BUF1 = FWORDS;                       // 乒乓基址
localparam SDM_AW = 11;                                   // 2048 字够放两块

localparam NRAW = 12, NYCC = 8, NGRAY = 8, NBAR = 8;       // 各模式跑帧数

reg mem_clk = 1'b0, video_clk = 1'b0, cam_clk = 1'b0;
reg rst = 1'b1;
always #4    mem_clk    = ~mem_clk;    // 125MHz
always #20   video_clk  = ~video_clk;  // 25MHz
always #20.5 cam_clk    = ~cam_clk;    // 41ns 周期, 与另两个时钟异步

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
assign Sdr_rd_en = Sdr_rd_en_r;  assign Sdr_rd_dout = Sdr_rd_dout_r;

always @(posedge mem_clk) begin
    if (App_wr_en) sdm[App_wr_addr[SDM_AW-1:0]] = App_wr_din;
    Sdr_rd_en_r   <= App_rd_en;
    Sdr_rd_dout_r <= sdm[App_rd_addr[SDM_AW-1:0]];
end

// 读写突发必须严格互斥 —— 这是 O_wr_busy/O_rd_busy 交叉锁存在的全部意义, 也是行为级
// 存储模型唯一查不出来的错误 (两条 App_*_en 同时拉高时它只会照单全收)。
// into_burst 的实现被为时序改写过, 这条不变量就是那次改动的验收判据。
integer collide = 0;
always @(posedge mem_clk) begin
    if (!rst && App_wr_en && App_rd_en) begin
        collide = collide + 1;
        if (collide < 5)
            $display("FAIL t=%0t 读写突发撞在同一拍: wr_addr=%0d rd_addr=%0d",
                     $time, App_wr_addr, App_rd_addr);
    end
end

// ---------------------------------------------------------------------------
// 被测: 乒乓 + 帧读写仲裁 + 显示通路
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
reg        vs_o_d2 = 1'b0;   // 输出帧起始检测用
integer pause_chk = 1;           // 模式切换后先丢一帧再判
integer gen_seen[0:3];
integer buf_seen[0:1];

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

display_path #(.DATA_WIDTH(24), .PIPE_LAT(2)) u_dp (
    .video_clk(video_clk), .rst(rst),
    .read_data(disp_read_data),
    .hs_i(hs0), .vs_i(vs0), .de_i(de0), .mode(mode),
    .read_en(disp_read_en),
    .hs_o(hs_o), .vs_o(vs_o), .de_o(de_o), .vout_data(vout)
);

// ---------------------------------------------------------------------------
// 编码函数 (先转 integer 再乘, 避开 Verilog 表达式被拉成无符号的坑)
// ---------------------------------------------------------------------------
function automatic [7:0] enc_r(input integer x, input integer g);
    enc_r = {x[5:0], g[1:0]};
endfunction
function automatic [7:0] enc_g(input integer y);
    enc_g = {5'b0, y[2:0]};
endfunction
// B 只有 8 位: 放 x 的低 4 位 + y 的 3 位 + 1 位填充 (x 的高 2 位由 R 携带)
function automatic [7:0] enc_b(input integer x, input integer y);
    enc_b = {x[3:0], y[2:0], 1'b0};
endfunction

// ---------------------------------------------------------------------------
// 摄像头写入进程: 每帧 FWORDS 像素, 每 8 个 cam_clk 一个
//   写帧周期 = 512*8*41ns = 168us, 读帧周期 = 12*80*40ns = 38.4us -> 4.4 倍,
//   与真实 OV5640 13.1fps / HDMI 59.5Hz 的 4.55 倍一致
// ---------------------------------------------------------------------------
integer wgen = 0;
integer wr_i, wx, wy;
initial begin
    wait (rst === 1'b0);
    forever begin
        @(negedge cam_clk); cam_write_req <= 1'b1;
        do @(negedge cam_clk); while (cam_write_req_ack !== 1'b1);
        cam_write_req <= 1'b0;
        @(negedge cam_clk);
        for (wy = 0; wy < V_ACT; wy = wy + 1) begin
            for (wx = 0; wx < H_ACT; wx = wx + 1) begin
                for (wr_i = 0; wr_i < 7; wr_i = wr_i + 1) @(negedge cam_clk);
                cam_write_data <= { enc_r(wx, wgen), enc_g(wy), enc_b(wx, wy), 8'd0 };
                cam_write_en   <= 1'b1;
                @(negedge cam_clk);
                cam_write_en   <= 1'b0;
                for (wr_i = 0; wr_i < 7; wr_i = wr_i + 1) @(negedge cam_clk);
            end
        end
        // 帧间消隐: 最后一段突发还要 ~2us 才落进 S_END, 真实摄像头有 76ms 间隔
        repeat (500) @(negedge cam_clk);
        wgen = (wgen + 1) % 4;
    end
end

// ---------------------------------------------------------------------------
// 显示时序发生器
// ---------------------------------------------------------------------------
reg [7:0] hx;
reg [3:0] vy;
reg vs0_r = 1'b0, de0_r = 1'b0, hs0_r = 1'b0;
assign vs0 = vs0_r; assign de0 = de0_r; assign hs0 = hs0_r;

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        vs0_r <= 1'b0; de0_r <= 1'b0; hs0_r <= 1'b0; hx <= 8'd0; vy <= 4'd0;
    end
    else begin
        hx <= (hx == HT-1) ? 8'd0 : hx + 8'd1;
        if (hx == HT-1) vy <= (vy == VT-1) ? 4'd0 : vy + 4'd1;
        vs0_r <= (vy < VSP);
        de0_r <= (vy >= V_BL) && (vy < V_BL + V_ACT) && (hx < H_ACT);
        hs0_r <= (hx >= 4) && (hx < 12);
    end
end

// vs 下沿发读请求 (与 video_timing_data 同法)
reg vs0_d;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin vs0_d <= 1'b0; disp_read_req <= 1'b0; end
    else begin
        vs0_d <= vs0_r;
        if (vs0_d & ~vs0_r)         disp_read_req <= 1'b1;
        else if (disp_read_req_ack) disp_read_req <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// 记账: 每块缓冲最近一次提交的帧代标 (从存储模型最后一个字里读 gen, 免竞态)
// ---------------------------------------------------------------------------
integer committed_gen [0:1];
integer ready = 0;
integer lastw;
integer n_commit = 0;
initial begin
    committed_gen[0] = -1; committed_gen[1] = -1;
    gen_seen[0]=0; gen_seen[1]=0; gen_seen[2]=0; gen_seen[3]=0;
    buf_seen[0]=0;  buf_seen[1]=0;
    wait (rst === 1'b0);
    forever begin
        @(posedge mem_clk);
        if (frame_commit) begin
            lastw = ((fb_wr_index == 2'd1) ? BUF1 : BUF0) + FWORDS - 1;
            committed_gen[fb_wr_index] = sdm[lastw][25:24];
            n_commit = n_commit + 1;
            if (!ready) $display("[info] t=%0t 首次提交: buf=%0d gen=%0d rd_idx=%0d", $time, fb_wr_index, committed_gen[fb_wr_index], fb_rd_index);
            ready = 1;
        end
    end
end

// 读请求发出的那一刻, 锁定本帧应看到的模式与代标
integer exp_gen = 0, frame_mode = 0;
always @(posedge video_clk) begin
    if (!rst && (vs0_d & ~vs0_r)) begin
        exp_gen    = (committed_gen[fb_rd_index] < 0) ? 0 : committed_gen[fb_rd_index];
        frame_mode = mode;
        gen_seen[exp_gen & 3] = 1;
        buf_seen [fb_rd_index] = 1;
    end
end

// ---------------------------------------------------------------------------
// 输出像素检查
// ---------------------------------------------------------------------------
integer errors = 0, n_out = 0, n_checked = 0, pos_err = 0, skew_err = 0, gen_err = 0;
reg [7:0] bar_seen = 8'd0;
integer bar_bad = 0;
integer bar_n = 0;
integer px_in_frame, gx, gy, yy, cb, cr, gg;
reg [5:0] rx; reg [2:0] gy3;
reg [7:0] rr, ggr, bbr;

always @(posedge video_clk) begin
    if (!rst) begin
        if (vs_o && !vs_o_d2) n_out = 0;        // 每个输出帧开头清零计数
        if (de_o) begin
            n_out = n_out + 1;
            if (ready && !pause_chk && n_out <= FWORDS) begin
                n_checked = n_checked + 1;
                rx  = vout[23:18];               // R = {x[5:0], gen[1:0]}
                gy3 = vout[10:8];                // G = {5'b0, y[2:0]} -> y 在 G 的低 3 位
                if (frame_mode == 0) begin
                    // (1) 通道自洽: B 必须等于用 R、G 解出的 x、y 拼出的值
                    if ({rx[3:0], gy3, 1'b0} !== vout[7:0]) begin
                        skew_err = skew_err + 1; errors = errors + 1;
                        if (errors < 15)
                            $display("FAIL t=%0t 通道 skew: RGB=%b,%b,%b 解出 x=%0d y=%0d 但 B=%b",
                                     $time, vout[23:16], vout[15:8], vout[7:0], rx, gy3, vout[7:0]);
                    end
                    // (2) 位置: 第 n 个像素必须落在 (n%64, n/64)
                    if (rx !== ((n_out-1) % H_ACT) || gy3 !== (((n_out-1) / H_ACT) % 8)) begin
                        pos_err = pos_err + 1; errors = errors + 1;
                        if (errors < 15)
                            $display("FAIL t=%0t 位置错位: 第%0d 个像素 x=%0d y=%0d",
                                     $time, n_out-1, rx, gy3);
                    end
                end
                else if (frame_mode == 3) begin
                    // BAR: 与帧缓冲内容无关, 只要求落在 8 色彩条表内。
                    // 彩条计数器按真实显示宽度自由推进 (8×80=640), 本 TB 每行只有 64
                    // 列, 所以条带跨行缓动 —— 能覆盖满 8 档说明计数器、mux、延迟链都在
                    // 走, 而输出完全不由显存内容决定。
                    bar_n = bar_n + 1;
                    case (vout)
                        24'hFFFFFF: bar_seen[0] = 1'b1;
                        24'hFFFF00: bar_seen[1] = 1'b1;
                        24'h00FFFF: bar_seen[2] = 1'b1;
                        24'h00FF00: bar_seen[3] = 1'b1;
                        24'hFF00FF: bar_seen[4] = 1'b1;
                        24'hFF0000: bar_seen[5] = 1'b1;
                        24'h0000FF: bar_seen[6] = 1'b1;
                        24'h000000: bar_seen[7] = 1'b1;
                        default: begin
                            bar_bad = bar_bad + 1; errors = errors + 1;
                            if (errors < 15)
                                $display("FAIL t=%0t BAR 出现表外颜色 %h", $time, vout);
                        end
                    endcase
                end
                else begin
                    // (3) GRAY / YCC: 与 TB 独立算出的 BT.601 结果比
                    gx = (n_out-1) % H_ACT; gy = (n_out-1) / H_ACT;
                    rr  = {gx[5:0], exp_gen[1:0]};
                    ggr = {5'b0, gy[2:0]};
                    bbr = {gx[3:0], gy[2:0], 1'b0};
                    yy = (( 66*rr + 129*ggr +  25*bbr + 128) >> 8) + 16;
                    cb = (($signed((-38*rr -  74*ggr + 112*bbr + 128))) >> 8) + 128;
                    cr = (($signed(( 112*rr -  94*ggr -  18*bbr + 128))) >> 8) + 128;
                    gg =  (77*rr + 150*ggr +  29*bbr + 128) >> 8;
                    if (frame_mode == 1) begin
                        if (vout !== {gg[7:0], gg[7:0], gg[7:0]}) begin
                            gen_err = gen_err + 1; errors = errors + 1;
                            if (errors < 15) $display("FAIL t=%0t GRAY x=%0d y=%0d gen=%0d 期望 %03d 实际 %03d,%03d,%03d",
                                                      $time, gx, gy, exp_gen, gg, vout[23:16], vout[15:8], vout[7:0]);
                        end
                    end
                    else begin
                        if (vout !== {yy[7:0], cb[7:0], cr[7:0]}) begin
                            gen_err = gen_err + 1; errors = errors + 1;
                            if (errors < 15) $display("FAIL t=%0t YCC x=%0d y=%0d gen=%0d 期望 %03d,%03d,%03d 实际 %03d,%03d,%03d",
                                                      $time, gx, gy, exp_gen, yy, cb, cr,
                                                      vout[23:16], vout[15:8], vout[7:0]);
                        end
                    end
                end
            end
        end
    end
end

always @(posedge video_clk) vs_o_d2 <= vs_o;

// ---------------------------------------------------------------------------
// 流程控制
// ---------------------------------------------------------------------------
initial begin
    #12_000_000;
    $display("TIMEOUT: 12ms 仿真时间到, commit=%0d n_out=%0d n_checked=%0d ready=%0d wgen=%0d ack_seen=%0d",
             n_commit, n_out_seen, n_checked, ready, wgen, ack_seen);
    $display("位置错位=%0d 通道skew=%0d 颜色错=%0d 错误=%0d", pos_err, skew_err, gen_err, errors);
    $finish;
end
integer ack_seen = 0;
integer n_out_seen = 0;
always @(posedge cam_clk) if (cam_write_req_ack) ack_seen = ack_seen + 1;
always @(posedge video_clk) if (de_o) n_out_seen = n_out_seen + 1;

initial begin
    repeat (30) @(posedge video_clk);
    rst = 1'b0;
    mode = 2'd0;                    // RAW
    switch_and_run(NRAW, 2'd0);
    switch_and_run(NYCC,  2'd2);    // YCC
    switch_and_run(NGRAY, 2'd1);    // GRAY
    switch_and_run(NBAR,  2'd3);    // BAR (调试彩条, 不读显存)
    repeat (VT*HT*2) @(posedge video_clk);

    $display("---- tb_display_path 统计 ----");
    $display("检查输出像素 = %0d (约 %0d 帧), 写侧完成帧数指示 gen=%0d", n_checked, n_checked/FWORDS, wgen);
    $display("位置错位 = %0d, 通道 skew = %0d, 颜色/代标错 = %0d", pos_err, skew_err, gen_err);
    $display("写侧提交次数 = %0d; 颜色检查覆盖到的帧代标 = {%0d,%0d,%0d,%0d}; 读过的 bank = {%0d,%0d}",
             n_commit, gen_seen[0], gen_seen[1], gen_seen[2], gen_seen[3], buf_seen[0], buf_seen[1]);
    $display("BAR 模式: 检查像素 = %0d, 表外颜色 = %0d, 见到的彩条位图 = %b", bar_n, bar_bad, bar_seen);
    $display("读写突发撞拍 = %0d", collide);
    if (collide != 0) begin
        $display("FAIL: 仲裁让读写突发同时占了 SDRAM 端口, 帧内容会错位"); errors = errors + 1;
    end
    if (bar_bad != 0) begin
        $display("FAIL: BAR 模式输出了彩条表以外的颜色, 说明 mode=3 没走到独立图样"); errors = errors + 1;
    end
    if (bar_seen != 8'hFF) begin
        $display("FAIL: BAR 模式只走到 %b, 彩条计数器没跑满 8 档", bar_seen); errors = errors + 1;
    end
    if (!(gen_seen[0] && gen_seen[1] && gen_seen[2] && gen_seen[3])) begin
        $display("FAIL: 颜色检查没覆盖全部 4 个代标, 乒乓换帧未被真正验证"); errors = errors + 1;
    end
    if (!(buf_seen[0] && buf_seen[1])) begin
        $display("FAIL: 读侧从没切到另一块 bank, 乒乓未被验证"); errors = errors + 1;
    end
    if (n_checked < (NRAW+NYCC+NGRAY+NBAR)*FWORDS*3/4) begin
        $display("FAIL: 检查像素太少 (%0d), 读通路可能没跑起来", n_checked);
        errors = errors + 1;
    end
    if (errors == 0) $display("== tb_display_path PASS ==");
    else             $display("== tb_display_path FAIL (共 %0d) ==", errors);
    $finish;
end

// 切模式后丢掉当前这一帧: mux 是组合逻辑, 帧中途换模式会让那一帧混两种输出
task automatic switch_and_run(input integer nf, input [1:0] m);
    begin
        pause_chk = 1;
        mode = m;
        wait (ready);
        repeat (VT * HT + 4) @(posedge video_clk);
        pause_chk = 0;
        repeat (nf * VT * HT + 4) @(posedge video_clk);
    end
endtask

endmodule
