`timescale 1ns/1ps
// =============================================================================
// frame_buffer_ctrl 乒乓仿真
//
// 用行为级“帧存储”模型替代 SDRAM, 按实测速率比例复现读写并发:
//   写帧 = NPIX 像素 × WP 拍 = 200 拍    (对应 OV5640 76.4ms/帧)
//   读帧 = NPIX 像素 × 1 拍 (+ 4 拍消隐) = 44 拍 (对应 HDMI 16.8ms/帧)
//   200 / 44 = 4.55, 与实测 76.4 / 16.8 = 4.55 一致
//
// 判定的不变量:
//   A/B 逐拍比对 DUT 的 wr_index/rd_index 与 TB 参考模型 —— 覆盖“rd_index 只在
//      提交后变化、新值 = 刚写完的块、两指针恒互补”
//   C   读侧整帧每个像素的写入代标必须等于该块最近一次提交的代标 —— 无撕裂判据
//   D   统计读写落在同一块的重叠窗口, 确认这个危险场景真的被测到
// =============================================================================
module tb_frame_buffer_ctrl;

localparam NPIX    = 40;              // 一帧像素数 (缩小规模, 保留比例)
localparam WP      = 5;               // 写侧每像素拍数
localparam WFRAME  = NPIX * WP;       // 200
localparam RFRAME  = NPIX + 4;        // 44
localparam NFRAME  = 40;              // 模拟写帧数
localparam ABORT_F = 5;               // 该帧写一半被打断, 不提交

reg         mem_clk   = 1'b0;
reg         rst       = 1'b1;
reg         wr_finish = 1'b0;
wire [1:0]  wr_index, rd_index;

always #4 mem_clk = ~mem_clk;         // 125MHz

frame_buffer_ctrl dut (
    .mem_clk(mem_clk), .rst(rst), .wr_finish(wr_finish),
    .wr_index(wr_index), .rd_index(rd_index)
);

// ---------------------------------------------------------------------------
// 行为级帧存储: mem2[buf][pix] = 写入该像素时的帧代标
// ---------------------------------------------------------------------------
integer mem2      [0:1][0:NPIX-1];
integer committed [0:1];              // 每块最近一次提交完成的帧代标
reg [1:0] ref_wr, ref_rd;             // 乒乓参考模型 (定宽, 勿用 integer 做拼接比较)
integer tc = 0;
integer q;

integer wf, wp, rf, rp, rb;
integer errors = 0, n_read = 0, overlap = 0, aborted = 0, n_commit = 0;
integer first_commit = 0;

initial begin
    ref_wr = 2'd0; ref_rd = 2'd0;
    for (q = 0; q < 2; q = q + 1) begin
        committed[q] = -1;
        for (rp = 0; rp < NPIX; rp = rp + 1) mem2[q][rp] = -1;
    end
    repeat (5) @(negedge mem_clk);
    rst = 1'b0;
    tc  = 0;
end

always @(negedge mem_clk) begin
    if (!rst) begin
        wf = tc / WFRAME;
        wp = (tc % WFRAME) / WP;
        rf = tc / RFRAME;
        rp = tc % RFRAME;

        // ---- A/B: 先比对上一拍已稳定下来的指针, 再更新本拍 ----
        if ({wr_index, rd_index} !== {ref_wr, ref_rd}) begin
            errors = errors + 1;
            $display("FAIL t=%0t DUT wr=%0d rd=%0d 与参考模型 wr=%0d rd=%0d 不符",
                     $time, wr_index, rd_index, ref_wr, ref_rd);
        end
        if (first_commit && wr_index === rd_index) begin
            errors = errors + 1;
            $display("FAIL t=%0t 读写指针落在同一块", $time);
        end

        // ---- 读侧: 读帧起始锁存块号 (对应 frame_fifo_read 在 S_ACK 锁基址) ----
        if (rp == 0) rb = rd_index;
        if (rp < NPIX && first_commit) begin
            n_read = n_read + 1;
            // C: 读到的必须是该块“最近一次提交”的那一代, 否则就是读到了正在写的数据
            if (mem2[rb][rp] !== committed[rb]) begin
                errors = errors + 1;
                $display("FAIL t=%0t 读块%0d 第%0d 行 = %0d, 已提交代标 = %0d (读到正在写的数据)",
                         $time, rb, rp, mem2[rb][rp], committed[rb]);
            end
            if (rb == wr_index) overlap = overlap + 1;   // D
        end

        // ---- 写侧: 逐像素写入当前 wr_index 指定的块 ----
        if (wf < NFRAME)
            mem2[wr_index][wp] = wf;

        // ---- 帧尾提交 ----
        if (wf < NFRAME && tc % WFRAME == WFRAME - 1) begin
            if (wf == ABORT_F) begin
                // 模拟写入中途被新 write_req 打断: frame_fifo_write 直接回 S_ACK,
                // 不经过 S_END, 因此不发 write_finish, 写指针保持不动
                aborted = aborted + 1;
                wr_finish <= 1'b0;
                $display("[note] t=%0t 帧 %0d 未提交 (wr_index=%0d 应保持)", $time, wf, wr_index);
            end
            else begin
                wr_finish       <= 1'b1;
                committed[wr_index] = wf;
                ref_rd = ref_wr;
                ref_wr = (ref_wr == 2'd0) ? 2'd1 : 2'd0;
                n_commit = n_commit + 1;
                if (!first_commit) first_commit = 1;
            end
        end
        else
            wr_finish <= 1'b0;

        tc = tc + 1;
    end
end

// ---------------------------------------------------------------------------
// 收尾
// ---------------------------------------------------------------------------
always @(posedge mem_clk) begin
    if (tc >= NFRAME * WFRAME + RFRAME) begin
        $display("---- tb_frame_buffer_ctrl 统计 ----");
        $display("提交帧数 = %0d, 未提交(打断)帧数 = %0d, 读像素检查数 = %0d", n_commit, aborted, n_read);
        $display("读写同块重叠拍数 = %0d", overlap);
        if (aborted == 0) begin
            $display("FAIL: 未覆盖打断帧场景");       errors = errors + 1;
        end
        if (overlap == 0) begin
            $display("FAIL: 未覆盖读写同块重叠场景, 测试不充分"); errors = errors + 1;
        end
        if (n_commit != NFRAME - aborted) begin
            $display("FAIL: 提交次数与预期 %0d 不符", NFRAME - aborted); errors = errors + 1;
        end
        if (errors == 0) $display("== tb_frame_buffer_ctrl PASS ==");
        else             $display("== tb_frame_buffer_ctrl FAIL (%0d errors) ==", errors);
        $finish;
    end
end

endmodule
