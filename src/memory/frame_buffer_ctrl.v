`timescale 1ns/1ps
// =============================================================================
// 帧缓冲乒乓管理器 (Phase 2)
//
// frame_read_write 本来就暴露了 write_addr_0..1 / read_addr_0..1 + *_addr_index
// 的多槽基址选择, 所以乒乓不需要动它的仲裁 FSM, 只要在这里产出两个索引:
//   wr_index —— 摄像头当前正在写入的缓冲
//   rd_index —— 最近一次写完整、可以被读出的缓冲
// 两者恒为 0/1 互补, 因此读侧永远读不到正在写的那一块。
//
// 提交点选 wr_finish (= frame_fifo_write 的 write_finish, 状态机 S_END 那一拍):
// 写入中途若被新的 write_req 打断, FSM 会从 S_CHECK_FIFO / S_WRITE_BURST_END
// 直接回 S_ACK 而不经过 S_END, 半截帧因此不会被提交, 读侧继续用上一整帧。
//
// 双缓冲在本系统里不会撕裂的依据: 读侧 640x480@59.5Hz (16.8ms/帧), 写侧 OV5640
// 1856x984@24MHz (76.4ms/帧, 约 13.1fps), 读比写快 4.6 倍, 故写指针绕回读侧正在
// 显示的那一块时, 读头始终领先写头, 行区间不重叠。
//
// 已知残余风险 (两拍采样窗, 分析所得, 未在本 TB 覆盖): frame_read_write 内部把
// *_addr_index 打了 2 拍同步后才在 S_ACK 锁基址, 所以
//   - 读侧: 若提交正好落在读请求 S_ACK 前 0~2 个 mem_clk 内, 读 FSM 会锁到旧索引,
//     即去读"写侧刚开始写"的那一块 -> 一帧画面滚动撕裂。窗口 2/2.1M 拍, 按 13.1
//     次提交/秒估算约 22 小时出现一次, 一帧后自愈。长时间拷机测试 (Phase 8) 有可能撞到。
//   - 写侧: 要求相邻两帧的 write_req 与上一帧提交至少隔 3 个 mem_clk, 当前摄像头
//     间隔 76ms, 裕量极大。
// 消除办法是上第 3 块缓冲 (frame_read_write 的 addr_2 槽位现成, 3 帧只占片内 SDRAM
// 44%): 那样即使锁到旧索引, 旧块也是一份完整且没在被写的帧, 只会晚一帧显示, 不会撕裂。
//
// 另外注意 frame_fifo_write/read 每轮突发固定 BURST_SIZE=256 字且不看剩余帧长, 所以
// 一帧的字数必须是 256 的整数倍, 否则最后一轮突发会越界写进下一块缓冲。307200 =
// 1200×256 ✓; 以后换分辨率必须保持这个整除关系。
// =============================================================================
module frame_buffer_ctrl (
    input  wire      mem_clk,
    input  wire      rst,        // 高有效, 与 mem_clk 同步

    input  wire      wr_finish,  // 一帧完整写入完成, 单拍脉冲

    output reg [1:0] wr_index,   // 接 frame_read_write.write_addr_index
    output reg [1:0] rd_index    // 接 frame_read_write.read_addr_index
);

always @(posedge mem_clk) begin
    if (rst) begin
        wr_index <= 2'd0;
        rd_index <= 2'd0;
    end
    else if (wr_finish) begin
        rd_index <= wr_index;                                  // 刚写完的一块交给读侧
        wr_index <= (wr_index == 2'd0) ? 2'd1 : 2'd0;          // 写侧让到另一块
    end
end

endmodule
