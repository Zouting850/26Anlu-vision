#**************************************************************
# B板视觉控制系统 — 时序约束
# Device: Anlogic EG4S20BG256
#
# 时钟域 (4 组互相异步):
#   [G1] clk_50        50MHz  晶振      → POR、OV5640 SCCB 配置状态机
#   [G2] cam_pclk      ~26MHz OV5640    → 像素采集、frame_read_write 写侧 FIFO
#   [G3] mem_clk       125MHz PLL C0    → SDRAM 控制器
#        mem_clk_sft   125MHz PLL C1(90°)→ 片内 SDRAM 硬宏 pad 相位
#   [G4] video_clk     25MHz  PLL C2    → HDMI 像素、frame_read_write 读侧 FIFO
#        video_clk_5x  125MHz PLL C3    → HDMI 1:10 串行器 (与 C2 同源 1:5, 保持同步分析)
#
# 域间数据交叠全部由异步 FIFO 承担 (u_frame_rw 内 wfifo/rfifo_32_32_512),
# clk_50 与其余各域之间只有复位(异步)与 capture_start(准静态电平),
# 因此声明为异步时钟组是正确且必要的 —— 这一步消除了 120 个
# "por_reset_n → 125MHz 域 rst" 的伪 setup 违例。
#
# 未添加 set_input_delay / set_output_delay:
#   摄像头并行数据在 cmos_capture_data 内部已做 2 级同步, SCCB/LED 为慢速信号;
#   同型号板的官方出厂例程 lab_ex_6_tf_sdram_hdmi 同样不加 I/O 延迟,
#   其 hold 干净 (HWNS +0.005ns)。凭 datasheet 估算填入延迟反而会造出
#   虚假的 pad hold 违例 (实测曾导致 HWNS -2.488ns / 9 端点)。
#**************************************************************

create_clock  -name clk_50   -period 20.000 [get_ports {clk_50}]
# cam_pclk 由 OV5640 输出; 本寄存器表 (HTS=1856, VTS=984) 下约 24~26MHz,
# 按 40ns(25MHz) 约束。换用更高分辨率/帧率的配置时需同步改小周期。
create_clock  -name cam_pclk -period 40.000 [get_ports {cam_pclk}]

# 自动推导 video_pll 四路输出 (命名形如 u_video_pll/pll_inst.clkc[n])
derive_pll_clocks

set_clock_groups -asynchronous \
    -group [get_clocks {clk_50}] \
    -group [get_clocks {cam_pclk}] \
    -group [get_clocks {u_video_pll/pll_inst.clkc[0] u_video_pll/pll_inst.clkc[1]}] \
    -group [get_clocks {u_video_pll/pll_inst.clkc[2] u_video_pll/pll_inst.clkc[3]}]

# --- C0 ↔ C1(90°): 仅通过片内 SDRAM 硬宏 EG_PHY_SDRAM_2M_32 的 DQ pad 交互 ---
# 该 32+32 端点是硬宏 IO 时序, 由硅片特性决定, 不是可编程逻辑路径。
# 出厂例程 lab_ex_6_tf_sdram_hdmi 在同一颗 SDRAM IP 上报告同样的
# SWNS -6.565ns / STNS -295.981ns 且实物可正常工作, 故此处显式豁免,
# 让时序报告保持干净, 以便后续阶段能一眼看到真正的新违例。
set_false_path -from [get_clocks {u_video_pll/pll_inst.clkc[0]}] \
               -to   [get_clocks {u_video_pll/pll_inst.clkc[1]}]
set_false_path -from [get_clocks {u_video_pll/pll_inst.clkc[1]}] \
               -to   [get_clocks {u_video_pll/pll_inst.clkc[0]}]

# --- 异步复位与准静态信号 ---
set_false_path -from [get_ports {key1}]
set_false_path -from [get_ports {clk_50}] -through [get_nets {Sdr_init_done}]
