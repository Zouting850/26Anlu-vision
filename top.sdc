// =============================================================================
// B板视觉控制系统 - Phase 1 时序约束
// FPGA: Anlogic EG4S20BG256 (HX4S20C开发板)
// 
// 时钟频率：
//   - clk_50: 50MHz (外部晶振)
//   - ext_mem_clk: 125MHz (SDRAM)
//   - video_clk: 25MHz (HDMI 640x480@60Hz)
//   - video_clk_5x: 125MHz (HDMI 序列化器)
// ===========================================================================

# 1. 定义主时钟源 (50MHz, period = 20ns)
create_clock -name clk_50 -period 20.0 [get_ports clk_50]

# 2. SDRAM 相关约束 (ext_mem_clk = 125MHz via PLL)
set_clock_group -name sdram_pll -asynchronous \
    [get_clocks {clk_50}] \
    [get_clocks [get_pins u_video_pll/clk0_out]]

# 3. HDMI 显示路径约束 (video_clk = 25MHz via PLL)
set_clock_group -name hdmi_pll -asynchronous \
    [get_clocks {clk_50}] \
    [get_clocks [get_clocks [get_pins u_video_pll/clk2_out]]]

# 4. 跨时钟域约束 (camera_pclk is input from OV5640)
set_false_path -from [get_ports cam_pclk] -to [get_cells *frame_read_write*]

# 5. HDMI LVDS输出约束
set_max_delay -clock [get_clocks [get_pins u_hdmi_tx/PXLCLK_I]] \
    -to [get_ports {HDMI_CLK_P HDMI_D0_P HDMI_D1_P HDMI_D2_P}] 0.5

# 6. 摄像头输入路径约束
set_input_delay -clock [get_clocks [get_pins u_ov5640_dri/cam_pclk]] \
    -max 2.0 [get_ports {cam_data[7:0]}]

set_input_delay -clock [get_clocks [get_pins u_ov5640_dri/cam_pclk]] \
    -max 2.0 [get_ports {cam_vsync cam_href}]

# 7. 忽略不必要的路径
set_false_path -from [get_ports rst_n] -to [get_cells *]
