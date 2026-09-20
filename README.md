# 26Anlu-vision

B 板视觉控制系统 —— 安路 **EG4S20BG256**（康芯 HX4S20C 板）+ **OV5640** 摄像头，
按 `docs/implementation-plan.md` 分 8 个 Phase 推进：采集 → 片内 SDRAM 帧缓冲 → YCbCr →
HDMI 显示，再逐级叠加火灾 / 烟雾 / 运动 / 手势识别与通信中转。

## 当前状态

| Phase | 内容 | 状态 |
|---|---|---|
| 1 | OV5640 → 片内 SDRAM → HDMI 基础通路 | ✅ 上板出图，TD 全流程时序收敛 |
| 2 | SDRAM 乒乓双缓冲 + BT.601 颜色空间转换 + 显示模式切换 | ✅ 代码与仿真完成；板上已确认灰度档与彩条档，0/2 档与新的读/写心跳 LED 待复看 |
| 3–8 | 火灾 / 烟雾 / 运动 / 手势 / 通信 / 集成 | 未开始 |

## 数据通路

```
ov5640_dri (SCCB 配置 + DVP 采集, RGB565)
  → ov5640_delay (RGB565→RGB888, 打包 {R,G,B,8'd0})
  → frame_read_write (写: cam_pclk 域 / 读: video_clk 域, 内部异步 FIFO 仲裁)
      ↑ 基址由 frame_buffer_ctrl 按帧提交在 BUF0/BUF1 间交替 (0 / 307200 字)
  → sdram (片内 2M×32, EG_PHY_SDRAM_2M_32 硬宏, 引脚不得写进 .adc)
  → display_path (rgb_to_ycbcr 两级流水 + 模式 mux + video_delay PIPE_LAT=2 对齐)
  → hdmi_tx (VHDL, TMDS LVDS)
```

关键数字：显示 640×480@59.5Hz（25MHz / 800×525），采集 1856×984@24MHz ≈ **13.1fps**，
一帧 307200 字 = `BURST_SIZE(256)` 的整数倍（这条必须成立，否则最后一轮突发会越界写进
另一块缓冲），两块共占片内 SDRAM 的 29%。**读比写快 4.6 倍**是双缓冲不撕裂的依据：
写指针绕回读侧正在显示的那一块时，读头始终领先。

## 板级判据（上电后一眼能读的东西）

| 丝印 | 球号 | 含义 |
|---|---|---|
| led1 | A4 | 常亮 = OV5640 SCCB 配置表写完 |
| led2 | A3 | 常亮 = 片内 SDRAM 初始化完成 |
| led3 | C10 | **闪 0.93Hz = 读通道活着**（每读完一帧翻转一次，59.5Hz/32） |
| led4 | B12 | **闪 0.82Hz = 写通道在提交整帧**（13.1Hz/8） |
| dled | A8 / A7 | 摄像头出帧 / 显示时序心跳，各约 0.8Hz |

指示器一律用**事件翻转计数**，不用"展宽脉冲"：亮灭各半才可判读，展宽脉冲的占空比一旦
接近 50% 肉眼看就是常亮（这一条本项目踩过坑）。

`key2`(B2) 切显示档位，`key1`(A2) 复位：

| 档位 | 显示 | 数据来源 |
|---|---|---|
| 0 | 原图 RGB | 帧缓冲 |
| 1 | 灰度（77R+150G+29B） | 帧缓冲 |
| 2 | 伪彩 R=Y, G=Cb, B=Cr | 帧缓冲 |
| 3 | **8 列彩条** | 内部计数器，完全绕开帧缓冲 |

第 3 档是调试档：它把"HDMI/显示器/同步这条链路是否活着"与"帧缓冲内容对不对"分开判断。
彩条按 `read_en` 计数（与显示像素同序），按 `de_i` 计会超前流水线深度而在行首行尾露白。

## 回归

```bash
sh sim/run_all.sh          # 需要 D:/iverilog/iverilog/bin 在 PATH 里
```

- 第 0 步按 `26Anlu-vision.al` 自己的文件清单跑 iverilog 并扫 "Unknown module"，等价于 TD 的
  black box 检查（加密 IP 与 TD 生成的 `ip/*.v` 用 `sim/lint_stubs.v` 顶替）
- `tb_rgb_to_ycbcr`：4264 像素逐位比对，值域实测 Y[16,235] Cb/Cr[16,240] gray[0,255]
- `tb_frame_buffer_ctrl`：按真实读写速率比建模，代标记分板 + 无撕裂判据
- `tb_display_path`：**真实** `frame_read_write` + 真实异步 FIFO 的端到端像素对齐，
  四档全过，并断言 `App_wr_en` 与 `App_rd_en` 永不同拍（仲裁互斥被破坏时，像素位置/颜色
  检查全都发现不了，只有这条能报）

`src/sdram/enc_file/*.enc.v`、`src/hdmi/enc_file/*.enc.vhd` 是加密 IP，iverilog 读不了，
所以只有纯 RTL 部分能本地仿真。

## 时序

125MHz（`mem_clk`）域现在 SWNS **+0.919ns**、0 违例端点、Fmax 141.2MHz，hold +0.105ns。
这条曾经是 `frame_fifo_write` 里 `write_len_latch <= (rdusedw + write_cnt)` 一个表达式造成的：
FIFO 指针相减的 5 级 ADDER 后面又串了 2 级加法 + 21 位幅值比较，直达读侧 FSM。改法见
`src/memory/frame_fifo_write.v` 注释。报告位置：`26Anlu-vision_Runs/syn_1/run.log`（登记与
black box）、`phy_1/final_timing.rpt`（逐端点单元/网络延迟表）。

## 已知残留

读侧基址在 `S_ACK` 那拍锁存，而 `rd_index` 要先过 2 级同步，所以提交若落在该 16ns 窗口内，
那次读会锁到旧索引 → 读到正在被写的那块，出现**一帧**撕裂，下一帧自愈。按 59.5 帧/s × 16ns ×
13.1 次提交/s 估算约 **22 小时一次**。单独为它上第 3 块缓冲不划算；Phase 3/5 加视觉/运动读通道
时本来要复制 `frame_fifo_read` + 读 FIFO，那时顺手把 3 缓冲一起上（`read_addr_2` 槽位现成）。

## 相关文档

- `docs/implementation-plan.md` —— 完整实施方案：系统架构、各检测算法的定点化设计、
  引脚与通信协议、8 个 Phase 的目标与验收判据、资源估算
