# B板视觉控制系统实施方案

## Context

基于 HX4S20C 开发板（FPGA: EG4S20BG256）+ OV5640 摄像头，为 B 板设计视觉功能。B 板负责摄像头采集、多种视觉识别，通过 **UART** 向 A 板发送控制指令并中转 PC 应急指令，驱动 A 板的多媒体展示系统智能播放。B 板通过 **ESP32 WiFi 模块**（从A板移植）接收 PC 指令并回传状态。B 板本地通过 **HDMI** 输出摄像头画面及检测结果叠加，方便调试和演示。

**已确认**：
- 板间通信架构：
  - **ESP32 WiFi 模块**（从A板移植到B板）：PC ↔ ESP32 ↔ FPGA(B板)，双向通信
    - PC → WiFi → ESP32 → FPGA：PC发送应急指令，B板接收
    - FPGA → ESP32 → WiFi → PC：B板发送检测结果/状态到PC（调试/监控）
  - **UART**：FPGA(B板) → A板，发送控制指令
    - B板将PC应急指令**中转**给A板
    - B板将自身检测结果也发送给A板
  - 完整链路：`PC → WiFi → ESP32 → FPGA(B) → UART → A板`
  - ESP32与FPGA间接口待确认（UART或SPI），代码可直接从A板移植
- 本地显示：HDMI（摄像头画面 + 检测结果蒙版叠加）
- B板硬件：HX4S20C 开发板（与参考工程相同，引脚一致）
- 参考工程不是 A 板代码，仅复用摄像头框架
- 视觉功能：火灾检测 + 烟雾检测 + 运动检测/人体存在 + 手势方向识别

## 系统架构

```
                    ┌──────────┐
                    │   PC     │
                    └────┬─────┘
                     WiFi│
                    ┌────┴─────┐
                    │  ESP32   │ ← 从A板移植
                    └────┬─────┘
              UART/SPI │ (待确认接口)
                       │
[OV5640] --DVP/I2C--> [B板 FPGA]
                           |
              Camera → SDRAM帧缓冲（双缓冲乒乓）
                           |
              ┌────────────┼────────────────┐
              │            │                │
     ┌────────┴───┐  ┌────┴─────┐  ┌───────┴──────┐
     │ 火灾/烟雾   │  │ 运动检测  │  │ 手势方向      │
     │ 检测流水线   │  │ 人体存在  │  │ 识别         │
     │ (YCbCr阈值) │  │ (帧间差分)│  │ (区域运动分析)│
     └──────┬─────┘  └────┬─────┘  └──────┬───────┘
            │              │               │
            └──────────┬───┘───────────────┘
                       │
              检测结果融合 + PC指令中转
                       │
                  UART → A板
```

**B板FPGA通信职责**：
1. 接收ESP32转发的PC应急指令 → 解析 → 通过UART中转给A板
2. 自身视觉检测结果 → 通过UART发送给A板
3. 自身状态 → 通过ESP32回传给PC（调试/监控）

所有视觉功能共享 YCbCr 转换流水线，利用双缓冲 SDRAM 的当前帧/前一帧数据。

## 项目目录结构

```
D:\TD\26Anlu\26Anlu-vision\
├── top.v                              # 顶层模块
├── top.sdc                            # 时序约束
├── top.adc                            # 引脚约束（复用参考工程）
├── 26Anlu-vision.al                   # Anlogic TD工程文件
│
├── src\
│   ├── camera\                        # 摄像头驱动（复用）
│   │   ├── ov5640_dri.v
│   │   ├── i2c_dri.v
│   │   ├── i2c_ov5640_rgb565_cfg.v
│   │   ├── cmos_capture_data.v
│   │   └── ov5640_delay.v            # 修改：适配视觉流水线
│   │
│   ├── memory\                        # 存储（复用+修改）
│   │   ├── sdram.v                    # SDRAM控制器（复用）
│   │   ├── sdr_as_ram.enc.v
│   │   ├── frame_read_write.v         # Phase 2 乒乓、Phase 3 火焰都没改它；Phase 5 要第二条读通道(帧间差分)时才改
│   │   ├── frame_buffer_ctrl.v        # ✅ Phase 2 新增：乒乓索引
│   │   ├── frame_fifo_write.v
│   │   └── frame_fifo_read.v
│   │
│   ├── comm\                          # 板间通信（新增）
│   │   ├── uart_tx.v                  # UART发送模块（B板→A板）
│   │   ├── uart_rx.v                  # UART接收模块（ESP32→B板，接收PC指令）
│   │   ├── esp32_if.v                # ESP32接口模块（从A板移植/适配）
│   │   ├── cmd_sender.v             # B板→A板：检测结果打包+发送
│   │   └── cmd_relay.v              # PC指令中转：接收→解析→转发给A板
│   │
│   ├── vision\                        # 视觉处理（全部新增）
│   │   ├── rgb_to_ycbcr.v             # 颜色空间转换（共享基础）
│   │   ├── line_buffer.v              # 复用：3x3滑动窗口
│   │   ├── sobel_process.v            # 复用：边缘检测
│   │   │
│   │   ├── fire_detector.v            # ✅ Phase 3：火焰逐像素色域判据（纯组合）
│   │   ├── fire_region_analyzer.v     # ✅ Phase 3：块密度+帧统计+质心+报警（挂在 display_path 里）
│   │   │
│   │   ├── smoke_detector.v           # 烟雾检测核心
│   │   ├── smoke_region_analyzer.v    # 烟雾区域统计
│   │   │
│   │   ├── motion_detector.v          # 运动检测（帧间差分）
│   │   ├── presence_analyzer.v        # 人体存在判定
│   │   │
│   │   └── gesture_recognizer.v       # 手势方向识别
│   │
│   ├── video\                         # 本地显示（复用）
│   │   ├── video_timing_data.v
│   │   ├── video_delay.v              # ✅ Phase 2 加 PIPE_LAT 参数
│   │   ├── display_path.v             # ✅ Phase 2 新增：转换+mux+对齐（Phase 3 蒙版叠加挂这里）
│   │   └── hdmi_tx.enc.v
│   │
│   └── util\                          # 工具模块（复用）
│       ├── debounce.v
│       └── led.v
│
├── ip\                                # Anlogic IP核（复用）
│   ├── sys_pll.v
│   ├── video_pll.v
│   └── line_ram_640x8.v
│
└── sim\                               # 仿真测试
    ├── run_all.sh                     # ✅ Phase 2: sh sim/run_all.sh 跑全部纯 RTL 回归
    ├── tb_rgb_to_ycbcr.v              # ✅ Phase 2
    ├── tb_frame_buffer_ctrl.v         # ✅ Phase 2
    ├── tb_display_path.v              # ✅ Phase 2（真实仲裁+FIFO 的端到端像素对齐）
    ├── tb_fire_detector.v             # ✅ Phase 3：色域真值表 + 块密度/报警/质心/蒙版回读
    ├── tb_fire_overlay.v              # ✅ Phase 3：真实读通道下的蒙版落位端到端
    └── tb_motion_detector.v
```

## 关键模块设计

### 1. RGB→YCbCr 颜色空间转换 (`rgb_to_ycbcr.v`)

所有视觉功能共享此转换模块。从参考工程 `udp_cam_ctrl.v:319-322` 提取系数，2级流水线：

```
S1（组合逻辑乘法+寄存输出）:
  Y_pre  = 66*R + 129*G + 25*B + 128
  Cb_pre = -38*R - 74*G + 112*B + 128
  Cr_pre = 112*R - 94*G - 18*B + 128
  gray   = 77*R + 150*G + 29*B + 128

S2（算术右移+偏移）:
  Y  = (Y_pre  >>> 8) + 16
  Cb = (Cb_pre >>> 8) + 128
  Cr = (Cr_pre >>> 8) + 128
  gray 不移位偏移, 输出 0~255 全范围
```

> **Phase 2 修正**: 原稿写的 `+28672` 是错的。`28672 = 112<<8`, 右移后等于给 Cb/Cr
> 加了 112 而不是 128 的色度偏置 —— 纯色红会得 Cb=74、Cr=223, 正确值是 Cb=90、Cr=240。
> 正确形式是把 `+128` 当四舍五入偏置放进被移位的那个和里, `+128`(色度)/`+16`(亮度)
> 的偏置在移位之后加 —— 与参考工程 `udp_cam_ctrl.v:319-322` 一致。另外 `>>>` 必须是
> 算术右移: 若中间量按无符号处理, `-38*R` 会回卷且 `>>` 变逻辑移位 (Verilog 表达式
> 定标规则: 只要有一个操作数无符号, 整个式子就变无符号), 需用 `reg signed` 或先搬进
> `integer`。
>
> **不需要钳位**: 输入为 8bit 时值域天然是 Y[16,235]、Cb/Cr[16,240]、gray[0,255],
> 原稿的钳位边界正好就是这些极值, 比较器永不动作。仿真实测角点: 红(82,90,240)、
> 绿(144,54,34)、蓝(41,240,110)、黄(210,16,146), 与 BT.601 公布值一致。

输出 Y、Cb、Cr、gray 四路信号，供下游所有检测模块并行使用。

### 2. 火焰检测 (`fire_detector.v` + `fire_region_analyzer.v`)

**逐像素检测**（`fire_detector.v`）：

| 条件 | 判断 | 说明 |
|------|------|------|
| 亮度 | Y >= 180 | 火焰高亮度 |
| 红色色度 | Cr >= 155 | 火焰高红色色度 |
| 蓝色色度 | Cb <= 120 | 火焰低蓝色色度 |
| R > G | 红通道 > 绿通道 | 火焰偏红 |
| R > B | 红通道 > 蓝通道 | 火焰暖色调 |

可选空间验证（使用 `line_buffer.v` 3x3窗口）：邻域 >= 3 像素同为候选 → 确认。

**区域统计**（`fire_region_analyzer.v`）：
- 帧级累加：火焰像素计数、质心坐标（sum_x, sum_y）
- 帧结束输出：`fire_ratio`（占比）、`fire_cx/cy`（质心）、`fire_alarm`（报警）

### 3. 烟雾检测 (`smoke_detector.v` + `smoke_region_analyzer.v`)

**检测原理**：烟雾在 YCbCr 空间的特征为低饱和度（Cb、Cr 接近128）+ 中等亮度 + 区域随时间扩散。

**逐像素检测**（`smoke_detector.v`）：

| 条件 | 判断 | 说明 |
|------|------|------|
| 低饱和度 | abs(Cb - 128) < 20 且 abs(Cr - 128) < 20 | 烟雾颜色接近中性灰 |
| 中等亮度 | 80 < Y < 220 | 烟雾既不太暗也不太亮 |
| 灰度特征 | abs(Cb - Cr) < 15 | Cb和Cr接近，低彩色 |
| 亮度变化 | 与前一帧同位置像素亮度差 < 30 | 烟雾是缓慢变化的（需帧缓冲配合） |

**区域统计**（`smoke_region_analyzer.v`）：
- 低饱和度区域计数 `smoke_count`
- 与前一帧的烟雾区域对比：新增面积 `smoke_growth`
- 帧结束输出：`smoke_ratio`（占比）、`smoke_growth_rate`（扩散速率）、`smoke_alarm`

**判定逻辑**：
- 大面积低饱和度区域 + 持续扩散 → 烟雾报警
- 单纯低饱和度但不扩散（如灰色墙壁）→ 不报警

### 4. 运动检测 / 人体存在 (`motion_detector.v` + `presence_analyzer.v`)

**检测原理**：利用 SDRAM 双缓冲，当前帧灰度 vs 前一帧灰度，逐像素差分。

**帧间差分**（`motion_detector.v`）：

```
输入：当前帧 gray_curr[7:0]，前一帧 gray_prev[7:0]（从SDRAM另一bank读取）
处理：diff = abs(gray_curr - gray_prev)
      motion_pixel = (diff > motion_thresh) ? 1 : 0    // motion_thresh 默认 20
输出：motion_mask（二值运动掩码）
```

SDRAM 读取策略：视觉流水线读取当前帧用于火灾/烟雾检测的同时，运动检测模块读取前一帧对应位置的灰度值。可通过共享 `line_buffer` 延迟一行来实现帧对齐，或在 SDRAM 中额外开辟一行缓冲。

> **Phase 3 落地后的更正（2026-09-21）**：火灾/烟雾这类**色域**检测不再单独开读通道，而是分用
> `display_path` 正在显示的那一路（理由与代价见 Phase 3 一节）。因此"同时读两帧"这件事只剩
> 运动检测需要 —— 到 Phase 5 才复制 `frame_fifo_read` + 读 FIFO，并顺手把第 3 块缓冲换上
> （`read_addr_2` 槽位现成，可同时消除读侧 16ns 锁存窗口的撕裂残留）。

**人体存在判定**（`presence_analyzer.v`）：
- 帧级累加运动像素数 `motion_count`
- 计算运动比例 `motion_ratio = motion_count / (640*480)`
- 滑动窗口平滑（最近 N 帧平均）
- 输出：
  - `presence_detected`：有人存在（motion_ratio > 阈值，如 1%）
  - `motion_level[7:0]`：运动强度等级（0=静止，255=剧烈运动）
  - `presence_duration[15:0]`：持续存在时间（用于判断"停留"vs"路过"）

### 5. 手势方向识别 (`gesture_recognizer.v`)

**检测原理**：将画面分为左右两个区域，比较各区域的运动能量，判断运动方向。

```
┌──────────┬──────────┐
│          │          │
│  左区域   │  右区域   │
│  x:0-319 │  x:320-639│
│          │          │
└──────────┴──────────┘
```

**实现**（`gesture_recognizer.v`）：

```
每帧处理：
  left_motion  = 左区域运动像素累加
  right_motion = 右区域运动像素累加

  方向判定：
    if left_motion > right_motion * 2  →  "向左运动" (swipe_left)
    if right_motion > left_motion * 2  →  "向右运动" (swipe_right)
    if both > threshold                →  "靠近/远离" (需上下区域配合)

  去抖：连续 3 帧同方向才确认手势
  冷却：手势确认后 500ms 内不重复触发
```

输出：
- `gesture_type[2:0]`：0=无，1=左划，2=右划，3=上划，4=下划
- `gesture_valid`：手势有效标志（脉冲）
- `gesture_confidence[7:0]`：置信度

**对应A板控制**：
- 左划 → A板播放下一个内容
- 右划 → A板播放上一个内容
- 上划 → A板放大/切换详情
- 下划 → A板缩小/返回列表

### 6. 通信模块（双通道：ESP32↔PC + UART→A板）

B板有两条通信通道：
- **ESP32通道**（从A板移植）：接收PC应急指令 + 向PC回传状态
- **UART通道**（B板→A板）：发送检测结果 + 中转PC指令

```
┌─────────────────────────────────────────────────────────┐
│                    B板 FPGA                              │
│                                                         │
│  ┌──────────────┐     ┌──────────────┐                  │
│  │ esp32_if.v   │     │ cmd_sender.v │                  │
│  │ (ESP32接口)  │     │ (检测结果打包)│                  │
│  │ 从A板移植     │     │              │                  │
│  └──────┬───────┘     └──────┬───────┘                  │
│         │ 接收PC指令          │ 检测结果字节流             │
│         ▼                    ▼                          │
│  ┌──────────────┐     ┌──────────────┐                  │
│  │ cmd_relay.v  │     │  uart_tx.v   │                  │
│  │ (指令解析+    │────▶│  (UART发送)   │──── UART ──▶ A板 │
│  │  中转)        │     │              │                  │
│  └──────────────┘     └──────────────┘                  │
│         ▲                                               │
│  ┌──────┴───────┐                                      │
│  │  uart_rx.v   │                                      │
│  │ (UART接收    │◀── UART/SPI ── ESP32                 │
│  │  ESP32数据)  │                                      │
│  └──────────────┘                                      │
└─────────────────────────────────────────────────────────┘
```

#### 6.1 `esp32_if.v` — ESP32接口（从A板移植/适配）

此模块从A板已有的ESP32通信代码移植，负责FPGA与ESP32之间的数据交换。
- 具体接口方式（UART/SPI）待ESP32模块资料确认后确定
- 移植时需适配B板的引脚分配和时钟域

#### 6.2 `uart_rx.v` — UART接收（ESP32→FPGA）

接收ESP32转发的PC指令：
- 波特率：与ESP32配置匹配（待确认）
- 帧格式：1起始位 + 8数据位 + 1停止位
- 输出：接收到的字节流给 `cmd_relay.v`

#### 6.3 `cmd_relay.v` — PC指令解析与中转

```
功能：
1. 接收ESP32通道传来的PC指令字节流
2. 解析协议帧（帧头检测 + 长度 + 数据 + 校验）
3. 将有效指令重新打包后通过UART发送给A板
4. 可选：B板自身也对部分指令做响应（如切换检测模式）

PC指令类型（待A板协议文档确认后完善）：
- 应急控制指令：直接透传给A板
- B板配置指令：修改检测阈值、切换模式等（B板自身处理）
```

#### 6.4 `cmd_sender.v` — B板检测结果发送

**协议帧格式**（B板→A板）：

```
| 帧头(2B) | 类型(1B) | 长度(1B) | 数据(NB) | CRC8(1B) |
|  0xAA55  |   cmd   |   len   | payload  |  crc8   |
```

**指令类型定义**：

| cmd | 名称 | payload | 说明 |
|-----|------|---------|------|
| 0x01 | 心跳 | status[7:0] | 每100ms，含系统状态 |
| 0x02 | 火灾报警 | severity, ratio | 状态变化时+活跃期每200ms |
| 0x03 | 火焰坐标 | cx_h, cx_l, cy_h, cy_l | 每帧发送（有火时） |
| 0x04 | 烟雾报警 | severity, ratio, growth_rate | 状态变化时+活跃期每500ms |
| 0x05 | 运动状态 | presence, motion_level | 每帧发送 |
| 0x06 | 手势识别 | gesture_type, confidence | 手势发生时立即发送 |
| 0x07 | 播放控制 | mode, param | 综合判断后的播放指令 |
| 0x08 | 场景类型 | scene_id, confidence | 综合场景分类 |
| 0xF0 | PC指令中转 | original_cmd, original_payload | 透传PC发给A板的指令 |

**发送调度**：
- 优先级：火灾报警 > 烟雾报警 > 手势 > PC指令中转 > 播放控制 > 运动状态 > 心跳
- 心跳：固定100ms
- 报警类：状态变化立即发送 + 活跃期重复
- 手势：触发即发
- PC指令中转：收到即转发
- 状态类：每帧处理完成后发送

#### 6.5 `uart_tx.v` — UART发送（FPGA→A板）

标准UART发送模块：
- 波特率：115200（参数化，可配置至921600）
- 帧格式：1起始位 + 8数据位 + 1停止位，无校验
- 时钟：系统50MHz，分频产生波特率

### 7. SDRAM多缓冲策略

利用 `frame_read_write.v` 已有的多bank地址选择机制：

```
Bank A (0x00000-0x4BFFF): 帧缓冲A (640x480 x 32bit)
Bank B (0x4C000-0x97FFF): 帧缓冲B

帧N:   摄像头写Bank A
       视觉流水线读Bank A（当前帧处理）
       运动检测读Bank B（前一帧，用于帧间差分）
帧N+1: 摄像头写Bank B
       视觉流水线读Bank B
       运动检测读Bank A
vsync时交换bank角色
```

带宽分析（100MHz, 32bit = 400MB/s）：
- 摄像头写入: 36.8 MB/s (9.2%)
- 视觉读取（当前帧）: 36.8 MB/s (9.2%)
- 运动检测读取（前一帧）: 36.8 MB/s (9.2%)
- 总计 ~28%，远低于SDRAM带宽上限

## 需要修改的现有模块

| 模块 | 修改内容 | 源文件 |
|------|---------|--------|
| `ov5640_delay.v` | 适配视觉流水线写请求 | `import/ov5640_delay.v` |
| `frame_read_write.v` | **双缓冲不需要改它**（见 Phase 2）。只有 Phase 3/5 要加视觉读通道/运动检测读通道时才动：需为每条读通道复制一份 `frame_fifo_read` + 读 FIFO 并加仲裁 | `src/memory/frame_read_write.v` |
| `video_delay.v` | ✅ Phase 2 已加 `PIPE_LAT` 参数，移位寄存器按 `20+PIPE_LAT` 加宽；默认 0 与改动前逐拍等价 | `src/video/video_delay.v` |

## 复用的现有模块（不修改）

| 模块 | 源路径 |
|------|--------|
| `ov5640_dri.v` | `import/ov5640_dri.v` |
| `i2c_dri.v` | `import/i2c_dri.v` |
| `i2c_ov5640_rgb565_cfg.v` | `import/i2c_ov5640_rgb565_cfg.v` |
| `cmos_capture_data.v` | `import/cmos_capture_data.v` |
| `sdram.v` + `sdr_as_ram.enc.v` | `lab_ex_6/.../sdram/` |
| `frame_fifo_write.v` | `src/frame_fifo_write.v` |
| `frame_fifo_read.v` | `src/frame_fifo_read.v` |
| `line_buffer.v` | `import/line_buffer.v` |
| `sobel_process.v` | `import/sobel_process.v` |
| `video_timing_data.v` | `src/video_timing_data.v` |
| `video_delay.v` | `src/video_delay.v` |
| `hdmi_tx.enc.v` | `hdmi/` |
| `sys_pll.v` / `video_pll.v` | `lab_ex_6/.../al_ip/` |
| `debounce.v` | 参考工程 |
| `led.v` | `import/led.v` |

## 实施阶段

### Phase 1: 工程骨架 + 摄像头点亮
- 创建Anlogic TD工程，设备EG4S20BG256
- 复制所有可复用IP/加密核到项目目录
- 创建 `top.v`：摄像头 → SDRAM → HDMI 基础通路
- 引脚约束从参考工程复制
- **验证**: HDMI上看到摄像头实时画面

### Phase 2: SDRAM双缓冲 + 颜色空间转换 ✅（已完成，实现方式有偏差见下）
- **不改 `frame_read_write.v`**：它本来就暴露了 `write_addr_0..3` / `read_addr_0..3` +
  `*_addr_index` 的多槽基址选择，乒乓只要新增 `src/memory/frame_buffer_ctrl.v`
  产出两个索引即可（提交点用已有的 `write_finish`）
- `frame_read_write.v` 内部 FSM 有一个对乒乓很关键的性质：写入中途被新 `write_req`
  打断时状态机从 `S_CHECK_FIFO`/`S_WRITE_BURST_END` 直接回 `S_ACK`，不经过 `S_END`，
  所以半截帧不会发 `write_finish`、不会被提交
- `video_delay.v` 加 `PIPE_LAT` 参数（默认 0 保持原行为），补偿像素流水线延迟
- 实现 `rgb_to_ycbcr.v`，仿真验证
- HDMI 显示模式切换（key2/B2）：原图 → 灰度(Y) → 伪彩(R=Y,G=Cb,B=Cr) → 彩条(调试档)
- **验证**:
  - `sim/tb_rgb_to_ycbcr.v`：4264 个像素逐位比对 0 错误，角点与 BT.601 公布值一致
  - `sim/tb_frame_buffer_ctrl.v`：7132 个读像素的"帧代标"全部等于该块最近一次提交的
    代标（无撕裂判据），其中 632 拍确实处于读写同块的重叠窗口；含 1 帧被打断不提交的
    场景。**该 TB 抓出了三目翻转写反的真 bug**
  - 板级（两条独立心跳，一边盯乒乓的一半）：
    - `led3` = **读通道**心跳，接 `frame_read_write.read_finish`，每读完一帧翻转，
      59.5Hz/32 → 0.93Hz 闪。读侧一旦被 SDRAM 仲裁卡住它就灭
    - `led4` = **写通道**心跳，接 `write_finish`，13.1Hz/8 → 0.82Hz 闪
    - 第一版曾把 commit 脉冲展宽 40ms 点 led3、把写缓冲号直接点 led4：前者占空比
      53% 肉眼看是**常亮**（把"乒乓在翻页"说反了），后者 13.1Hz 看是**狂闪**。
      指示器一律改成"事件翻转计数"，亮灭各半才不会骗人
  - `display_path` 另有 `MODE_BAR`（key2 第 4 档）：8×80=640 列彩条，完全绕开帧缓冲，
    用来把"HDMI/显示器/同步是活的"与"帧缓冲内容对不对"分开判断。彩条按 `read_en`
    计数（与显示像素同序）；`tb_display_path` 已覆盖该档：5120 像素全部落在彩条表内、
    8 档全覆盖，其余三档仍 0 错
- **收尾：清掉 125MHz 域那条唯一的 setup 违例**。`frame_fifo_write.v:70` 原式
  `write_len_latch <= (rdusedw + write_cnt)` 让 FIFO 指针相减的 5 级 ADDER 后面又串了
  2 级加法 + 21 位比大小，直达 `O_wr_busy` → 读侧 FSM（7.8ns / 周期 8ns）。已把常量差值
  预计算成饱和寄存器 `burst_need`，比较降到 10 位。**TD 实测：SWNS -0.106 → +0.919ns，
  违例端点 1 → 0，该域 Fmax 123.4 → 141.2MHz**，hold +0.105ns 不变；新的最差路径回到
  FIFO 自身的指针→RAM 地址里，仲裁交叉锁不再是瓶颈。等价性与"晚一拍不削弱 busy 预测力"
  的论证写在源码注释里。`tb_display_path` 同步加了一条不变量：**`App_wr_en` 与 `App_rd_en`
  永不同拍**——实测删掉 `frame_fifo_read.v:224` 的 `~App_wr_busy`（造成 218 次撞拍）时，
  所有像素位置/颜色/提交数检查**仍然全过**，只有这条断言报 FAIL，所以它是仲裁改动的唯一验收

### Phase 3: 火灾检测 ✅ 代码与仿真完成（待上板）

**与原方案的偏差（决定，不是遗漏）**：原稿写的是"为视觉另开一条 SDRAM 读通道"（复制
`frame_fifo_read` + 读 FIFO 并加仲裁）。实现改为**分用显示读通道**——检测像素直接从
`display_path` 正在显示的那一路取。理由三条：

1. 火灾/烟雾都是**色域**判据，需要的就是"此刻显示的那一帧"，另开读通道读到的是同一块
   缓冲的同一份内容，白付一条通道（≈2 块 FIFO + 仲裁器 + 一份 SDRAM 带宽）。
2. 分用之后，蒙版写地址、蒙版读地址、检测坐标是**同一套光栅坐标**，不需要任何坐标变换；
   独立通道则要处理两条通道的行/帧错位（`tb_display_path` 已经证明这类 skew 是最难查的）。
3. 真正需要第二条读通道的是**帧间差分**（Phase 5 要同时读当前帧与前一帧），那时再复制，
   并且顺手上第 3 块缓冲（`read_addr_2` 槽位现成，见"已知残留"）。

代价：检测的时间单位变成"显示遍"（59.5Hz）而不是相机帧（13.1fps），同一相机帧被重复判
约 4.55 遍。两遍之间内容完全相同，重复判定结果也相同，所以只是让 `SET_N`/`CLR_N` 这类
"连续 N 帧确认"的时间常数要按遍来算（`SET_N=3 遍 ≈ 50ms`）。

**流水线对齐的关键一拍**：`video_delay` 把捕获级节拍引出为 `de_cap`/`vs_cap`（`de_d[CAP_IDX]`，
`CAP_IDX = 18 + 1 + PIPE_LAT`）。抽头留在 `video_delay` 里，因为只有它知道 `RD_IDX`/`CAP_IDX`，
复制到外面会随 `PIPE_LAT` 漂移。两个不显然的性质：

- `de_cap` 那一拍 `read_data` 里已经是**后面两个像素**了（每个 `read_en` 只保留一拍），
  所以喂给 `fire_detector` 的 R/G/B 必须另走一条 `PIPE_LAT` 移位链（`det_rgb_dly`）。
- 光栅坐标由 `de_cap`/`vs_cap` 自己数出来，与转换结果天然同列。`cx` 必须在 `de_cap` 当拍
  就 +1（写成"延一拍再 +1"会让整块蒙版右移一列，块边界那列的候选像素算进隔壁块）。

**三层过滤**（`fire_detector.v` 纯组合 + `fire_region_analyzer.v` 全部状态）：

| 层 | 判据 | 挡掉什么 |
|---|---|---|
| 逐像素 | `Y≥180 && Cr≥155 && Cb≤120 && R>G && R>B && Cr-Cb≥60` | 非暖色、暗色、偏紫 |
| 块密度 | 8×8 块内候选像素 ≥ 20/64（≈31%） | 孤立亮点、反光点、传感器噪点 |
| 面积+时间 | 标红块 ≥ 6 块（≈全帧 0.125%）连续 3 遍置位；低于 3 块连续 8 遍撤销（漏桶） | 一小团火苗级色块、单遍抖动 |

`R>G`/`R>B` 在数学上已被 `Cr≥155` 和 `Cr-Cb≥60` 蕴含（反证见 `fire_detector.v` 注释），保留
只因为门限是参数、调低后蕴含断裂，代价 2 个 LUT。

**呈现**：8×8 块级蒙版，两块 4800×1bit RAM 乒乓（各 13 位地址 = 8192 项 = 1 块 ERAM9K x1），
显示读"上一遍写完"的那块、本遍写另一块 → 读写永不落在同一块 RAM（ERAM 的读穿写在交叉
地址上是不定值），代价是蒙版比画面晚一遍（≤16.8ms）。叠加取 50% 半透明红，保留底图轮廓。
质心用帧末消隐期跑的 19 拍移位相减除法器（按块加权），省掉一整条 19 级比较器链。

**板级判读（右上角 4 格 HUD，2 块高 = 16 行，每格 8 像素宽，黑底 = 灭）**：

| 格 | 颜色 | 亮 = | 灭 = |
|---|---|---|---|
| 1 | 蓝 | 检测遍心跳（0.93Hz 明暗各半）→ 分析器在收遍 | 收遍停了（`de_cap`/`pass_end` 没来） |
| 2 | 黄 | 本遍有候选像素（色域判据命中过） | 色域门限太高 / 场景里没有暖色 |
| 3 | 橙 | 本遍有块过了密度门限（蒙版里确有红块） | 只有散点，没成块 |
| 4 | 红闪 | 报警中（1.98Hz） | 未达 6 块或未到 3 遍 |

HUD 在四档模式下都画（含彩条档），所以切到调试档时检测器状态仍然可读。

- **验证**：对真实火焰（蜡烛/打火机）有响应；对白墙、肤色、暗橙、灯光反光不误报
  （暗橙与肤色这两条反例已经做进仿真：`Y` 不到 180 与 `Cr-Cb=57<60`）
- **仿真证据**（`sh sim/run_all.sh` 第 4、5 步）：
  - `tb_fire_detector`：色域真值表 + 块密度/报警/质心 + 蒙版逐地址回读，全过
  - `tb_fire_overlay`：真实读通道 + 真实异步 FIFO 下 46080 个像素逐像素判读（约 10 遍），
    底图错 / 蒙版落位错 / HUD 错 / 读写撞拍 / 读通道空取 / 显存对账不合格 **全为 0**，
    四类区域与 HUD 四格、彩条 8 色都被覆盖到

### Phase 4: 烟雾检测
- 实现 `smoke_detector.v` + `smoke_region_analyzer.v`
- 仿真：注入合成烟雾图像（灰色扩散区域）
- HDMI叠加显示烟雾检测蒙版（灰色半透明）
- **验证**: 对真实烟雾有响应，灰色墙壁不误报

### Phase 5: 运动检测 + 人体存在
- 实现 `motion_detector.v` + `presence_analyzer.v`
- 仿真：注入前后帧差异图像
- HDMI叠加显示运动掩码（绿色轮廓）
- **验证**: 人走过画面时检测到运动，静止时无人存在标志

### Phase 6: 手势方向识别
- 实现 `gesture_recognizer.v`
- 依赖 Phase 5 的运动检测输出
- 仿真：注入左右方向运动序列
- HDMI叠加显示手势识别结果（方向箭头）
- **验证**: 左右挥手能正确识别方向

### Phase 7: 通信模块
- 实现 `uart_tx.v`（B板→A板UART发送）
- 实现 `uart_rx.v`（ESP32→B板UART接收）
- 实现 `cmd_sender.v`（检测结果打包+优先级调度）
- 实现 `cmd_relay.v`（PC指令解析+中转给A板）
- 待ESP32模块资料到位后：移植 `esp32_if.v` 并适配
- 先用PC串口助手模拟ESP32，验证指令接收和中转逻辑
- **验证**: 串口助手正确接收B板检测结果；模拟PC指令能正确中转给A板

### Phase 8: 系统集成 + 调优
- 全链路：摄像头 → 4种视觉检测 → 通信模块 → A板（待A板就绪）
- HDMI多色叠加显示：红色=火焰，灰色=烟雾，绿色=运动，蓝色=手势
- 阈值调优（真实场景）
- 压力测试：连续运行数小时
- 时序收敛验证
- 按键控制：检测模式切换、阈值调节
- LED指示：各检测状态

## 资源估算

| 资源 | 估算用量 | FPGA总量 | 占比 |
|------|---------|---------|------|
| LUT4 | ~4,500 | ~20,000 | ~22.5% |
| FF | ~3,500 | ~20,000 | ~17.5% |
| BRAM | 4 (line_buffer x2 + 帧灰度缓存) | -- | 中等 |

各检测模块共享 YCbCr 转换，增量资源主要来自比较器和累加器，整体资源占用可控。

## HDMI叠加显示方案

在HDMI输出上叠加各检测结果，方便调试和演示：

| 检测结果 | 叠加颜色 | 叠加方式 |
|---------|---------|---------|
| 火焰像素 | 红色 (R=255, G=0, B=0) | 半透明覆盖 |
| 烟雾区域 | 灰色 (R=128, G=128, B=128) | 半透明覆盖 |
| 运动区域 | 绿色轮廓 (R=0, G=255, B=0) | 边缘描线 |
| 手势方向 | 蓝色箭头 | 画面中央叠加 |
| 系统状态 | 左上角文字区 | 帧率、各检测状态、UART发送计数 |

## 验证方法

1. **仿真验证**: 对 `rgb_to_ycbcr`、`fire_detector`、`motion_detector` 做单元仿真
2. **HDMI视觉验证**: 多色叠加显示，用真实火焰/烟雾/手势场景测试
3. **通信验证**: PC串口助手模拟ESP32/A板，验证B板检测结果发送和PC指令中转
4. **端到端验证**: A板根据B板指令切换多媒体内容（待A板就绪）
5. **压力测试**: 连续运行24小时以上

## 需要用户补充的材料

1. **ESP32 WiFi 模块资料** — A板已有的ESP32通信代码（用于移植）、FPGA与ESP32间接口方式（UART/SPI）、引脚连接
2. **A板资料** — A板的Verilog工程或协议文档，用于对接UART通信和播放控制逻辑
3. **A板多媒体内容组织方式** — TF卡中图片如何命名/分组？A板支持哪些播放切换指令？
4. **UART/SPI引脚确认** — B板使用哪个GPIO连接器引出与ESP32和A板的通信引脚？需确认原理图可用引脚