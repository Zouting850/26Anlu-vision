#!/bin/sh
# 回归: 先对账 TD 工程登记, 再跑五个纯 RTL testbench (1 颜色转换 / 2 乒乓 / 3 显示通路端到端
#                                       / 4 火焰检测单元 / 5 蒙版落位端到端)
#   加密 IP (sdr_*.enc.v / *.enc.vhd) 与 TD 生成的 ip/*.v 用 sim/lint_stubs.v 顶替,
#   所以这一步只检“有没有模块忘了登记进 .al” —— 这正是 TD 会报 black box 的那类错。
# 用法: cd 26Anlu-vision && sh sim/run_all.sh
set -e
cd "$(dirname "$0")/.."
mkdir -p sim/out

echo "=== 0. 工程登记对账 (.al 清单 vs 实际例化) ==="
ALFILES=`python - <<'PY'
import re, xml.etree.ElementTree as ET
raw=open('26Anlu-vision.al','rb').read().decode('utf-8')
root=ET.fromstring(re.sub(r'&(?!amp;|lt;|gt;|quot;|apos;)','&amp;',raw))
print(' '.join(f.get('Path') for f in root.iter('File')
               if f.get('Path').endswith('.v') and not f.get('Path').endswith('.enc.v')
               and f.get('Path') != 'src/sdram/sdram.v'))
PY`
# 磁盘上有、但没登记的纯 .v (只作提示: 没被例化就不必登记)
for f in `find src -name '*.v' ! -name '*.enc.v' | sed 's|^\./||'`; do
  case " $ALFILES src/sdram/sdram.v " in
    *" $f "*) ;;
    *) echo "  [提示] 未登记(未被例化则无需登记): $f" ;;
  esac
done
iverilog -g2012 -tnull -I src/video -s top $ALFILES sim/lint_stubs.v 2>&1 \
  | grep -i "Unknown module" && { echo "FAIL: 有模块没登记进 .al, TD 会报 black box"; exit 1; } \
  || echo "  登记完整, 无黑盒 ✓"

echo "=== 1. tb_rgb_to_ycbcr ==="
iverilog -g2012 -o sim/out/tb_ycbcr.vvp sim/tb_rgb_to_ycbcr.v src/vision/rgb_to_ycbcr.v 2>/dev/null
vvp sim/out/tb_ycbcr.vvp | grep -aE "值域|比对像素|PASS|FAIL"

echo "=== 2. tb_frame_buffer_ctrl ==="
iverilog -g2012 -o sim/out/tb_fb.vvp sim/tb_frame_buffer_ctrl.v src/memory/frame_buffer_ctrl.v 2>/dev/null
vvp sim/out/tb_fb.vvp | grep -aE "统计|提交|重叠|PASS|FAIL"

echo "=== 3. tb_display_path (真实仲裁 + 真实异步 FIFO) ==="
iverilog -g2012 -o sim/out/tb_dp.vvp sim/tb_display_path.v \
  src/video/display_path.v src/video/video_delay.v src/vision/rgb_to_ycbcr.v \
  src/vision/fire_detector.v src/vision/fire_region_analyzer.v \
  src/memory/frame_buffer_ctrl.v src/memory/frame_read_write.v \
  src/memory/frame_fifo_write.v src/memory/frame_fifo_read.v \
  ip/afifo_16_32_256.v ip/afifo_32_16_256.v 2>/dev/null
vvp sim/out/tb_dp.vvp | grep -aE "统计|检查|覆盖|错位|撞拍|PASS|FAIL"

echo "=== 4. tb_fire_detector (色域真值表 + 块密度/报警/质心) ==="
iverilog -g2012 -o sim/out/tb_fire.vvp sim/tb_fire_detector.v \
  src/vision/fire_detector.v src/vision/fire_region_analyzer.v 2>/dev/null
vvp sim/out/tb_fire.vvp | grep -aE "遍|块|统计|真值|PASS|FAIL"

echo "=== 5. tb_fire_overlay (真实读通道下的蒙版落位) ==="
iverilog -g2012 -o sim/out/tb_fov.vvp sim/tb_fire_overlay.v \
  src/video/display_path.v src/video/video_delay.v src/vision/rgb_to_ycbcr.v \
  src/vision/fire_detector.v src/vision/fire_region_analyzer.v \
  src/memory/frame_buffer_ctrl.v src/memory/frame_read_write.v \
  src/memory/frame_fifo_write.v src/memory/frame_fifo_read.v \
  ip/afifo_16_32_256.v ip/afifo_32_16_256.v 2>/dev/null
vvp sim/out/tb_fov.vvp | grep -aE "遍|块|蒙版|落位|统计|PASS|FAIL"
