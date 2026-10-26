#!/usr/bin/env bash
# =============================================================================
# p6_probe_move_semantics.sh
#
# 目的：确认 AirSim 的 moveToPositionAsync 在**已有目标未完成**时再发一个新目标，
#       是「覆盖」还是「排队」。这直接决定 P6 能否用前瞻跳点（设计文档 §4.1 / 风险 ⑥）。
#
#   覆盖 → 载具会立刻改向 B 点       → 前瞻跳点成立
#   排队 → 载具先飞到 A 点，再去 B    → 前瞻跳点不成立，只能等停稳
#
# 判据（见脚本末尾的分析）：记录 /ground_truth/odom，看"到 A 的距离"曲线
#   · 先减后增（没到 A 就掉头）  → 覆盖
#   · 一直减到接近 0 再转向 B    → 排队
#
# 先决条件：
#   1. 宿主上模拟器已启动且 41451 可达（Windows 侧用常驻后台任务持有它！）
#   2. 客户机已 source ~/uav_ws/devel/setup.bash
#   3. 没有别的节点在抢 /uav/goal（本脚本只起桥接节点）
#
# 跑法：bash p6_probe_move_semantics.sh
# =============================================================================
set -u

PKG=octree_uav_3d_pathfinding
WS="${WS:-$HOME/uav_ws}"
OUT="$HOME/p6_probe_$(date +%m%d-%H%M%S)"
AX=40.0; AY=0.0;  AZ=8.0      # 点 A（东侧远处）
BX=-40.0; BY=0.0; BZ=8.0      # 点 B（西侧远处，与 A 相反方向）
RECORD_SECS=28
T_GOAL_A=3
T_GOAL_B=10

mkdir -p "$OUT"
echo "日志目录: $OUT"

source /opt/ros/noetic/setup.bash 2>/dev/null || true
if [ -f "$WS/devel/setup.bash" ]; then
  source "$WS/devel/setup.bash"
else
  echo "!! 找不到 $WS/devel/setup.bash"; exit 1
fi
rospack find "$PKG" >/dev/null 2>&1 || { echo "!! ROS 找不到包 $PKG"; exit 1; }

hr() { printf '%s\n' "----------------------------------------------------------------------"; }

# ---------------------------------------------------------------------------
# 0) 只起桥接节点（不建图、不规划、不 RViz）
#    用 roslaunch 而不是 rosrun —— 后者不会加载 config/params.yaml，
#    而 host / world_z_offset 这些必须来自 yaml。
# ---------------------------------------------------------------------------
hr; echo "[0] 启动桥接节点（仅 bridge）"; hr
pkill -f 'roslaunch'        2>/dev/null
pkill -f 'airsim_bridge'    2>/dev/null
pkill -f 'astar_planner'    2>/dev/null
sleep 2

pgrep -x rosmaster >/dev/null || { roscore > "$OUT/roscore.log" 2>&1 & sleep 6; }

roslaunch "$PKG" main.launch \
    mapping:=false planning:=false rviz:=false check:=false \
    > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 12

grep -E '连接|connected|airsim|起飞|takeoff' "$OUT/bridge.log" | head -8 | sed 's/^/    /'
if ! pgrep -f 'airsim_bridge' >/dev/null; then
  echo "!! 桥接节点已退出，看 $OUT/bridge.log"
  tail -20 "$OUT/bridge.log" | sed 's/^/    /'
  exit 1
fi

# ---------------------------------------------------------------------------
# 1) 开始记录位姿
# ---------------------------------------------------------------------------
hr; echo "[1] 记录 /ground_truth/odom（${RECORD_SECS}s）"; hr
rostopic echo -p /ground_truth/odom/pose/pose/position > "$OUT/odom.csv" 2>/dev/null &
REC_PID=$!
sleep 2

# ---------------------------------------------------------------------------
# 2) 发点 A，中途改发点 B
# ---------------------------------------------------------------------------
send_goal() {
  local x=$1 y=$2 z=$3 tag=$4
  rostopic pub -1 /uav/goal geometry_msgs/Point "{x: $x, y: $y, z: $z}" >/dev/null 2>&1
  echo "    t≈$(( $(date +%s) - T0 ))s  发 $tag ($x, $y, $z)"
}

T0=$(date +%s)
hr; echo "[2] 时间线（点 A 未到达时改发点 B）"; hr
sleep $T_GOAL_A
send_goal $AX $AY $AZ A
sleep 1
send_goal $AX $AY $AZ "A(重复，确保生效)"

sleep $(( T_GOAL_B - T_GOAL_A - 1 ))
send_goal $BX $BY $BZ B
sleep 1
send_goal $BX $BY $BZ "B(重复)"

sleep 6
kill $REC_PID 2>/dev/null
sleep 1

# ---------------------------------------------------------------------------
# 3) 分析
# ---------------------------------------------------------------------------
hr; echo "[3] 分析"; hr
python3 - "$OUT/odom.csv" $AX $AY $AZ $BX $BY $BZ <<'PYEOF'
import math, sys

csv = sys.argv[1]
ax, ay, az = map(float, sys.argv[2:5])
bx, by, bz = map(float, sys.argv[5:8])

skip = {'%time', 'field.header.seq', 'field.header.stamp', 'field.header.frame_id'}
rows = []
with open(csv, encoding='utf-8', errors='replace') as fh:
    for line in fh:
        line = line.strip()
        if not line or line.startswith('%'):
            continue
        parts = line.split(',')
        if len(parts) < 4 or parts[0] in skip:
            continue
        try:
            t, x, y, z = (float(v) for v in parts[:4])
        except ValueError:
            continue
        rows.append((t, x, y, z))

if len(rows) < 10:
    print('!! 采到的位姿点太少（%d），检查 /ground_truth/odom' % len(rows))
    sys.exit(1)

t0 = rows[0][0]
print('  %6s %9s %9s %9s %9s %9s' % ('t(s)', 'x', 'y', 'z', '到A', '到B'))
prev_da = None
turned_at = None
min_da = None
min_da_t = None
for t, x, y, z in rows:
    da = math.dist((x, y, z), (ax, ay, az))
    db = math.dist((x, y, z), (bx, by, bz))
    if min_da is None or da < min_da:
        min_da, min_da_t = da, t - t0
    # 判据：到 A 的距离先减小、后增大，且增大时仍离 A 很远 → 掉头了（覆盖）
    if prev_da is not None and da > prev_da + 0.05 and da > 3.0 and turned_at is None:
        turned_at = t - t0
    prev_da = da
    print('  %6.1f %9.2f %9.2f %9.2f %9.2f %9.2f' % (t - t0, x, y, z, da, db))

last = rows[-1]
final_da = math.dist(last[1:4], (ax, ay, az))
final_db = math.dist(last[1:4], (bx, by, bz))

print()
print('  最接近 A 的距离: %.2f m（t≈%.1fs）' % (min_da, min_da_t))
print('  结束时 到A=%.2f m  到B=%.2f m' % (final_da, final_db))
print()
print('  ===== 结论 =====')
if turned_at is not None and min_da > 3.0:
    print('  覆盖（override）：载具在 t≈%.1fs 掉头，最近也只到 A 的 %.1f m 处' % (turned_at, min_da))
    print('  → 前瞻跳点成立，可用 /uav/goal_cmd 逐点改写目标')
elif final_da < 3.0:
    print('  排队（queue）：载具先飞到了 A（末距 %.1f m），再去 B' % final_da)
    print('  → 前瞻跳点不成立，只能等每个点停稳再发下一点')
else:
    print('  不明确：需要看上面的表格人工判断')
    print('  提示：若"到A"曲线单调下降到很小 → 排队；若中途转为上升 → 覆盖')
PYEOF

# ---------------------------------------------------------------------------
# 4) 收尾
# ---------------------------------------------------------------------------
hr; echo "[4] 收尾"; hr
kill $BRIDGE_PID 2>/dev/null
pkill -f 'airsim_bridge' 2>/dev/null
echo "  已停止桥接节点。模拟器请自行处理（宿主侧）。"
echo "  日志与位姿数据: $OUT"
echo
echo "把 [3] 分析那一段完整贴回给我。"
