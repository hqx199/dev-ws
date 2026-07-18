#!/bin/bash
# ========================================
# 惯性导航 + YOLO避障 一键启动脚本
# 自动检测源码变更并编译，检查依赖
# 用法:
#   bash run_inertial_nav.sh           # 正常启动
#   bash run_inertial_nav.sh --log     # 启动并保存日志到 logs/ 目录
# ========================================
set -e

# 颜色
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

WORKSPACE="/home/sunrise/RacingDev/dev_ws"
SRC_DIR="${WORKSPACE}/src/racing/racing_inertial_nav"
CONFIG="${SRC_DIR}/config/params.yaml"
EXECUTABLE="${WORKSPACE}/install/racing_inertial_nav/lib/racing_inertial_nav/inertial_nav_controller"
SRC_CPP="${SRC_DIR}/src/inertial_nav_controller.cpp"
SRC_HPP="${SRC_DIR}/include/inertial_nav_controller.hpp"

# ========================================
# 参数解析
# ========================================
ENABLE_LOG=false
for arg in "$@"; do
    case "$arg" in
        --log|-l)
            ENABLE_LOG=true
            ;;
        --help|-h)
            echo "用法: bash run_inertial_nav.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --log, -l    保存运行日志到 logs/ 目录（文件名含时间戳）"
            echo "  --help, -h   显示此帮助信息"
            exit 0
            ;;
    esac
done

# ========================================
# 日志初始化（必须在其他输出之前）
# ========================================
LOG_FILE=""
if $ENABLE_LOG; then
    LOG_DIR="${WORKSPACE}/logs"
    mkdir -p "$LOG_DIR"
    LOG_FILE="${LOG_DIR}/inertial_nav_$(date +%Y%m%d_%H%M%S).log"
    # 所有输出同时送到终端（带颜色）和日志文件
    # 查看日志: less -R logs/xxx.log  (-R 正确渲染颜色)
    # 去除颜色: sed 's/\x1b\[[0-9;]*m//g' logs/xxx.log
    exec > >(tee -a "$LOG_FILE") 2>&1
fi

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  🚀 惯性导航 + YOLO避障 启动${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# ========================================
# 1. 加载环境
# ========================================
echo -e "${YELLOW}[1/5] 加载环境...${NC}"
export ROS_DOMAIN_ID=0
source /opt/tros/humble/setup.bash
source "${WORKSPACE}/setup_origicar_env.sh" 2>/dev/null || true
source "${WORKSPACE}/install/setup.bash" 2>/dev/null || true
export AMENT_PREFIX_PATH=/opt/tros/humble:$AMENT_PREFIX_PATH
echo -e "${GREEN}      ✓ 环境就绪${NC}"

# ========================================
# 2. 检查依赖
# ========================================
echo -e "${YELLOW}[2/5] 检查依赖...${NC}"

# 检查 YOLO 检测话题
if ros2 topic info /racing_obstacle_detection &>/dev/null 2>&1; then
    echo -e "${GREEN}      ✓ YOLO检测在线 (/racing_obstacle_detection)${NC}"
    YOLO_OK=true
else
    echo -e "${YELLOW}      ⚠ YOLO检测未运行，避障功能将不可用${NC}"
    echo -e "${YELLOW}        请先启动: bash dev_ws/start_yolo.sh${NC}"
    YOLO_OK=false
fi

# 检查里程计话题
if ros2 topic info /odom_combined &>/dev/null 2>&1; then
    echo -e "${GREEN}      ✓ EKF里程计在线 (/odom_combined)${NC}"
else
    echo -e "${RED}      ✗ EKF里程计未就绪，请先启动底盘${NC}"
    echo -e "${RED}        请先启动: bash dev_ws/start_origicar.sh --no-camera${NC}"
fi

# 检查配置文件
if [ ! -f "$CONFIG" ]; then
    echo -e "${RED}      ✗ 配置文件不存在: $CONFIG${NC}"
    exit 1
fi
echo -e "${GREEN}      ✓ 配置文件: ${CONFIG}${NC}"

# ========================================
# 3. 显示当前配置
# ========================================
echo -e "${YELLOW}[3/5] 当前配置...${NC}"

# 提取航点
WAYPOINTS=$(grep -E '^\s+[0-9]+\.[0-9]+,' "$CONFIG" | grep -v '^#' | wc -l)
echo "      航点数量: ${WAYPOINTS}"

# 显示前3个航点
grep -E '^\s+[0-9]+\.[0-9]+,' "$CONFIG" | grep -v '^#' | head -3 | while read line; do
    echo "        → $line"
done

# 显示避障参数
AVOID_AREA=$(grep 'avoid_area:' "$CONFIG" | awk '{print $2}')
AVOID_SPEED=$(grep 'avoid_linear_speed:' "$CONFIG" | awk '{print $2}')
echo "      避障触发面积: ${AVOID_AREA:-默认}"
echo "      避障速度: ${AVOID_SPEED:-默认} m/s"

# ========================================
# 4. 编译（按需）
# ========================================
echo -e "${YELLOW}[4/5] 检查编译...${NC}"

NEED_BUILD=false
if [ ! -f "$EXECUTABLE" ]; then
    echo "      可执行文件不存在，需要编译"
    NEED_BUILD=true
elif [ "$SRC_CPP" -nt "$EXECUTABLE" ] || [ "$SRC_HPP" -nt "$EXECUTABLE" ]; then
    echo "      检测到源码变更，需要重新编译"
    NEED_BUILD=true
fi

if $NEED_BUILD; then
    echo -e "${YELLOW}      ⚙ 编译中...${NC}"
    cd "$WORKSPACE"
    set +e
    colcon build --packages-select racing_inertial_nav 2>&1 | tail -5
    BUILD_RC=${PIPESTATUS[0]}
    set -e
    if [ $BUILD_RC -ne 0 ]; then
        echo -e "${RED}      ✗ 编译失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}      ✓ 编译完成${NC}"
else
    echo -e "${GREEN}      ✓ 可执行文件已是最新${NC}"
fi

# 日志配置（已在脚本开头初始化，此处仅显示路径）
if [ -n "$LOG_FILE" ]; then
    echo -e "${GREEN}📄 日志将保存至: ${LOG_FILE}${NC}"
fi

# ========================================
# 5. 退出清理
# ========================================
cleanup() {
    echo ""
    echo -e "${YELLOW}🛑 正在停止，发送零速指令...${NC}"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        '{linear:{x:0.0,y:0.0,z:0.0},angular:{x:0.0,y:0.0,z:0.0}}' 2>/dev/null || true
    if [ -n "$LOG_FILE" ] && [ -f "$LOG_FILE" ]; then
        echo -e "${GREEN}📄 日志已保存: ${LOG_FILE}${NC}"
    fi
    echo -e "${GREEN}✅ 已停止${NC}"
    exit 0
}
trap cleanup INT TERM

# ========================================
# 6. 启动
# ========================================
echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  启动惯性导航控制器${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${GREEN}按 Ctrl+C 停止并发送零速指令${NC}"
if [ -n "$LOG_FILE" ]; then
    echo -e "${GREEN}📄 日志保存至: ${LOG_FILE}${NC}"
fi
echo ""

"$EXECUTABLE" --ros-args --params-file "$CONFIG"
