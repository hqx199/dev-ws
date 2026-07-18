#!/bin/bash
# 惯性导航测试启动脚本
# 此脚本会按顺序启动所有必要的节点

set -e

echo "=========================================="
echo "  惯性导航测试系统启动"
echo "=========================================="

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/RacingDev/dev_ws/install/setup.bash

echo ""
echo "步骤 1/3: 检查 EKF 融合节点..."
if ! ros2 topic list | grep -q "/odom_combined"; then
    echo "  ⚠️  EKF 节点未运行，请先启动基础系统："
    echo "     ./start_origicar.sh"
    echo "  或："
    echo "     ros2 launch origincar_base ekf_rdk_imu.launch.py"
    exit 1
else
    echo "  ✓ EKF 节点正常运行"
fi

echo ""
echo "步骤 2/3: 启动惯性导航控制器..."
ros2 launch racing_inertial_nav inertial_nav.launch.py &
NAV_PID=$!

echo "  ✓ 惯性导航控制器已启动 (PID: $NAV_PID)"

echo ""
echo "步骤 3/3: 监控系统状态..."
sleep 2

echo ""
echo "=========================================="
echo "  系统启动完成！"
echo "=========================================="
echo ""
echo "监控命令："
echo "  查看里程计数据: ros2 topic echo /odom_combined"
echo "  查看控制指令:   ros2 topic echo /cmd_vel"
echo "  查看TF变换:     ros2 run tf2_ros tf2_echo odom_combined base_footprint"
echo ""
echo "停止系统: Ctrl+C"
echo "=========================================="

# Wait for navigation node
wait $NAV_PID
