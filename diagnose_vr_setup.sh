#!/bin/bash
# 快速诊断VR指令跨进程通信设置

echo "============================================================"
echo "VR Command Cross-Process Communication Diagnostics"
echo "============================================================"
echo ""

# Check if ROS2 is running
echo "[1/5] Checking ROS2 environment..."
if command -v ros2 &> /dev/null; then
    echo "    ✓ ROS2 command found"
else
    echo "    ✗ ROS2 not found. Please source ROS2 setup."
    exit 1
fi

# Check for VR command topics
echo ""
echo "[2/5] Checking VR command topics..."
TOPICS=$(ros2 topic list 2>/dev/null)

if echo "$TOPICS" | grep -q "/left/vr_sent_command"; then
    echo "    ✓ /left/vr_sent_command topic exists"
    LEFT_HZ=$(ros2 topic hz /left/vr_sent_command --window 10 2>&1 | grep "average rate" | head -1)
    echo "      $LEFT_HZ"
else
    echo "    ⚠ /left/vr_sent_command topic NOT found"
    echo "      (This is normal if teleoperation is not running)"
fi

if echo "$TOPICS" | grep -q "/right/vr_sent_command"; then
    echo "    ✓ /right/vr_sent_command topic exists"
    RIGHT_HZ=$(ros2 topic hz /right/vr_sent_command --window 10 2>&1 | grep "average rate" | head -1)
    echo "      $RIGHT_HZ"
else
    echo "    ⚠ /right/vr_sent_command topic NOT found"
    echo "      (This is normal if teleoperation is not running)"
fi

# Check for controller state topics
echo ""
echo "[3/5] Checking controller state topics..."
if echo "$TOPICS" | grep -q "/left/dg3f_b_controller/controller_state"; then
    echo "    ✓ /left/dg3f_b_controller/controller_state exists"
else
    echo "    ✗ Left controller state topic NOT found"
fi

if echo "$TOPICS" | grep -q "/right/dg3f_b_controller/controller_state"; then
    echo "    ✓ /right/dg3f_b_controller/controller_state exists"
else
    echo "    ✗ Right controller state topic NOT found"
fi

# Check CSV log files
echo ""
echo "[4/5] Checking CSV log files..."
if [ -f "/tmp/openteach_left_sent_commands.csv" ]; then
    LINES=$(wc -l < /tmp/openteach_left_sent_commands.csv)
    echo "    ✓ Left CSV log exists ($LINES lines)"
else
    echo "    ⚠ Left CSV log NOT found"
fi

if [ -f "/tmp/openteach_right_sent_commands.csv" ]; then
    LINES=$(wc -l < /tmp/openteach_right_sent_commands.csv)
    echo "    ✓ Right CSV log exists ($LINES lines)"
else
    echo "    ⚠ Right CSV log NOT found"
fi

# Check Python scripts
echo ""
echo "[5/5] Checking helper scripts..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -f "$SCRIPT_DIR/test_cross_process_vr_command.py" ]; then
    echo "    ✓ test_cross_process_vr_command.py found"
else
    echo "    ✗ test_cross_process_vr_command.py NOT found"
fi

if [ -f "$SCRIPT_DIR/verify_vr_command.py" ]; then
    echo "    ✓ verify_vr_command.py found"
else
    echo "    ✗ verify_vr_command.py NOT found"
fi

# Summary
echo ""
echo "============================================================"
echo "Summary"
echo "============================================================"
echo ""
echo "If teleoperation is running:"
echo "  - VR command topics should exist"
echo "  - CSV logs should be updating"
echo "  - Topic frequency should be ~60-90 Hz"
echo ""
echo "To test cross-process communication:"
echo "  Terminal 1: python teleop.py robot=both"
echo "  Terminal 2: python test_cross_process_vr_command.py left"
echo ""
echo "To verify collected data:"
echo "  python verify_vr_command.py extracted_data/demonstration_XX"
echo ""
echo "============================================================"
