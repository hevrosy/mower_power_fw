# V3 safety fix

This version adds protection against Arduino Nano USB/DTR reset and false early BUTTON_PRESS events. Arduino prints `PWRBOOT version=3`. The ROS node cancels early BUTTON_PRESS shutdown requests during the startup guard, and with `auto_shutdown:=false` it sends `CANCEL_SHUTDOWN` instead of leaving Arduino waiting to cut power.

# Mower power monitor integration

Files:
- `src/mower_control_ros2/src/power_monitor_node.cpp`
- `src/mower_control_ros2/CMakeLists.txt`
- `arduino/SmartPowerArduino.ino`
- `systemd/mower-power-monitor.service`
- `udev/99-mower-power-arduino.rules`

Build:
```bash
cd ~/dev_ws
colcon build --packages-select mower_control_ros2
source install/setup.bash
```

Manual run:
```bash
ros2 run mower_control_ros2 power_monitor_node --ros-args \
  -p serial_port:=/dev/mower_power_arduino \
  -p baudrate:=115200 \
  -p auto_shutdown:=true \
  -p power_cut_grace_ms:=90000 \
  -p shutdown_command:="/sbin/shutdown -h now"
```

Manual test without shutting down the Pi:
```bash
ros2 run mower_control_ros2 power_monitor_node --ros-args \
  -p serial_port:=/dev/mower_power_arduino \
  -p auto_shutdown:=false
```

Topics:
```bash
ros2 topic echo /power_status
ros2 topic echo /power_events
ros2 topic pub --once /power_commands std_msgs/msg/String "{data: 'STATUS?'}"
ros2 topic pub --once /power_commands std_msgs/msg/String "{data: 'SHUTDOWN_NOW'}"
```

Install systemd service:
```bash
sudo cp systemd/mower-power-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mower-power-monitor.service
sudo systemctl start mower-power-monitor.service
sudo systemctl status mower-power-monitor.service
journalctl -u mower-power-monitor.service -f
```

Important hardware note:
- Do not let the Arduino be powered from Raspberry Pi USB if it has its own 5 V supply.
- Use a data-only USB cable or cut/isolate USB +5 V (VBUS/red wire).
- Keep D+, D-, and GND connected.
