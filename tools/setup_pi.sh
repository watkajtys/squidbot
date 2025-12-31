#!/bin/bash

# SQUID DRONE SETUP SCRIPT
# Target: Raspberry Pi Zero 2 W (Bookworm or Bullseye)
# Run as: sudo ./setup_pi.sh

set -e # Exit on error

echo ">>> STARTING SQUID DRONE SETUP <<<"
echo "Note: This script assumes you are running on the Pi."

# 1. UPDATE SYSTEM
echo ">>> [1/5] Updating System Packages..."
apt update && apt upgrade -y
apt install -y git python3-pip i2c-tools python3-smbus python3-dev build-essential libatlas-base-dev

# 2. CONFIGURE INTERFACES
echo ">>> [2/5] Enabling Hardware Interfaces..."
# Enable I2C and UART (Hardware Serial)
# This modifies /boot/config.txt indirectly via raspi-config non-interactive mode
if command -v raspi-config > /dev/null; then
    raspi-config nonint do_i2c 0
    raspi-config nonint do_serial 2 # Disable console on serial, enable hardware UART
    echo "I2C and Serial enabled."
else
    echo "WARNING: raspi-config not found. Please enable I2C and UART manually in /boot/config.txt"
fi

# 3. INSTALL PYTHON DEPENDENCIES
echo ">>> [3/5] Installing Python Libraries..."
# Using --break-system-packages because Pi OS Bookworm is strict. 
# Ideally, use a venv, but for a dedicated drone controller, system-wide is often preferred for service access.
pip3 install rpi.gpio smbus2 spidev numpy scipy matplotlib flask python-socketio eventlet pyserial rich --break-system-packages

# 4. SETUP ROS2 (HUMBLE)
# Note: Installing ROS2 on Pi Zero is heavy. We use a minimal install if possible, 
# or recommend the user flash a pre-built image. 
# This section adds the repo but lets the user decide when to install the full desktop/base.
echo ">>> [4/5] Setting up ROS2 Sources (Humble)..."
apt install -y software-properties-common
add-apt-repository universe
apt update && apt install -y curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update

echo ">>> ROS2 sources added. To install ROS2 Base (recommended), run:"
echo "    sudo apt install ros-humble-ros-base"

# 5. PERMISSIONS
echo ">>> [5/5] Setting Permissions..."
usermod -aG i2c,gpio,dialout $SUDO_USER

echo ">>> SETUP COMPLETE! <<<"
echo "Please reboot your Pi: sudo reboot"
