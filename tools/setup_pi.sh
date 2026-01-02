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
if command -v raspi-config > /dev/null; then
    raspi-config nonint do_i2c 0
    raspi-config nonint do_serial 2 # Disable console on serial, enable hardware UART
    echo "I2C and Serial enabled via raspi-config."
fi

# CRITICAL: Pi Zero 2 W UART Fix
# The Pi Zero shares the main UART with Bluetooth. We MUST disable Bluetooth to get stable flight control comms.
CONFIG_FILE="/boot/config.txt"
if [ -f "/boot/firmware/config.txt" ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
fi

echo ">>> Applying overlays to $CONFIG_FILE..."

# Check and append disable-bt
if ! grep -q "dtoverlay=disable-bt" "$CONFIG_FILE"; then
    echo "dtoverlay=disable-bt" >> "$CONFIG_FILE"
    echo "Bluetooth disabled (Overlay applied)."
fi

# Disable Bluetooth Service
systemctl disable hciuart
systemctl stop hciuart

# 2.5 MEMORY & BLOATWARE OPTIMIZATION
echo ">>> [2.5/5] Reclaiming RAM & CPU..."

# 1. GPU Memory Split (Headless Mode)
# The Pi defaults to giving 64MB+ to the GPU. We are headless. 16MB is enough.
# This frees ~48MB of RAM for Python/ROS2.
if command -v raspi-config > /dev/null; then
    raspi-config nonint do_memory_split 16
    echo "RAM Optimized: GPU memory set to 16MB via raspi-config."
else
    # Fallback manual method
    if ! grep -q "gpu_mem=16" "$CONFIG_FILE"; then
        echo "gpu_mem=16" >> "$CONFIG_FILE"
        echo "RAM Optimized: GPU memory set to 16MB (Manual edit)."
    fi
fi

# 2. Disable Bloatware Services
# We don't need to print from the drone (cups) or handle media keys (triggerhappy).
SERVICES_TO_DISABLE="cups cups-browsed triggerhappy modemmanager wpa_supplicant@wlan0"
# Note: wpa_supplicant is needed for WiFi, but sometimes managed by NetworkManager. Leaving it alone for safety.
SAFE_SERVICES="cups cups-browsed triggerhappy modemmanager"

for SERVICE in $SAFE_SERVICES; do
    if systemctl is-active --quiet "$SERVICE"; then
        echo "Disabling $SERVICE..."
        systemctl stop "$SERVICE"
        systemctl disable "$SERVICE"
    fi
done

# 3. PERFORMANCE TUNING
echo ">>> [3/5] Performance Tuning..."
# Set CPU governor to performance to minimize jitter
echo "performance" | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Note on OVERCLOCKING:
# We do NOT recommend 'arm_freq=1200' without active cooling (fan).
# However, 'force_turbo=1' locks the CPU at base max (1GHz) preventing spin-up lag.
# It invalidates warranty if combined with over_voltage.
# We stick to the 'performance' governor which achieves similar results safely.

# ISOLATE CPU CORE 3
# We want Core 3 DEDICATED to the flight loop. No OS tasks allowed.
CMDLINE_FILE="/boot/cmdline.txt"
if [ -f "/boot/firmware/cmdline.txt" ]; then
    CMDLINE_FILE="/boot/firmware/cmdline.txt"
fi

if ! grep -q "isolcpus=3" "$CMDLINE_FILE"; then
    echo "Isolating CPU Core 3 for Real-Time tasks..."
    # Append to the end of the line (cmdline.txt must be one line)
    sed -i 's/$/ isolcpus=3/' "$CMDLINE_FILE"
fi

# 3.5 IRQ ROUTING (HARDWARE INTERRUPTS)
echo ">>> [3.5/5] Pinning Hardware Interrupts..."
# Disable irqbalance (The "Noisy Neighbor" Manager)
# This daemon moves IRQs around. We want them pinned away from Core 3.
if systemctl is-active --quiet irqbalance; then
    systemctl stop irqbalance
    systemctl disable irqbalance
    echo "irqbalance disabled."
fi

# Set Default SMP Affinity to Cores 0-2 (Mask 7 -> Binary 0111)
# This tells the kernel: "By default, send hardware interrupts to Cores 0, 1, or 2. Leave Core 3 alone."
# We add this to rc.local to persist across reboots.
if ! grep -q "echo 7 > /proc/irq/default_smp_affinity" /etc/rc.local; then
    # Insert before 'exit 0'
    sed -i -e '$i \echo 7 > /proc/irq/default_smp_affinity' /etc/rc.local
    echo "Default IRQ Affinity set to Cores 0-2 (Mask 7) in rc.local."
fi

# 4. INSTALL PYTHON DEPENDENCIES
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
