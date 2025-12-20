#!/bin/bash

HOSTS_FILE="/etc/hosts"
HOST_IP="192.168.1.39"
HOST_NAME="vicon"

RPI_CONFIG_FILE="/boot/firmware/config.txt"

PWM_CONFIG="dtoverlay=pwm"
UART_CONFIG_1="enable_uart=1"
UART_CONFIG_2="dtoverlay=disable-bt,uart0,ctsrts"
CAMERA_CONFIG="dtoverlay=ov9281,cam0"
LED_CONFIG="dtparam=spi=on"

CONFIGS_TO_ADD=(
    "$PWM_CONFIG"
    "$UART_CONFIG_1"
    "$UART_CONFIG_2"
    "$CAMERA_CONFIG"
    "$LED_CONFIG"
)

CAM_LOC_REPO="https://github.com/flslab/fls-marker-localization.git"
CAM_LOC_REPO_DIR="~/fls-marker-localization"
MAVPROXY_REPO="https://github.com/Hamedamz/MAVProxyVicon"
MAVPROXY_REPO_DIR="~/MAVProxyVicon"

sudo -v

# Adding configs

if [ ! -f "$RPI_CONFIG_FILE" ]; then
    echo "Error: File $RPI_CONFIG_FILE not found."
    exit 1
fi

echo "Checking configuration in $RPI_CONFIG_FILE..."

for config in "${CONFIGS_TO_ADD[@]}"; do
    # -F: Fixed string (not regex)
    # -x: Exact line match
    # -q: Quiet (no output)
    if grep -Fxq "$config" "$RPI_CONFIG_FILE"; then
        echo "  [OK] Already exists: $config"
    else
        echo "  [UPDATE] Adding: $config"
        # Echo the config line and pipe it into sudo tee -a (append)
        # > /dev/null suppresses the output of tee to the screen
        echo "$config" | sudo tee -a "$RPI_CONFIG_FILE" > /dev/null
    fi
done

echo ""

# Setting the Vicon host name
echo "Checking $HOSTS_FILE..."

# Check using regex (-e):
# ^          = Start of line (prevents matching 1192.168...)
# [[:space:]]+ = One or more spaces or tabs
if grep -q -e "^$HOST_IP[[:space:]]\+$HOST_NAME" "$HOSTS_FILE"; then
    echo "  [OK] Entry already exists: $HOST_NAME"
else
    echo "  [UPDATE] Adding: $HOST_IP $HOST_NAME"
    # printf generates the line with a tab (\t) and a newline (\n)
    # We pipe it to 'sudo tee -a' to write to the protected file
    printf "$HOST_IP\t$HOST_NAME\n" | sudo tee -a "$HOSTS_FILE" > /dev/null
fi

mkdir logs -p

# Install motioncapture lib dependencies
sudo apt install libboost-system-dev libboost-thread-dev libeigen3-dev ninja-build
pip install -r requirements.txt
pip install -r requirements_servo.txt
pip install -r requirements_led.txt

# install modified pymavlink
if [ -d "$MAVPROXY_REPO_DIR" ]; then
    echo "Skip: Repository already cloned at MAVProxyVicon"
else
    echo "Cloning repository..."
    git clone "$MAVPROXY_REPO" "$MAVPROXY_REPO_DIR"
fi
cd "$MAVPROXY_REPO_DIR"
python setup.py build install
sudo apt install libgtk-3-dev
pip install pathlib2

# Install marker localization
if [ -d "$CAM_LOC_REPO_DIR" ]; then
    echo "Skip: Repository already cloned at $CAM_LOC_REPO_DIR"
else
    echo "Cloning repository..."
    git clone "$CAM_LOC_REPO" "$CAM_LOC_REPO_DIR"
fi
sudo apt install -y cmake libopencv-dev nlohmann-json3-dev libeigen3-dev libcamera-dev
cd "$CAM_LOC_REPO_DIR"
mkdir build -p
cd build
mkdir logs -p
cmake ..
make -j4
cp ../src/gs_camera_config.json camera_config.json
