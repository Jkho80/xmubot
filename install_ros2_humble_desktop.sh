#!/bin/bash
set -e

# ---------------------------
# 基础配置
# ---------------------------
echo "🚀 开始安装ROS2 Humble (Ubuntu 22.04)"
echo "注：若遇到问题，请查看终端输出或联系开发者"

# 使用清华镜像源加速
export MIRROR="https://mirrors.tuna.tsinghua.edu.cn"
echo "🔧 正在配置镜像源: $MIRROR"

# ---------------------------
# 第1步：系统准备
# ---------------------------
echo -e "\n📦 步骤1/6: 系统更新与基础工具安装"
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl software-properties-common python3-pip gnupg2

# ---------------------------
# 第2步：配置ROS2仓库
# ---------------------------
echo -e "\n🔑 步骤2/6: 配置ROS2仓库"
# 添加ROS2 GPG密钥
# echo "185.199.108.133 raw.githubusercontent.com" >> /etc/hosts

sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加仓库源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# ---------------------------
# 第3步：安装ROS2
# ---------------------------
echo -e "\n💻 步骤3/6: 安装ROS2 Humble"
sudo apt update
sudo apt upgrade
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions ros-dev-tools

# ---------------------------
# 第4步：环境配置
# ---------------------------
echo -e "\n⚙️ 步骤4/6: 环境配置"
# 自动检测用户使用的shell类型
if [ -n "$BASH_VERSION" ]; then
    SHELL_RC="$HOME/.bashrc"
elif [ -n "$ZSH_VERSION" ]; then
    SHELL_RC="$HOME/.zshrc"
else
    SHELL_RC="$HOME/.bashrc"
fi

# 添加环境变量
if ! grep -q "source /opt/ros/humble/setup.bash" "$SHELL_RC"; then
    echo -e "\n# ROS2 Humble" >> "$SHELL_RC"
    echo "source /opt/ros/humble/setup.bash" >> "$SHELL_RC"
fi
source "$SHELL_RC"

# ---------------------------
# 第5步：创建工作区
# ---------------------------
echo -e "\n📁 步骤5/6: 创建工作区"
WORKSPACE_DIR="$HOME/xmu_ros2_ws"
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    mkdir -p "$WORKSPACE_DIR/src"
    echo "  工作区已创建: $WORKSPACE_DIR"
else
    echo "  工作区已存在，跳过创建"
fi

# ---------------------------
# 第6步：验证安装
# ---------------------------
echo -e "\n✅ 步骤6/6: 验证安装"
echo -e "\n----------------------------------------"
echo "安装完成！请按以下步骤验证："
echo "1. 打开新终端"
echo "2. 运行命令: ros2 run demo_nodes_cpp talker"
echo "3. 再开一个终端运行: ros2 run demo_nodes_py listener"
echo -e "----------------------------------------\n"