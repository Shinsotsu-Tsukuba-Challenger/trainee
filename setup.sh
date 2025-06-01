#!/bin/bash -evx

sudo apt update -y
sudo apt install -y python3-vcstool

# 引数の確認
TARGET=$1
VCS_HASH_DIR=${2:-}  # キャッシュディレクトリが指定されていない場合は空

git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git $HOME/trainee
grep -q "source $HOME/trainee/install/setup.bash" $HOME/.bashrc || echo "source $HOME/trainee/install/setup.bash" >> $HOME/.bashrc
grep -q "export TRAINEE_WS=$HOME/trainee" $HOME/.bashrc || echo "export TRAINEE_WS=$HOME/trainee" >> $HOME/.bashrc
source $HOME/.bashrc
cd $TRAINEE_WS && mkdir -p src

# リポジトリのインポート
if [ "$1" == "pc" ]; then
    echo "Setting up for PC"
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
    sudo apt update -y
    sudo apt install -y ros-$ROS_DISTRO-gazebo-*
    rosdep update
    rosdep install -y --from-paths src --skip-keys odrive_ros2_control --ignore-src --rosdistro $ROS_DISTRO
elif [ "$1" == "raspi" ]; then
    echo "Setting up for Raspberry Pi"
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
    sudo apt update -y
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
else
    echo "No or incorrect argument provided. Default setup will be used."
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
fi

# キャッシュ未指定時のフルビルド
if [ -z "$VCS_HASH_DIR" ]; then
    echo "No cache directory specified. Proceeding with full build."
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install
    source $HOME/.bashrc
    exit 0
fi

# キャッシュディレクトリの作成
mkdir -p "$VCS_HASH_DIR"

# リポジトリごとにハッシュファイルを作成
for repo in $(ls src); do
    find src/$repo -type d -exec git -C {} rev-parse HEAD \; > "${VCS_HASH_DIR}/${repo}.txt"
done

echo "Checking for existing build artifacts..."

# キャッシュが存在しない場合はフルビルド
if [ ! -d "install" ] || [ ! -d "build" ] || [ ! -d "log" ]; then
    echo "No existing build artifacts found. Performing full build..."
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install
    source $HOME/.bashrc
    exit 0
fi

# キャッシュが存在する場合の整理
echo "Existing build artifacts found. Checking for missing parts..."

for repo in $(ls src); do
    # キャッシュが存在しない場合は該当フォルダを削除
    if [ ! -f "${VCS_HASH_DIR}/${repo}.txt" ]; then
        echo "Cache for $repo is missing. Cleaning up..."
        rm -rf install/$repo build/$repo log/$repo
    fi
done

# ビルド処理
echo "Starting colcon build..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install

source $HOME/.bashrc
