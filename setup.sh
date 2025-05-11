#!/bin/bash -evx

sudo apt update -y
sudo apt install -y python3-vcstool rsync

# 引数の確認
TARGET=$1
VCS_HASH_DIR=${2:-}  # キャッシュディレクトリが指定されていない場合は空

mkdir -p $HOME/trainee/
git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git /tmp/trainee
rsync -av /tmp/trainee/ $HOME/trainee/
grep -q "source $HOME/trainee/install/setup.bash" $HOME/.bashrc || echo "source $HOME/trainee/install/setup.bash" >> $HOME/.bashrc
grep -q "export TRAINEE_WS=$HOME/trainee" $HOME/.bashrc || echo "export TRAINEE_WS=$HOME/trainee" >> $HOME/.bashrc
source $HOME/.bashrc
cd $TRAINEE_WS && mkdir -p src

# リポジトリのインポート
if [ "$1" == "pc" ]; then
    echo "Setting up for PC"
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
    sudo apt update -y
    sudo apt install -y ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-glim-ros
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

echo "Generating hash files for each repository..."

# リポジトリごとにハッシュファイルを作成 & 比較
for repo in $(ls src); do
    repo_path="src/$repo"
    hash_file="${VCS_HASH_DIR}/${repo}.txt"

    # 現在のリポジトリのハッシュを取得
    current_hash=$(git -C $repo_path rev-parse HEAD 2>/dev/null || echo "NO_HASH")

    # キャッシュハッシュの取得
    cached_hash=""
    if [ -f "$hash_file" ]; then
        cached_hash=$(cat "$hash_file")
    fi

    # ハッシュの比較
    if [ "$current_hash" != "$cached_hash" ]; then
        echo "Hash mismatch for $repo. Removing cached build artifacts for $repo."
        rm -rf install/$repo build/$repo log/$repo
    else
        echo "Hash match for $repo. Retaining existing build artifacts."
    fi

    # ハッシュを更新
    echo "$current_hash" > "$hash_file"
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

# ビルド処理
echo "Starting colcon build..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install

source $HOME/.bashrc