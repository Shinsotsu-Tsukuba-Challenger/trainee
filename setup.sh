#!/bin/bash -evx

TARGET=$1
CACHE=$2
CACHE_PATH=$3

sudo apt update -y
sudo apt install -y python3-vcstool git rsync

mkdir -p $HOME/trainee/
git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git /tmp/trainee
rsync -av /tmp/trainee/ $HOME/trainee/

grep -q "source \$HOME/trainee/install/setup.bash" $HOME/.bashrc || echo "source \$HOME/trainee/install/setup.bash" >> $HOME/.bashrc
grep -q "export TRAINEE_WS=\$HOME/trainee" $HOME/.bashrc || echo "export TRAINEE_WS=\$HOME/trainee" >> $HOME/.bashrc
source $HOME/.bashrc
cd $TRAINEE_WS && mkdir -p src

if [ "$TARGET" == "pc" ]; then
    echo "Setting up for PC"
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
    sudo apt install -y ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-glim-ros
elif [ "$TARGET" == "raspi" ]; then
    echo "Setting up for Raspberry Pi"
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
else
    echo "No or incorrect argument provided. Default setup will be used."
    vcs import src < repos/$ROS_DISTRO/trainee.repos --debug
fi

rosdep update
rosdep install -y --from-paths src --skip-keys odrive_ros2_control --ignore-src --rosdistro $ROS_DISTRO

if [ "$CACHE" == "true" ] && [ -d "$CACHE_PATH" ]; then
    echo "Cache mode: comparing packages in $CACHE_PATH"
    for pkg in $(ls src); do
        echo "Checking package: $pkg"
        cd "src/$pkg"
        local_commit=$(git rev-parse HEAD)
        cache_commit=$(git --git-dir="$CACHE_PATH/$pkg/.git" rev-parse HEAD)
        if [ "$local_commit" == "$cache_commit" ]; then
            echo "Commit match for $pkg. Syncing..."
            rsync -a --delete "$CACHE_PATH/$pkg/" "$TRAINEE_WS/src/$pkg/"
        else
            echo "Commit mismatch for $pkg. Skipping rsync."
        fi
        cd "$TRAINEE_WS"
    done
else
    echo "Cache mode not enabled or $CACHE_PATH does not exist."
fi

# ビルド
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source $HOME/.bashrc
