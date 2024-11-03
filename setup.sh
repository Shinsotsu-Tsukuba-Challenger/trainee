#!/bin/bash -evx

sudo apt update -y
sudo apt install -y python3-vcstool

git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git $HOME/trainee
grep -q "source $HOME/trainee/install/setup.bash" $HOME/.bashrc || echo "source $HOME/trainee/install/setup.bash" >> $HOME/.bashrc
grep -q "export TRAINEE_WS=$HOME/trainee" $HOME/.bashrc || echo "export TRAINEE_WS=$HOME/trainee" >> $HOME/.bashrc
source $HOME/.bashrc
cd $TRAINEE_WS && mkdir src

if [ "$1" == "pc" ]; then
    echo "Setting up for PC"
    vcs import src < trainee_pc.repos --debug
    sudo apt update -y
    rosdep update
    rosdep install -y --from-paths src --skip-keys odrive_ros2_control --ignore-src --rosdistro $ROS_DISTRO
elif [ "$1" == "raspi" ]; then
    echo "Setting up for Raspberry Pi"
    vcs import src < trainee_raspi.repos --debug
    sudo apt update -y
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
else
    echo "No or incorrect argument provided. Default setup will be used."
    vcs import src < trainee.repos --debug
fi

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source $HOME/.bashrc