git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git $HOME/trainee
grep -q "source $HOME/trainee/install/setup.bash" $HOME/.bashrc || echo "source $HOME/trainee/install/setup.bash" >> $HOME/.bashrc
grep -q "export TRAINEE_WS=$HOME/trainee" $HOME/.bashrc || echo "export TRAINEE_WS=$HOME/trainee" >> $HOME/.bashrc
source $HOME/.bashrc
cd $TRAINEE_WS && mkdir src
vcs import src < trainee.repos --debug
sudo apt update -y
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source $HOME/.bashrc