git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git $HOME/trainee
grep -q "source $HOME/trainee/install/setup.bash" ~/.bashrc || echo "source $HOME/trainee/install/setup.bash" >> ~/.bashrc
grep -q "export TRAINEE_WS=$HOME/trainee" ~/.bashrc || echo "export TRAINEE_WS=$HOME/trainee" >> ~/.bashrc
source $HOME/.bashrc
cd $TRAINEE_WS && mkdir src
vcs import src < trainee.repos --debug
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install
source $HOME/.bashrc