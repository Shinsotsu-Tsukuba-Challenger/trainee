# trainee

## Install vcstool

```
sudo apt install python3-vcstool
```

## How to set up workspace

* Install

```
git clone git@github.com:Shinsotsu-Tsukuba-Challenger/trainee.git $HOME/trainee
grep -q "source $HOME/trainee/install/setup.bash" ~/.bashrc || echo "source $HOME/trainee/install/setup.bash" >> ~/.bashrc
grep -q "export TRAINEE_WS=$HOME/trainee" ~/.bashrc || echo "export TRAINEE_WS=$HOME/trainee" >> ~/.bashrc
cd $TRAINEE && mkdir src
vcs import src < trainee.repos --debug
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install
source $HOME/.bashrc
```

* Update

```
vcs import src < trainee.repos --debug
vcs pull src --debug
```