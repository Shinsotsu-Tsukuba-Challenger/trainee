# trainee

## Install vcstool

```
sudo apt update
sudo apt install python3-vcstool
```

## How to set up workspace

* Install

```
bash <(curl -s https://raw.githubusercontent.com/Shinsotsu-Tsukuba-Challenger/trainee/main/setup.sh)
```

* Update

```
vcs import src < trainee.repos --debug
vcs pull src --debug
```