# trainee

## How to set up workspace

* Install

#### PC
```
source <(curl -s https://raw.githubusercontent.com/Shinsotsu-Tsukuba-Challenger/trainee/main/setup.sh) pc
```

#### Raspberry Pi
```
source <(curl -s https://raw.githubusercontent.com/Shinsotsu-Tsukuba-Challenger/trainee/main/setup.sh) raspi
```

* Update

```
vcs import src < trainee.repos --debug
vcs pull src --debug
```