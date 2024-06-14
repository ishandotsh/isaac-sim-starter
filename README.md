# isaac-sim-starter

A starter kit / cheatsheet for building simulation environments in Isaac Sim.

### Prerequisites

* ROS 1 Noetic (Robostack works for Ubuntu 22.04/Fedora 40)
* Isaac Sim 2023.1.1 (Probably also works in IsaacSim 4)
* Familiarity with Isaac Sim's documentation (https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)

## Setup

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
git clone https://github.com/ishandotsh/isaac-sim-starter.git
cd isaac-sim-starter

# If you installed Isaac Sim 2023.1.1 in the default location this will work:
# Otherwise change $ISAAC_PATH in setup.sh before running 
./setup.sh
```

## Run

Make sure Omniverse's Nucleus is running or the environments won't load 

```bash
roscore

# in another terminal
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh isaac-sim-starter/isaac.py
```