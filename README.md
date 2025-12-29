[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

CAWSR: ScenarioRunner for CARLA with support for Autoware
========================

CAWSR (Carla Autoware Scenario Runner) is a scenario execution engine built for the testing of [Autoware](https://autoware.org/autoware-overview/) in route-based scenarios.

Prerequisites
---------------------------
Both CARLA and Autoware require a high-spec computer with a high-end Nvidia GPU. It is also possible to run a [**distributed**]() setup with multiple machines to help ease the workload, or run the entire stack locally. Currently, only Linux is supported (guide was written on Ubuntu 24.04).

Ensure the target machine(s) have the [Docker Engine]() and [Nvidia Container toolkit]() installed to enable gpu accelerated workflows in Docker.

CAWSR Setup
---------------------------
Setup access to the [**Github container registry**](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)

Once you have been granted access, pull the following images:
```
docker pull carlasim/carla:0.9.15
docker pull ghcr.io/intelligent-testing-lab/autoware-scenario-runner:latest
docker pull ghcr.io/intelligent-testing-lab/autoware:latest
```

Autoware and ROS use a custom messaging interface for communication, known as DDS. For maximum performance, configure your network settings as follows. If not configured, you will see [heavy performance issues](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) as the default ubuntu buffer sizes fill up fast, especially when running over lossy networks such as WiFi.
```bash
# Increase the maximum receive and send buffer size for network packets, allowing our containers to communicate
sudo sysctl -w net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB
sudo sysctl -w net.core.wmem_max=2147483647

# IP fragmentation settings
sudo sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```
Save the following commands in `setup.sh`, allow it to be executable `chmod +x setup.sh`.
These settings are **temporary** and will revert on restart.

To allow GUI applications (like Autoware and CARLA) to run through Docker, you must allow xhost connections from the `docker` group.
```bash
xhost +local:docker
```

Running CAWSR
------------------------

CAWSR can both be ran `locally` or `distributed`. Due to the high-spec requirements, it is recommended to run distributed if you do not meet the following minimum specs:
- At least **10GB** VRAM and a modern GPU (2080 ti or newer)
- At least **32GB** RAM
- A modern Intel or AMD CPU with at least 8 cores.

### Locally

To run locally, set *MODE* in `.env`
```env
[Network]
MODE=local
```

Ensure multicast is enabled for the *localhost* interface:
```bash
sudo ip link show lo
1: lo: <LOOPBACK,UP,LOWER_UP, MULTICAST>
```
If **MUTLICAST** is not present, you can enable it with `sudo ip link set lo multicast on`.

Run the entire stack
```bash
docker compose up
```

### Distributed

When running distributed, we use *unicast* to enable compatibility with all networks. This requires some extra configuration.

Running CAWSR distributed using the following setup:
- **Machine A**: Carla and CAWSR
- **Machine B**: Autoware

Configure the `.env` and ensure is it the same across both machines
```
[Network]
MODE=local # or distributed (caps sensitive)
ROS_DOMAIN_ID=0

# For distributed mode
HOST_IP=127.0.0.1 # CAWSR and CARLA
AUTOWARE_IP=127.0.0.1 # Autoware
```
The `ROS_DOMAIN_ID` *must* match, otherwise the ROS2 nodes will not be able to find each other. Once configured, start CAWSR and Carla on Machine A
```
docker compose up carla cawsr
```
and Autoware on Machine B
```
docker compose up autoware-latest
```

If you are experiencing problems, you can trouble shoot with the following command, replacing `eth0` and `192.168.1.20` with your network interface and destination IP:
```bash
ip addr show # find the network interface used
sudo tcpdump -i eth0 udp and src 192.168.1.20
```
This will tell you if packets are flowing between Machine A and Machine B. If no packets are flowing, it is likely an issue with your network configuration.

Using CAWSR
------------------------
After completing the prerequisite steps, clone the [CAWSR workspace](https://github.com/Intelligent-Testing-Lab/cawsr_workspace) repository.
The structure of the workspace is as follows.
```
scenarios/ -> this folder holds all the scenario configurations
configs/ -> this folder holds all user config files
results/ -> results from runs are stored here
algorithms/ -> holds all custom algorithm scripts
docker_compose.yml
.env
```
All folders are mounted as Docker volumes into the CAWSR container, so any changes persist between host and container.

## Configuring CAWSR

CAWSR is designed to be highly configurable and supports easy swapping of config files.
1. Create a config file in `configs/` based on one of the examples.
2. Modify the `CAWSR_CONFIG` environmental variable in `.env` to point towards the selected file. **All files use relative paths from the CAWSR root directory**.

## Execution Mode

In CAWSR, there are two modes you can configure `algorithm` or `benchmark`. To set the mode, modify the **mode** variable in `config.yaml`.

### Algorithm

This mode enables the use of a custom algorithm to modify / optimize the scenario definition after execution.
```yaml
algorithm:
    initial_definition: scenarios/examples/example_scenario.json # can be null
    seed: 10
    runs: 50
    path: algorithms/random_search
    args:
      lanelet2: algorithms/resources/Town01.osm
```

Included in `algorithms/basic_algorithm.py` is the BasicAlgorithm class, from which all algorithms inherit. The algorithm is ran on every
iteration of the scenario, modifying the definition based on the result of the previous scenario. At beginning of every iteration, the method
```python
 def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
```
is called.

#### Implementing a custom algorithm
Create a class than inherits `BasicAlgorithm` and implements the function `scenario_callback`. The function must accept the current scenario_definition and the driving score, returning a new scenario definition. To use outside resources, such as loading a lanelet file (see example config), pass them in via the args config variable. This gets converted into a python dictionary and passed to the algorithm class when initialised. Algorithms are run synchronously, so CAWSR will wait for completion.

The algorithm will execute **runs** times.

### Benchmark

Benchmark simply executes all scenario definitions in a given directory. Set `scenarios` to a path containing `.json` scenario definitions, and enable / disable random sampling. If enabled, CAWSR will executed each scenario once in a random order.


Scenario Definition
-------------------

We use a custom implementation of a scenario definition in JSON. We have included a scenario domain model, as well as plenty of examples in the CAWSR Workspace repository `scenarios/examples/`.

Domain Model:
![Domain Model](./docs/resources/scenario_domain.png)

Notes
------------
Currently, traffic light recognition is disabled due to an issue with the [CARLA map format](https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface#traffic-light-recognition). The updated LaneLet files (as well as the Autoware images) will be published accordingly once development has finished.

Contributing
------------

Please take a look at our [Contribution guidelines]().

License
-------
### 1. CAWSR
Core CAWSR logic and Autoware integration.
* **Copyright:** © 2025 University of Sheffield
* **License File:** [`LICENSE Sheffield`](./LICENSE%20Sheffield)

### 2. Scenario Runner (CARLA) (MIT)
Scenario execution engine for CARLA.
* **Copyright:** © Intel Corporation / CARLA Team
* **License File:** [`LICENSE Carla`](./LICENSE%20Carla)

### 3. Autoware Carla Interface (Apache 2.0)
Autoware communication bridge.
**Apache License 2.0**.
* **Copyright:** © The Autoware Foundation / Tier IV, Inc. / AutoCore / Leo Drive
* **License File:** [`LICENSE-APACHE`](./LICENSE-APACHE)

**Notices:** See [`NOTICE`](./NOTICE) for the full list of required attributions.
