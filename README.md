[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/carla-simulator/scenario_runner.svg)

CAWSR: ScenarioRunner for CARLA with support for Autoware
========================
This repository contains scenario definition and an execution engine
for CARLA. Support has been added to run route-based scenarios with the ego being controlled by [Autoware](https://autoware.org/autoware-overview/)

Prerequisites
---------------------------
Both CARLA and Autoware require a high-spec computer with a high-end Nvidia GPU. It is also possible to run a [**distributed**]() setup with multiple machines to help ease the workload. Currently, only Linux is supported (guide was written on Ubuntu 24.04).

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

Autoware and ROS use a custom messaging interface for communcation, known as DDS. They support various implementations, but they all rely on specific network settings to enable maximum data transfer. Save the following command in `setup.sh`, allow it to be executable `chmod +x setup.sh` and run.
```bash
# Increase the maximum receive buffer size for network packets
sudo sysctl -w net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
sudo sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```
These settings are **temporary** and will revert on restart.

To allow GUI applications (like Autoware and CARLA) to run through Docker, you must allow xhost connections from the `docker` group.
```bash
xhost +local:docker
```

Using CAWSR
------------------------

After completiting the prerequisite steps, clone the CAWSR workspace repository. To launch CAWSR, navigate to the CAWSR workspace and run `docker compose up`.

The structure of the workspace is as follows.
```
scenarios/ -> this folder holds all the scenario configurations
configs/ -> this folder holds all user config files
results/ -> results from runs are stored here
algorithms/ -> holds all custom algorithm scripts
```
All folders are mounted as Docker volumes into the CAWSR container, so any changes persist between host and container.

In CAWSR, there are two modes you can configure `algorithm` or `benchmark`. To set the mode, modify `mode: 'benchmark' # benchmark or algorithm` in a `config.yaml` file. You can create multiple configuration files in `configs/`. To use a specific config, modify the **CAWSR_CONFIG** ENV variable in the `docker-compose.yaml`, pointing it to the path of your config file. **All files use relative paths from the CAWSR root directory**.

**Algorithm**
Algorithm config:
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
iteration of the scenario, modifying the defintion based on the result of the previous scenario. At beginning of every iteration, the method
```python
 def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
```
is called. To implement a custom algorithm, create a class than inherits from `BasicAlgorithm` and implements the function `scenario_callback`. The function must follow the signature above, returning a new scenario definition. To use outside resources, such as loading a lanelet file (see example config), pass them in via the args config variable. This gets converted into a python dictionary and passed to the algorithm class when initialised. Algorithms are run sync, so CAWSR will wait for completion.

The algorithm will execute **runs** times.


Scenario Definition
-------------------

We use a custom implementation of a scenario definition in JSON. We have included a scenario domain model, as well as plenty of examples in the CAWSR Workspace repository `scenarios/examples/`.

Domain Model:
![Domain Model](./docs/resources/scenario_domain.png)

Contributing
------------

Please take a look at our [Contribution guidelines](https://carla.readthedocs.io/en/latest/#contributing).

FAQ
------

If you run into problems, check our
[FAQ](http://carla.readthedocs.io/en/latest/faq/).

License
-------

ScenarioRunner specific code is distributed under MIT License.
