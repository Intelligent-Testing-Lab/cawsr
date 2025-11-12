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

Using the ScenarioRunner
------------------------

Please take a look at our [Getting started](Docs/getting_scenariorunner.md)
documentation.


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
