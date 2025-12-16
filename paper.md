---
title: 'CAWSR: Carla-AutoWare Scenario Runner'
tags:
    - autonomous vehicles
    - autonomous driving
    - autonomous driving testing
    - autonomous driving agents
    - autonomous driving system
    - testing
    - carla
    - autoware
    - ros
    - scenario
    - scenario based testing
authors:
  - name: David Gasinski
    orcid: 0009-0008-7597-333X
    affiliation: "1"
  - name: Olek Osikowicz
    orcid: 0009-0002-7515-7101
    affiliation: "1"
  - name: Gwilym Rutherford
    orcid: 0009-0007-8820-1091
    affiliation: "1"
  - name: Donghwan Shin
    orcid: 0000-0002-0840-6449
    affiliation: "1"
affiliations:
  - name: The University of Sheffield
    index: 1
date: 00 December 2025
bibliography: paper.bib
---

# Summary

`CAWSR` (**C**ARLA-**A**uto**W**are-**S**cenario **R**unner) facilitates the simulation-based testing of the open-source autonomous driving system, Autoware, within CARLA, the state-of-the-art open-source driving simulator. Building on existing tools, this project introduces a research-oriented testing framework for the execution of complex driving scenarios, as well as supporting implementation of a wide range of verification strategies.

# Statement of Need

Verifying Autonomous Driving Systems (ADS) is a critical step before they can be deployed.
However, relying only on real-world testing is too expensive, inefficient, and potentially dangerous.
Consequently, simulation-based testing has become essential, allowing researchers to safely test driving agents against critical situations at scale.
Among these tools, CARLA [@carla_sim] has become the de-facto standard in the research community due to its rich ecosystem of open-source tools, benchmarks, and documentation.

Currently, the standard for evaluating ADS in CARLA is the CARLA Leaderboard and its engine, Scenario Runner (SR) [@carla_scenario_runner_2025].
This framework is typically used to test "black-box" driving agents, such as those based on Vision Language Models or Reinforcement Learning (DS: add refs here).
By running a set of predefined, challenging driving scenarios, researchers can systematically assess agent performance using common metrics like driving score, infractions, and route completion.
However, applying this testing framework to industry-grade ADS, such as Autoware [@kato2018autoware] or Apollo [@apollo], remains difficult.
Although communication bridges exist between CARLA and these systems [@guardstrikelab_2023_carla; @carlaautowarebridge], they lack native support for scenario execution engines, which limits their utility for scenario-based testing.

This gap has created a significant bottleneck for the research community.
Previously, researchers developing scenario generation algorithms mainly relied on combining Apollo with the LGSVL simulator [@9294422].
However, LGSVL is now outdated, with official support ending in January 2022.
This leaves many researchers without a suitable industry-grade "subject" for evaluating their algorithms.
While recent tools like PCLA [@tehrani2025pcla] attempt to simplify deploying Autoware (and other ADS implementations) into CARLA, they focus primarily on simplifying the ADS implementations and abstracting the setup process across different CARLA versions.
They lack the deep integration required between the agent and simulator to execute complex, route-based scenarios.

`CAWSR` aims to bridge this gap by enabling the evaluation of Autoware in complex driving scenarios within CARLA.
By building on the established CARLA platform, this work provides a modern replacement for the outdated Apollo/LGSVL workflow.
It also allows Autoware to be directly compared with state-of-the-art research agents on the CARLA Leaderboard.

Effective ADS verification requires the ability to systematically explore the operational design domain.
To support this, `CAWSR` provides a flexible interface for algorithmic scenario generation.
This facilitates a wide range of verification strategies based on common metrics, such as the CARLA Leaderboard’s driving score [@carla_leaderboard].

Lastly, it is worth noting that simulators can often introduce unintended nondeterminism, which leads to inconsistent test results [@9793395; @osikowicz2025empirically].
Therefore, `CAWSR` is designed to minimise such nondeterminism throughout the evaluation pipeline.


# Tool Overview

`CAWSR` is a fully synchronous testing framework that directly integrates the CARLA simulator, Scenario Runner (as the scenario executor), and Autoware (as the System Under Test) to facilitate autonomous driving testing research. The tool is distributed as a Docker container designed and currently supports two modes of operation:

1. *Scenario Generation Mode:* Enables the dynamic generation and execution of scenarios (e.g. iterative scenario generation) provided by a user-defined algorithm. This is particularly useful for assessing the performance of new simulation-based ADS testing techniques.
2. *Benchmark Mode:* Allows the execution of a predefined set of scenario definitions provided by the user. This is useful for standardised evaluations and comparisons between different driving agents.

The evaluation pipeline is engineered to be fully synchronous, minimising unintentional non-determinism to facilitate reproducible results. However, it is noted that minor variations may still persist due to inherent non-determinism in upstream dependencies, such as the driving simulator or the driving agent itself [@9793395; @osikowicz2025empirically].

![Internal component diagram of CAWSR.](./docs/resources/component_diagram.pdf)

Figure 1 shows the overall architecture of `CAWSR`.

(**David**: This is the first approach, although i feels a bit short and doesn't elaborate much on each component and how they interact together.)

| Component                                            | Function                                                                                                                                                                                                                     |
|------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CarlaClient                                          | A native class of the CARLA PythonAPI, facilitating interaction with the simulator.                                                                                                                                          |
| JSON parser                                          | A module which converts the *scenario_defintion* into XML behaviour trees, used for scenrario execution by the ScenarioManager.                                                                                              |
| ScenarioManager                                      | Scenario Runner module which handles the execution of scenarios through behaviour trees.                                                                                                                                     |
| CarlaBridge                                          | This module handles the transformation of sensor data from CARLA to ROS. At each step, data is sent to Autoware, responding with a control command that is applied to the ego vehicle in simulation.                         |
| Agent                                                | Monitors the internal state of key Autoware modules, handling the communication between CAWSR and Autoware. Responsible for sending over route information during initialisation and executing the CarlaBridge at each step. |
: Function of the key modules in CAWSR.

(**David**: Second approach, more text and less readability than the first, but offers significanlty more detail about how the framework functions and how the modules interact.)

Each component of CAWSR plays a fundamental role in the overall architecture of the framework. The **CarlaClient** is a native class of the CARLA PythonAPI. Accepting a *host* IP and *port*, it creates a TCP connection to the simulator and allows interaction with its interval server, enabling CAWSR modules to extract information and spawn entities. This forms the basis of CAWSR and the single point of communication between the framework and CARLA.

The **JSON parser** converts the JSON *scenario_definition* into a behaviour tree (BT), extracting information about the *route* and events that trigger along it. The fundamental elements of the behaviour trees are assembled from *Atomic Behaviours* and *Atomic Conditions*, introduced by Scenario Runner. These represent individual behaviours and variables within CARLA, such as spawning a pedestrian, serving as the building blocks of scenarios.

**ScenarioManager** handles the initial setup and execution loop, using the CarlaClient to spawn the relevant scenario entities. At each step, the scenario behaviour tree is executed, calculating updated states for each actor and triggering relevant conditions. Through the CarlaClient, a single *tick* (step) is sent to CARLA, incrementing its internal clock and generating a new snapshot of the simulation. This snapshot is sent to the **Agent** module, monitoring the internal state of Autoware modules and sending information about the route. During initialisation, the agent instantiates a connection with Autoware through ROS. Once this connection is established, at each execution step, the **CarlaBridge** extracts data through the snapshot. Sensor data is transformed into Autoware's coordinate system and published to the relevant modules. Once processed, Autoware updates its internal state and responds with a control command, applying it to the ego vehicle in simulation.

The internal loop within ScenarioManager continues executing until one of the following termination conditions is met, as defined by the CARLA Leaderboard evaluation criteria [@carla_leaderboard], as shown in Table 2.

| Criteria           | Description                                                |
|--------------------|------------------------------------------------------------|
| Route_Completion   | Agent reached the end of the route.                        |
| Actor_Blocked      | Agent is blocked, not moving for 180s.                     |
| Simulation_Timeout | No client-server communication can be established for 30s. |
: Termination Criteria of each scenario within CAWSR.

(**David**: If we go with the second approach, is it worth adding a paragraph on how we calculate the driving score for use with the algorithm mode? It follows the same principles as the termination criteria, also based on the CARLA leaderboard)


![Scenario definition domain model.](./docs/resources/scenario_domain.pdf)

To facilitate development, we introduce a new domain model for the definition of route-based scenarios within CARLA, described in Figure 3, alongside a `JSON` implementation.
This model is based on the format introduced by Scenario Runner, facilitating support between both frameworks.

# Conclusion

To summarise, `CAWSR` provides ADS testing research community an easy to use Autoware evaluation pipeline.
We hope that this work can facilitate the evaluation of new testing approaches on a state of the art driving system.


# Acknowledgements

This work was supported by the Institute of Information & Communications Technology Planning & Evaluation(IITP) grant funded by the Korea government(MSIT) (No. RS-2025-02218761, 50%) and by the Engineering and Physical Sciences Research Council (EPSRC) [EP/Y014219/1].


# References
