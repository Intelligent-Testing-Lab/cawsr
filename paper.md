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

Figure 1 shows architecture of `CAWSR` with the fundamental components of the framework. **CarlaClient**, a native CARLA PythonAPI class, establishes a TCP connection to the simulator via a host *IP* and *port*. As the framework’s sole communication link with CARLA, it allows `CAWSR` modules to extract data and spawn entities by interacting with the internal server.

**JSON parser** translates *scenario_definition* into a Behavior Tree (BT) by extracting the route and its associated trigger events. These trees are constructed using Scenario Runner’s `Atomic Behaviours` and `Atomic Conditions`. Serving as the framework's building blocks, these elements represent discrete CARLA actions and variables—such as spawning a pedestrian—to define the scenario's logic.

**ScenarioManager** manages the setup and execution loop, using **CarlaClient** to spawn entities. During each loop, it executes the behavior tree to update actor states and evaluate conditions. It then triggers a simulation tick, advancing CARLA’s internal clock and generating a snapshot. This snapshot is passed to **Agent**, which monitors Autoware’s internal state and route data. Upon initialisation, Agent connects to Autoware via ROS2. Subsequently, at each step, **CarlaBridge** [@carlaautowarebridge] extracts snapshot data, transforms sensor inputs into Autoware’s coordinate system, and publishes them. Finally, Autoware processes this data and issues control commands, which are applied to the ego vehicle.

The internal loop within ScenarioManager continues executing until one of the following termination conditions is met, as defined by the CARLA Leaderboard evaluation criteria [@carla_leaderboard], shown in Table 1.

| Termination Criteria | Description                                       |
|----------------------|---------------------------------------------------|
| Route_Completion     | Agent reached the end of the route.               |
| Actor_Blocked        | Agent is blocked, not moving for 180s.            |
| Simulation_Timeout   | No client-server communication established (30s). |
: Termination Criteria of each scenario within CAWSR.

The same set of standard evalutaion critera is employed to calculate the driving score (DS) for each scenario execution, shown in Table 2.

| Evaluation Criteria                   |
|---------------------------------------|
| Collisions_with_pedestrians           |
| Collisions_with_other_vehicles        |
| Collisions_with_static_elements       |
| Running_a_red_light                   |
| Failure_to_yield_to_emergency_vehicle |
| Running_a_stop_sign                   |
: Evaluation Criteria of each scenario within CAWSR, per the CARLA Leaderboard [@carla_leaderboard]. Each criteria applies a fixed penality to the DS.

![Scenario definition domain model.](./docs/resources/scenario_domain.pdf)

To facilitate development, we introduce a new domain model for the definition of route-based scenarios within CARLA, described in Figure 4, alongside a `JSON` implementation.
This model is based on the format introduced by Scenario Runner, facilitating support between both frameworks.

# Conclusion

To summarise, `CAWSR` provides ADS testing research community an easy to use Autoware evaluation pipeline.
We hope that this work can facilitate the evaluation of new testing approaches on a state of the art driving system.


# Acknowledgements

This work was supported by the Institute of Information & Communications Technology Planning & Evaluation(IITP) grant funded by the Korea government(MSIT) (No. RS-2025-02218761, 50%) and by the Engineering and Physical Sciences Research Council (EPSRC) [EP/Y014219/1].


# References
