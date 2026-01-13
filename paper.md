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
This framework is typically used to test "black-box" driving agents, such as ML-based systems which expose only sensor-level inputs and driving control outputs.
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

# Research Impact

CAWSR is the first verification tool of its kind to bridge the gap between CARLA and Autoware, intended as a modern replacement for the Apollo/LGSVL workflow. Designed for ease of use and reproducibility, it enables researchers to build and evaluate scenario generation algorithms on a state-of-the-art simulator platform with a modern, industry-grade ADS system.

A primary contribution of this tool is the unification of evaluation standards. Industry-grade AD systems operate in disparate evaluation environments when compared to academic agents, which introduce unintended nondeterminism[@9793395; @osikowicz2025empirically]. Built on the unified CARLA platform, CAWSR enables the execution of modular industry systems, facilitating direct comparison to academic agents (end-to-end models) under identical conditions.

Furthermore, CAWSR offers a flexible programmatic interface for search-based testing and verification of Autoware through scenario generation algorithms. This enables researchers to design verification strategies based on various metrics extracted from scenario execution, such as the CARLA Leaderboard's driving score.

CAWSR prioritises a streamlined deployment process for ease of use and reproducibility. By leveraging containerised deployment through Docker, the framework removes complex dependencies associated with Autoware and CARLA, simplifying the complexity of setup drastically. To support community adoption, a comprehensive set of tools and example implementations is provided, supplying the foundations for the development of new scenarios and verification strategies.


# Software Design
<!-- I've renamed the "Tool Overview" section since it fits quite well here, and added a few justifications.-->

`CAWSR` is a fully synchronous testing framework that directly integrates the CARLA simulator, Scenario Runner (as the scenario executor), and Autoware (as the System Under Test) to facilitate autonomous driving testing research. The tool is distributed as a containerized deployment using Docker to manage complex dependencies and simplify the setup process. Currently two modes of operation are supported:

1. *Scenario Generation Mode:* Enables the dynamic generation and execution of scenarios (e.g. iterative scenario generation) provided by a user-defined algorithm. This is particularly useful for assessing the performance of new simulation-based ADS testing techniques.
2. *Benchmark Mode:* Allows the execution of a predefined set of scenario definitions provided by the user. This is useful for standardised evaluations and comparisons between different driving agents.

The evaluation pipeline is engineered to be fully synchronous, minimising unintentional non-determinism to facilitate reproducible results. However, it is noted that minor variations may still persist due to inherent non-determinism in upstream dependencies, such as the driving simulator or the driving agent itself [@9793395; @osikowicz2025empirically].

![Internal component diagram of CAWSR.\label{fig:components}](./docs/resources/component_diagram.pdf)

\autoref{fig:components} illustrates the `CAWSR` architecture and its fundamental components. The framework operates through four primary modules:

- CarlaClient: A native CARLA PythonAPI class that establishes a TCP connection (via host IP and port). It serves as the framework's exclusive interface for extracting simulation data and spawning entities.

- JSON Parser: Translates the *scenario_definition* (see \autoref{fig:scenario_domain}) into a Behavior Tree (BT). It utilises Scenario Runner's *Atomic Behaviours* and *Atomic Conditions* as modular primitives to define discrete actions (e.g., spawning pedestrians) and logic triggers.

<!-- David: Citing SR for added transparency, since this class is natively part of it -->
- ScenarioManager[@carla_scenario_runner_2025]: Orchestrates the simulation loop by evaluating the BT to update actor states and triggering CARLA simulation ticks. Execution terminates based on CARLA Leaderboard criteria [@carla_leaderboard], as summarised in \autoref{tab:termination_criteria}. Post-execution, the module calculates the Driving Score (DS) according to the official leaderboard metrics.

- Agent and CarlaBridge: The Agent manages the ROS2 connection to Autoware. At each timestep, the CarlaBridge [@carlaautowarebridge] transforms CARLA snapshots and sensor data into the Autoware coordinate system. Autoware processes these inputs to issue control commands, which the Agent then applies to the ego vehicle.

| Termination Criteria | Description                                       |
|----------------------|---------------------------------------------------|
| Route_Completion     | Agent reached the end of the route.               |
| Actor_Blocked        | Agent is blocked, not moving for 180s.            |
| Simulation_Timeout   | No client-server communication established (30s). |
: Termination Criteria of each scenario within CAWSR.\label{tab:termination_criteria}

To facilitate development, we introduce a new domain model for the definition of route-based scenarios within CARLA, described in \autoref{fig:scenario_domain}, alongside a `JSON` implementation.
This model is based on the format introduced by Scenario Runner, facilitating support between both frameworks.

![Scenario definition domain model.\label{fig:scenario_domain}](./docs/resources/scenario_domain.pdf)

# Conclusion

To summarise, `CAWSR` provides ADS testing research community an easy to use Autoware evaluation pipeline.
We hope that this work can facilitate the evaluation of new testing approaches on a state of the art driving system.

# AI Usage Disclosure

<!-- David: I used AI to help explain concepts some concepts and generate short code snippets to get me started (mostly about ROS2 and networking). Is this the correct way to phrase it? -->
Generative AI tools were used in this work solely to support high-level research concepts and structural ideas. All software implementation, including the source code, architecture, and deployment scripts, was authored entirely by the researchers without AI assistance.

# Acknowledgements

This work was supported by the Institute of Information & Communications Technology Planning & Evaluation(IITP) grant funded by the Korea government(MSIT) (No. RS-2025-02218761, 50%) and by the Engineering and Physical Sciences Research Council (EPSRC) [EP/Y014219/1].


# References
