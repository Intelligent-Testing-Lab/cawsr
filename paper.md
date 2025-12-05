---
title: 'CAWSR: Carla-Autoware Scenario Runner'
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
date: 03 December 2025
bibliography: paper.bib
---

# Summary

CAWSR facilitates the testing of the open-source autonomous driving system, Autoware, within CARLA, the state-of-the-art open-source driving simulator. Building on existing tools, this project introduces a research-oriented testing framework for the execution of complex driving scenarios, as well as supporting
implementation of a wide range of verification strategies.

# Statement of Need

The aim of this project is to address the technical challenges associated with the verification of state-of-the-art Autonomous Driving Systems (ADS) in virtual environments. Industry-grade systems, such as Autoware [@kato2018autoware] or Apollo [@apollo], are inherently complex, requiring extensive configuration and are typically integrated into specialized simulators like AWSIM [@tier4awsim_2025] or the Apollo Game Engine Based Simulator [@apollow_sim], respectively.

Conversely, CARLA [@carla_sim] has emerged as the de facto standard within the ADS research community.
It holds a prominent position in the autonomous vehicle domain due to its rich ecosystem of open-source tooling. While alternatives such as BeamNG.tech [@beamngtech] and AWSIM have recently gained traction, they are often designed for narrower applications. For instance, BeamNG.tech utilizes a soft-body physics engine to simulate high-fidelity vehicle dynamics, whereas AWSIM is tailored specifically for integrating a single AD system (Autoware).
Despite these alternatives, CARLA remains the preferred choice for end-to-end testing in the research community, due to its support for a wide variety of driving systems, extensive testing benchmarks, and utility tools.

While a basic support for communication between Autoware and CARLA exists, it does not currently form a executable scenario-based testing framework, a leading approach for ADS verification.

This work bridges the gap by enabling the evaluation of Autoware in complex driving scenarios within CARLA.
Adhering to the widely used CARLA platform rather than nascent alternatives, this work introduces a framework built directly upon the established CARLA ecosystem, facilitating direct comparison with the state-of-the-art driving agents evaluated on the CARLA Leaderboard.

The effective ADS verification requires the ability to systematically explore the operational design domain. To support this, the tool provides an extensible interface for algorithmic scenario generation and prioritization, facilitating a wide range of verification strategies based on common evaluation metrics such as the CARLA Leaderboard’s driving score [@carla_leaderboard].

Lastly, maintaining a consistent testing environment is essential, as simulators can introduce unintentional nondeterminism that leads to inconsistent evaluation results [@osikowicz2025empirically]. Therefore, this framework seeks to minimize the introduction of additional nondeterminism into the evaluation pipeline.


# State of the Field

As previously established, CARLA offers a rich ecosystem of tools and documentation.
Multiple communication bridges already exist for supporting popular open-source AD systems, such as *apollo-carla-bridge* [@guardstrikelab_2023_carla] for Apollo and *autoware-carla-bridge* [@carlaautowarebridge] for Autoware.
These tools solve a key issue; transforming sensor and control data from CARLA to formats supported by the each system.
However, native support for scenario execution engines does not exist, restraining their use for scenario-based testing, a dominant approach for end-to-end driving verification [@tang2023survey].

The current standard for ADS evaluation within CARLA is the CARLA Leaderboard and its scenario execution engine, Scenario Runner (SR) [@carla_scenario_runner_2025].
A common use cases of these frameworks include testing end-to-end black-box driving agents, such as those based on VLM (Vision Language Models) or Reinforcement Learning.
<!-- OO: I believe this is wrong: ROS is just a communication layer, it doesn't tell much about the internal architecture of neural networks. I can imange end-to-end driving system that is communicating via ROS  -->
<!-- No support is included for testing ROS based AD systems, which focus on a modular approach to autonomous driving, rather than end-to-end.  -->
PCLA [@tehrani2025pcla], a framework for CARLA, addresses the topic of scenario-based testing by creating a clear deployment pipeline for autonomous agents / systems into CARLA, including Autoware. However, their approach focuses on the simplification of agent implementation and abstraction of setup across various CARLA versions. While this allows for quick use and evaluation without relying on external codebases (such as the CARLA Leaderboard), there is a clear gap on deep integration between the agent and simulator, limiting the execution of complex route-based scenarios.

# Tool Summary

CAWSR (**C**ARLA-**A**uto**w**are-**S**cenario **R**unner) is a fully synchronous testing framework that directly integrates the CARLA simulator, Scenario Runner (as the scenario executor), and Autoware (as the System Under Test) to facilitate autonomous driving testing research. The tool is distributed as a Docker container designed and currently supports two modes of operation:


1. *Algorithm Mode:* Enables the execution of dynamically generated scenarios provided by a user-defined algorithm.
2. *Benchmark Mode:* Allows the execution of a predefined set of scenario definitions provided by the user.

The evaluation pipeline is engineered to be fully synchronous, minimizing unintentional non-determinism to facilitate reproducible results. However, it is noted that minor variations may still persist due to inherent non-determinism in upstream dependencies, such as the driving simulator or the driving agent itself.


![Internal component diagram of CAWSR.](./docs/resources/component_diagram.pdf)

To facilitate development, we introduce a new domain model for the definition of route-based scenarios within CARLA, alongside a `JSON` implementation. This model is based on the format introduced by Scenario Runner, facilitating support between both frameworks.

![Scenario definition domain model.](./docs/resources/scenario_domain.pdf)

# Acknowledgements

This work was supported by the Institute of Information & Communications Technology Planning & Evaluation(IITP) grant funded by the Korea government(MSIT) (No. RS-2025-02218761, 50%) and by the Engineering and Physical Sciences Research Council (EPSRC) [EP/Y014219/1].

# Conclusion

To summarize, CAWSR provides ADS testing research community an easy to use Autoware evaluation pipeline.
We hope that this work can facilitate the evaluation of new testing approaches on a state of the art driving system.

# References
