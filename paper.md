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
    ror: needs filling
date: 03 December 2025
bibliography: paper.bib
---

# Summary

CAWSR facilities the testing of the open-source autonomous driving system, Autoware, within CARLA [@carla_sim], the state-of-the-art open-source driving simulator. Building on existing tools, this project introduces a research-oriented testing framework for the execution of complex driving scenarios, as well as supporting implementation of a wide range of verification strategies.

# Statement of Need

The aim of this project is to address the technical challenges associated with verification of complex autonomous driving systems (ADS) such as Autoware [@kato2018autoware] or Baidu's Apollo [@ap].
<!-- particularly when compared to end-to-end models like Transfuser++ [@Jaeger2023ICCV].  -->
CARLA is the choice of the simulator for this tool due its rich ecosystem of open-source tooling and its prominence in the domain of autonomous vehicles. Alternatives such as BeamNG [@beamngtech] or AWSIM [@tier4awsim_2025] have recently been rising in popularity, specifically designed for natively integrating AD systems such as Autoware. However CARLA still remains the standard choice in end-to-end testing of autonomous vehicles in the autonomous driving testing research community.

In practice, maintaining a consistent testing environment is essential, as simulators can introduce unintentional nondeterminism leading to inconsistent evaluation results [@osikowicz2025empirically]. While support for Autoware within the CARLA simulator exists, it does not currently extend to scenario-based testing, a critical component in established frameworks like the CARLA Leaderboard [@carla_leaderboard] and its execution engine, Scenario Runner.

By adhering to the widely used CARLA platform rather than nascent alternatives, this paper introduces a framework directly built on established projects within the CARLA community, facilitating direct comparison with state-of-the-art driving agents evaluated on the CARLA Leaderboard, and ensuring that no additional non-determinism is introduced into the evaluation pipeline. The tool provides an extensible interface for algorithmic scenario generation and optimisation, supporting a wide range of verification strategies based on common evaluation metrics such as CARLA Leaderboard's driving score.

Despite the increasing use of Autoware, there is still exists a clear lack of tooling for the testing and verification of the system in complex road scenarios. CAWSR aims to address this, enabling researchers to test Autoware in programmatically defined road scenarios, evaluating its safety and capability as an ADS.

<!-- OO: Finished here -->
# State of the Field

As previously established, CARLA offers a rich ecosystem of tools and documentation. Multiple frameworks alreadty exist for supporting popular open-source AD systems, such as *apollo-carla-bridge* [@guardstrikelab_2023_carla] for Apollo and *autoware-carla-bridge* [@carlaautowarebridge] for Autoware. These tools solve a key issue; transforming sensor and control data from CARLA to formats supported by the each system. However, native support for scenario execution engines is non-existent, restraining their use for scenario-based testing and verification. These techniques significantly reduce the effort required to validate autonomous driving systems, lowering the technical barrier of entry compared to alternative approaches such as mileage-based testing, which require high startup costs.

The current standard for scenario execution within CARLA is the CARLA Leaderboard and its execution engine, Scenario Runner (SR) [@carla_scenario_runner_2025]. Common use cases of these frameworks include testing end-to-end driving models, such as those based on VLM (Vision Language Models) or Reinforcement Learning. No support is included for testing ROS based AD systems, which focus on a modular approach to autonomous driving, rather than end-to-end. PCLA [@tehrani2025pcla], a framework for CARLA, addresses the topic of scenario-testing by creating a clear deployment pipeline for autonomous agents / systems into CARLA, including Autoware. However, their methodology focuses on the simplification of agent implementation and abstraction of setup across various CARLA versions. While this allows for quick use and evaluation without relying on external codebases (such as the CARLA Leaderboard), there is a clear gap on deep intergration between the agent and simulator, limiting the execution of complex route-based scenarios.

# Tool Summary

CAWSR is a fully synchronous testing-framework directly integrating Scenario Runner. It is built and deployed as a Docker container alongside Autoware and includes two main modes of operation. *Algorithm* mode supports the execution of custom verification strategies and algorithms implemented by the user. *Benchmark* mode includes functionality to execute and evaluate a set of scenario definitions empirically. A fully synchronous pipeline has been developed to ensure no new non-determinism is introduced, although some may exist directly within the simulator itself [@osikowicz2025empirically].
![Internal component diagram of CAWSR.](./docs/resources/component_diagram.pdf)

To facilitate development, we introduce a new domain model for the definition of route-based scenarios within CARLA, alongside a JSON implementation. This model is based on the format introduced by Scenario Runner, facilitating support between both frameworks.
![Scenario definition domain model.](./docs/resources/scenario_domain.pdf)

# Acknowledgements

This work was supported by the Institute of Information & Communications Technology Planning & Evaluation(IITP) grant funded by the Korea government(MSIT) (No. RS-2025-02218761, 50%) and by the Engineering and Physical Sciences Research Council (EPSRC) [EP/Y014219/1].

# References
