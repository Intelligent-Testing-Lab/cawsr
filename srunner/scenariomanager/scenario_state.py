from __future__ import annotations
from typing import TYPE_CHECKING, Optional
from enum import Enum
from threading import Lock

if TYPE_CHECKING:
    from srunner.autoagents.autoware_nodes.state_node import StateNode


class ScenarioState(Enum):
    """Enum representing the state of scenario execution in CAWSR."""

    NOT_STARTED = 0
    INITIALISING = 1
    RUNNING = 2
    FINISHED = 3


class CAWSRState:
    """
    Class that holds the global state of CAWSR scenario execution
    """

    CURRENT_STATE: ScenarioState = ScenarioState.NOT_STARTED
    PREVIOUS_STATE: ScenarioState = ScenarioState.NOT_STARTED

    _publisher_node: Optional[StateNode] = None
    _state_lock = Lock()

    @classmethod
    def set_publisher_node(cls, publisher_node: Optional[StateNode]) -> None:
        with cls._state_lock:
            cls._publisher_node = publisher_node

    @classmethod
    def set_state(cls, state: ScenarioState) -> None:
        with cls._state_lock:
            cls.PREVIOUS_STATE = cls.CURRENT_STATE
            cls.CURRENT_STATE = state
            publisher_node = cls._publisher_node
            current_state = cls.CURRENT_STATE

        if publisher_node:
            publisher_node.publish_cawsr_state(current_state)

    @classmethod
    def get_state(cls) -> ScenarioState:
        with cls._state_lock:
            return cls.CURRENT_STATE

    @classmethod
    def get_previous_state(cls) -> ScenarioState:
        with cls._state_lock:
            return cls.PREVIOUS_STATE
