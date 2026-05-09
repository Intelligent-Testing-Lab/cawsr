#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Wrapper for autonomous agents required for tracking and checking of used sensors
"""

from __future__ import print_function


class AgentWrapper(object):
    """
    Wrapper for autonomous agents required for tracking and checking of used sensors
    """

    _agent = None
    _sensors_list = []

    def __init__(self, agent):
        """
        Set the autonomous agent
        """
        self._agent = agent

    def __call__(self, timestamp=None):
        """
        Pass the call directly to the agent.
        If timestamp is provided, forward it to propagate the CARLA
        snapshot through the chain so the agent does not re-acquire it.
        """
        return self._agent(timestamp)

    def setup_sensors(self, vehicle, debug_mode=False):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param vehicle: ego vehicle
        :return:
        """
        return

    def cleanup(self):
        """
        Call agent cleanup method
        """
        # call agent cleanup
        self._agent.cleanup()
