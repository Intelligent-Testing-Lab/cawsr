#!/bin/bash
set -e

source "${ROS_PKG}"
source "${AUTOWARE_MSG_PKG}"

exec "$@"