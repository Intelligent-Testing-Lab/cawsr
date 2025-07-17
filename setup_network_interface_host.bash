#!/bin/bash

sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.core.rmem_max=2147483647 
sudo sysctl -w net.ipv4.ipfrag_time=3