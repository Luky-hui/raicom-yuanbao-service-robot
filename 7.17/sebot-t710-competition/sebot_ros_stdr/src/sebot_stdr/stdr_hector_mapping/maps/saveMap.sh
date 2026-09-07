#!/bin/bash
# Description: bash file to save stdr_hector_mapping's map

rosrun map_server map_saver map:=/hector_map -f hector_map
exit 0

