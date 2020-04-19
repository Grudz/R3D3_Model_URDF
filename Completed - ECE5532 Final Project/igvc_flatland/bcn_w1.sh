#!/bin/bash

echo "hello world"

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped'{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 21.0, y: 15.0, z: 0.0}, orientation: {w: 1.0}}}
