#!/bin/bash

# Find all processes related to Gazebo
pids=$(pgrep -f gazebo)

# Kill all processes related to Gazebo
if [[ -n "$pids" ]]; then
  echo "Killing Gazebo processes..."
  kill $pids
  echo "Done."
else
  echo "No Gazebo processes found."
fi