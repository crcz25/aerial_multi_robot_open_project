#!/bin/bash

echo $'\nLoading ROS Environment...\n'

# Load the current directory
curr_dir=$(pwd)
echo $"Current Directory: $curr_dir"

# Find the Tello ROS Package to source
setup=$(find "${curr_dir}/install" -type f -name "setup.bash" | grep -E setup.bash$)

# Verify if the setup file was found
if [[ -z "$setup" ]]; then
  echo $"\nCould not find setup.bash file under the install directory."
  exit 1
fi

# Print the setup file
echo $'\nSetup File:'
echo "$setup"

# Find the Tello model directory
model_dir=$(find "${curr_dir}/install" -type d -name "models" | grep -E models$)

# Verify if the model directory was found
if [[ -z "$model_dir" ]]; then
  echo $'\nCould not find tello_gazebo model directory.'
  exit 1
fi

# Print the models directory (only the first one)
echo $'\nModel Directory:'
echo "$model_dir"

# Source the Tello ROS Package
source $setup

# Add the path models to the GAZEBO_MODEL_PATH variable (if it exists and is not already there)
if [[ -n "$GAZEBO_MODEL_PATH" ]]; then
  if [[ ! "$GAZEBO_MODEL_PATH" == *"$model_dir"* ]]; then
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$model_dir
  fi
else
  export GAZEBO_MODEL_PATH=$model_dir
fi

# Set the turtlebot3 model
export TURTLEBOT3_MODEL=waffle_pi

# Print the model type
echo $'\nTurtlebot3 Model: '$TURTLEBOT3_MODEL

# Source Gazebo
gazebo_setup=/usr/share/gazebo/setup.sh
source $gazebo_setup

# Print Gazebo setup file
echo $'\nGazebo Setup File:'
echo "$gazebo_setup"

# Source ROS
ros_setup=/opt/ros/galactic/setup.bash
source $ros_setup

# Print ROS setup file
echo $'\nROS Setup File:'
echo "$ros_setup"

# Generate a random number between 1 and 99
rand_num=$((1 + RANDOM % 99))
# Only change the ROS_DOMAIN_ID if it is not already set
if [[ -z "$ROS_DOMAIN_ID" ]]; then
  export ROS_DOMAIN_ID=$rand_num
fi

# Print the ROS_DOMAIN_ID
echo $'\nexport ROS_DOMAIN_ID='$ROS_DOMAIN_ID