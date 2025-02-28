#!/bin/bash
set -e

ROOT=$(dirname "$(dirname "$(readlink -f $0)")")

# Store command history in the workspace which is persistent across rebuilds
echo "export HISTFILE=${ROOT}/.zsh_history" >> ~/.zshrc

# Detect installed ROS version and source the appropriate setup file
if [ -d "/opt/ros/humble" ]; then
    ROS_DISTRO="humble"
elif [ -d "/opt/ros/jazzy" ]; then
    ROS_DISTRO="jazzy"
else
    # Find any ROS installation
    for dir in /opt/ros/*; do
        if [ -d "$dir" ]; then
            ROS_DISTRO=$(basename "$dir")
            break
        fi
    done
    
    if [ -z "$ROS_DISTRO" ]; then
        echo "No ROS distribution found in /opt/ros/"
        exit 1
    fi
fi

echo "Found ROS distribution: $ROS_DISTRO"

# Setup the ROS environment in shells
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.zsh" >> ~/.zshrc

# Setup fzf completions
echo "eval \"\$(fzf --bash)\"" >> ~/.bashrc
echo "source <(fzf --zsh)" >> ~/.zshrc