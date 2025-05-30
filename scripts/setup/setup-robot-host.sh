#!/bin/bash

# Exit on error
set -e

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then 
    echo "Please run this script with sudo, e.g. sudo ./scripts/setup/setup-robot-host.sh"
    exit 1
fi

# Set the target user
USER="robotx"
USER_HOME="/home/${USER}"

echo "Setting up robot host system..."

# Get the workspace root directory
ROOT=$(dirname $(dirname $(dirname $(readlink -f $0))))
echo "Workspace root directory: ${ROOT}"

# Make all setup scripts executable
echo "Making setup scripts executable..."
chmod +x ${ROOT}/scripts/setup/*.sh

# Update package lists
apt-get update

# Install essential system utilities
echo "Installing essential system utilities..."
apt-get install -yq --no-install-recommends \
    sudo \
    gh \
    git \
    git-lfs \
    curl \
    wget \
    ca-certificates \
    gnupg \
    lsb-release \
    htop \
    glances \
    net-tools \
    python3 \
    python3-pip \
    python-is-python3 \
    software-properties-common

# Add universe repository
add-apt-repository universe

# Install development tools
echo "Installing development tools..."
apt-get install -yq --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    bash-completion \
    nano \
    vim \
    tmux \
    xclip \
    tmuxinator \
    tree \
    inetutils-ping \
    x11-apps \
    dbus-x11

# Install code formatting and linting tools
echo "Installing code formatting tools..."
apt-get install -yq --no-install-recommends \
    clang-format

# Install gitman and pre-commit
echo "Installing gitman and pre-commit..."
if grep -q "24.04" /etc/os-release; then
    pip install --no-cache-dir --break-system-packages gitman pre-commit
else
    pip install --no-cache-dir gitman pre-commit
fi

# Run additional setup scripts
echo "Running additional setup scripts..."

# Install fzf
echo "Setting up fzf..."
${ROOT}/scripts/setup/setup-fzf.sh 0.52.1

# Install ROS2
echo "Setting up ROS2..."
${ROOT}/scripts/setup/setup-ros.sh

# Install Graph MSF
echo "Setting up Graph MSF..."
${ROOT}/scripts/setup/setup-graph-msf.sh

# Setup shell configurations
echo "Setting up shell configurations..."

# Setup fzf completions
if ! grep -q "source <(fzf --zsh)" ${USER_HOME}/.zshrc; then
    echo "source <(fzf --zsh)" >> ${USER_HOME}/.zshrc
fi
if ! grep -q "source <(fzf --bash)" ${USER_HOME}/.bashrc; then
    echo "source <(fzf --bash)" >> ${USER_HOME}/.bashrc
fi

# Store command history in the workspace which is persistent across rebuilds
if ! grep -q "export HISTFILE=${ROOT}/.zsh_history" ${USER_HOME}/.zshrc; then
    echo "export HISTFILE=${ROOT}/.zsh_history" >> ${USER_HOME}/.zshrc
fi
if ! grep -q "export HISTFILE=${ROOT}/.bash_history" ${USER_HOME}/.bashrc; then
    echo "export HISTFILE=${ROOT}/.bash_history" >> ${USER_HOME}/.bashrc
fi

# Source the smb script
if ! grep -q "source ${ROOT}/scripts/smb_zshrc.sh" ${USER_HOME}/.zshrc; then
    echo "source ${ROOT}/scripts/smb_zshrc.sh" >> ${USER_HOME}/.zshrc
fi
if ! grep -q "source ${ROOT}/scripts/smb_bashrc.sh" ${USER_HOME}/.bashrc; then
    echo "source ${ROOT}/scripts/smb_bashrc.sh" >> ${USER_HOME}/.bashrc
fi

# Setup git config
echo "Setting up git configuration..."
sudo -u ${USER} git config --global core.autocrlf false # Prevent line ending conversion on Windows

# Setup tmux
echo "Setting up tmux configuration..."

# Create tmux directories with proper permissions
mkdir -p ${USER_HOME}/.config/tmux/plugins
chown -R ${USER}:${USER} ${USER_HOME}/.config/tmux

# Create tmux plugin directory
mkdir -p ${USER_HOME}/.tmux/plugins
chown -R ${USER}:${USER} ${USER_HOME}/.tmux

# Link tmux config
ln -sf ${ROOT}/.tmux.conf ${USER_HOME}/.config/tmux/tmux.conf
chown ${USER}:${USER} ${USER_HOME}/.config/tmux/tmux.conf

# Install tmux plugin manager
if [ ! -d ${USER_HOME}/.tmux/plugins/tpm ]; then
    sudo -u ${USER} git clone https://github.com/tmux-plugins/tpm ${USER_HOME}/.tmux/plugins/tpm
fi

# Start tmux server in the background as the user
sudo -u ${USER} tmux new-session -d

# Source tmux config
sudo -u ${USER} tmux source ${USER_HOME}/.config/tmux/tmux.conf

# Install tmux plugins non-interactively
sudo -u ${USER} ${USER_HOME}/.tmux/plugins/tpm/bin/install_plugins

# Stop tmux server
sudo -u ${USER} tmux kill-server

# Clean up
echo "Cleaning up..."
apt-get clean
pip cache purge
rm -rf /tmp/*

echo "Robot host setup completed successfully!" 