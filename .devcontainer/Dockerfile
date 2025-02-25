FROM ubuntu:24.04

# Prevent interactive prompts during installation
ARG DEBIAN_FRONTEND=noninteractive

# Set up locale and timezone
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      locales \
      tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV LANG="en_US.UTF-8"

# Install essential system utilities
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      sudo \
      git \
      git-lfs \
      curl \
      wget \
      ca-certificates \
      gnupg \
      lsb-release \
      python3 \
      python3-pip \
      python-is-python3 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      build-essential \
      cmake \
      gdb \
      bash-completion \
      vim \
      tmux \
      tree \
      inetutils-ping \
      x11-apps \
      dbus-x11 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install code formatting and linting tools
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      clang-format && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# We'll let the devcontainer.json handle user creation
# This prevents conflicts with the common-utils feature
