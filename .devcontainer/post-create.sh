#!/bin/bash

# Setup fzf
echo "eval \"\$(fzf --bash)\"" >> ~/.bashrc
echo "source <(fzf --zsh)" >> ~/.zshrc
