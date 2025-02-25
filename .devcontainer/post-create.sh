#!/bin/bash

# Get the current user and group
USERNAME=$(whoami)
USERGROUP=$(id -gn)

echo "Setting folder permissions for user: $USERNAME"

# Change ownership of the current directory and all its contents recursively
sudo chown -R $USERNAME:$USERGROUP .

# Set appropriate permissions
# 755 for directories (rwx for owner, rx for group and others)
# 644 for files (rw for owner, r for group and others)
find . -type d -exec chmod 755 {} \;
find . -type f -exec chmod 644 {} \;

# Make sure script files are executable
find . -name "*.sh" -exec chmod +x {} \;

echo "Permissions updated successfully!"
