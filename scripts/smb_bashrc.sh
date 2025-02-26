# This configuration file provides common settings for Docker containers and robot hosts.
# It includes useful aliases and environment configurations to enhance productivity.
#
# shellcheck shell=bash      # shellcheck: ignore missing shebang as this file is meant to be sourced
# shellcheck disable=SC2139  # shellcheck: allow expansion during alias definition rather than execution

# Safety check: ensure this script is being sourced and not executed directly
if [ "${BASH_SOURCE[0]}" -ef "$0" ] ; then
  echo "ERROR: $(basename "${BASH_SOURCE[0]}") must be sourced, not executed directly!"
  exit 1
fi