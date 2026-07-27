#!/usr/bin/env bash
# Source the ROS distribution and prefer this repository's nested install tree.
#
# Usage:
#   source aqua_localization/scripts/setup_nested_workspace_env.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source this script so environment changes persist:" >&2
  echo "  source $0" >&2
  exit 2
fi

_aqua_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export AQUA_WORKSPACE="${AQUA_WORKSPACE:-$(cd -- "$_aqua_script_dir/../.." && pwd)}"
_aqua_ros_setup="${AQUA_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -r "$_aqua_ros_setup" ]]; then
  echo "ROS setup is not readable: $_aqua_ros_setup" >&2
  return 1
fi

# shellcheck disable=SC1090
source "$_aqua_ros_setup"

if [[ -r "$AQUA_WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$AQUA_WORKSPACE/install/setup.bash"
fi

_aqua_prepend_path_once() {
  local name="$1"
  local value="$2"
  local current="${!name:-}"
  case ":$current:" in
    *":$value:"*) ;;
    *) printf -v "$name" '%s%s%s' "$value" "${current:+:}" "$current" ;;
  esac
  export "$name"
}

shopt -s nullglob
for _aqua_prefix in "$AQUA_WORKSPACE"/install/*; do
  if [[ ! -d "$_aqua_prefix" ]]; then
    continue
  fi
  _aqua_prepend_path_once AMENT_PREFIX_PATH "$_aqua_prefix"
  if [[ -d "$_aqua_prefix/lib" ]]; then
    _aqua_prepend_path_once LD_LIBRARY_PATH "$_aqua_prefix/lib"
  fi
done
shopt -u nullglob

unset _aqua_script_dir _aqua_ros_setup _aqua_prefix
unset -f _aqua_prepend_path_once

