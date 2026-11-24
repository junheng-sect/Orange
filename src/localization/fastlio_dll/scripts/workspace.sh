# Find the owning checkout from either a source script or its installed copy.
dll_workspace() {
  local directory="$1"
  while [[ "$directory" != / ]]; do
    if [[ -f "$directory/scripts/ros_net.sh" && -f "$directory/devel/setup.bash" ]]; then
      printf '%s\n' "$directory"
      return 0
    fi
    directory=$(dirname -- "$directory")
  done
  echo '找不到已构建的 Orange 工作空间，请检查检出位置或显式指定回放环境。' >&2
  return 1
}
