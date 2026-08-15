#!/usr/bin/env bash
# Copy the moveit2-extended packages from this fork into the nav2 workspace so colcon can build
# them.
#
# Why a copy and not a symlink: the dgx-nav2 container mounts only
# /Work39_SSD/cheesiang_leow/nav2_workspace. This fork lives under the user's home directory,
# which is not mounted, so a symlink from src/ into the fork dangles inside the container and
# colcon silently skips the package. Until the fork lives inside the mounted tree (or the
# container is recreated with a second mount), the fork is the git home and the workspace gets a
# copy.
#
# The fork is authoritative. This script only ever writes into the workspace, never back, so an
# accidental edit in src/ is overwritten rather than silently becoming the source of truth.
#
#   bash sync_to_nav2_workspace.sh          # sync
#   bash sync_to_nav2_workspace.sh --check  # report drift, change nothing
set -euo pipefail

FORK="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${NAV2_WORKSPACE:-/misc/Work39_SSD/cheesiang_leow/nav2_workspace}"
DEST="$WS/src"

PACKAGES=(
  moveit_cable_carrier
  moveit2_extended_msgs
  moveit2_extended_core
  moveit2_extended_behaviors
  moveit2_extended_carrier
  moveit2_extended_waypoints
  moveit2_extended_tools
  moveit2_extended_studio
  moveit2_extended_rviz
)

if [[ ! -d "$DEST" ]]; then
  echo "workspace src not found: $DEST" >&2
  exit 1
fi

check_only=0
[[ "${1:-}" == "--check" ]] && check_only=1

status=0
for pkg in "${PACKAGES[@]}"; do
  src="$FORK/$pkg"
  [[ -d "$src" ]] || continue

  if (( check_only )); then
    # --dry-run with an itemised list: anything printed is drift.
    drift="$(rsync -rc --dry-run --delete --itemize-changes \
             --exclude='README.md' --exclude='.git' \
             "$src/" "$DEST/$pkg/" 2>/dev/null || true)"
    if [[ -n "$drift" ]]; then
      echo "DRIFT in $pkg:"; echo "$drift" | sed 's/^/  /'
      status=1
    else
      echo "ok    $pkg"
    fi
    continue
  fi

  mkdir -p "$DEST/$pkg"
  # -c compares checksums rather than timestamps, so a copied-back file with a newer mtime but
  # identical content does not churn. README.md is fork-only on purpose.
  rsync -rc --delete --exclude='README.md' --exclude='.git' "$src/" "$DEST/$pkg/"
  echo "synced $pkg"
done

exit $status
