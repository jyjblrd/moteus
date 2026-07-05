#!/bin/bash
# Build and test the current moteus checkout on a remote build
# server, inside a dedicated detached git worktree so that other
# checkouts on the server are left untouched.
#
# Usage:
#   utils/remote_build.sh [host] [remote-repo-path]
#
# Defaults:
#   host              root@142.93.3.84, or $MOTEUS_BUILD_SERVER
#   remote-repo-path  autodetected among common locations, or
#                     $MOTEUS_REMOTE_REPO

set -euo pipefail

HOST="${1:-${MOTEUS_BUILD_SERVER:-root@142.93.3.84}}"
REMOTE_REPO="${2:-${MOTEUS_REMOTE_REPO:-}}"

cd "$(git rev-parse --show-toplevel)"

SHA=$(git rev-parse HEAD)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
SAFE_BRANCH="${BRANCH//\//-}"

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "warning: uncommitted changes will NOT be built (building $SHA)" >&2
fi

if [ -z "$REMOTE_REPO" ]; then
  REMOTE_REPO=$(ssh "$HOST" '
    for d in $HOME/moteus /root/moteus $HOME/src/moteus /opt/moteus; do
      if [ -e "$d/.git" ]; then echo "$d"; break; fi
    done')
  if [ -z "$REMOTE_REPO" ]; then
    echo "error: could not find a moteus repo on $HOST" >&2
    echo "pass it explicitly: utils/remote_build.sh $HOST /path/to/moteus" >&2
    exit 1
  fi
fi

WORKTREE="/root/moteus-build-worktrees/$SAFE_BRANCH"

echo "== Pushing $SHA to $HOST:$REMOTE_REPO"
# Push to a ref outside refs/heads so no checked-out branch can
# conflict with the receive.
git push --force --quiet "$HOST:$REMOTE_REPO" "$SHA:refs/build/$SAFE_BRANCH"

echo "== Preparing worktree $WORKTREE"
ssh "$HOST" "
  set -e
  cd '$REMOTE_REPO'
  if [ ! -e '$WORKTREE' ]; then
    mkdir -p \"\$(dirname '$WORKTREE')\"
    git worktree add --detach '$WORKTREE' $SHA
  else
    git -C '$WORKTREE' checkout --detach $SHA
  fi
"

echo "== Running host tests"
ssh "$HOST" "cd '$WORKTREE' && tools/bazel test --config=host //:host"

echo "== Building target firmware"
ssh "$HOST" "cd '$WORKTREE' && tools/bazel build --config=target //:target"

echo "== OK: $SHA tested and built on $HOST in $WORKTREE"
