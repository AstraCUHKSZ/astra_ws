#!/usr/bin/env bash
# Commit and push this monorepo using one message.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./commit.sh [--dry-run] "commit message"

Stages all changes (including untracked files), commits them, then pushes the
current branch to its configured upstream.
EOF
}

dry_run=false
case "${1:-}" in
  --dry-run)
    dry_run=true
    shift
    ;;
  -h|--help)
    usage
    exit 0
    ;;
esac

if [[ $# -ne 1 || -z $1 ]]; then
  usage >&2
  exit 2
fi
message=$1

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
if ! git -C "$root" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  printf 'error: %s is not a Git work tree\n' "$root" >&2
  exit 1
fi

if [[ -z $(git -C "$root" status --porcelain=v1) ]]; then
  printf 'Nothing to commit.\n'
  exit 0
fi

if "$dry_run"; then
  printf 'Dry run only; no files were changed.\n'
  exit 0
fi

git -C "$root" add -A
git -C "$root" commit -m "$message"
git -C "$root" push
