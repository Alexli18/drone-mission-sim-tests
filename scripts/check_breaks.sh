#!/usr/bin/env bash
# Prove that the unit tests notice deliberate breaks: for every patch in docs/breaks/,
# apply it to a clean copy of the committed tree and require `pytest -m unit` to fail;
# first require the unmodified copy to pass. Patches are run from the committed tree
# (git archive HEAD), so commit before running.
#
# Usage: check_breaks.sh [PYTHON]   (default: python; needs the dev and ros extras)
set -uo pipefail

python="${1:-python}"
cd "$(dirname "$0")/.." || exit 2
root="$PWD"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

fresh_copy() {
  rm -rf "${work:?}/tree"
  mkdir "$work/tree"
  git archive HEAD | tar -x -C "$work/tree"
}

run_unit() {
  (cd "$work/tree" && "$python" -m pytest -m unit -q -p no:cacheprovider >"$work/out.txt" 2>&1)
}

fresh_copy
if ! run_unit; then
  echo "FAIL: the unmodified tree does not pass the unit tests:" >&2
  tail -20 "$work/out.txt" >&2
  exit 1
fi
echo "ok    clean tree: unit tests pass"

status=0
for patch in docs/breaks/*.patch; do
  fresh_copy
  if ! (cd "$work/tree" && git init -q && git apply "$root/$patch" 2>"$work/apply.txt"); then
    echo "FAIL  $patch does not apply:" >&2
    cat "$work/apply.txt" >&2
    status=1
    continue
  fi
  if run_unit; then
    echo "FAIL  $patch: the unit tests still pass, the break went unnoticed" >&2
    status=1
  else
    echo "ok    $patch: $(tail -1 "$work/out.txt")"
    grep -E '^(FAILED|ERROR)' "$work/out.txt" | head -3 | sed 's/^/        /'
  fi
done
exit "$status"
