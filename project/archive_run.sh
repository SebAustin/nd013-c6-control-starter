#!/bin/bash
#
# Archive the tuning run that just finished.
#
# run_main_pid.sh erases steer_pid_data.txt and throttle_pid_data.txt at the
# start of every run, so run this script after each run you want to keep.
# It copies the two data files and pid_gains.txt into
#
#     runs/steer_<Kp>_<Ki>_<Kd>__throttle_<Kp>_<Ki>_<Kd>[__<note>]/
#
# and saves the plots there as PNG files. An existing archive is never
# overwritten.
#
# Usage, from the project/ directory:
#     ./archive_run.sh            # label built from the gains only
#     ./archive_run.sh final      # adds a note to the label

set -euo pipefail

readonly GAINS_FILE="pid_gains.txt"
readonly STEER_DATA="steer_pid_data.txt"
readonly THROTTLE_DATA="throttle_pid_data.txt"
readonly RUNS_DIR="runs"

fail() {
  echo "Error: $1" >&2
  exit 1
}

# Print "<Kp>_<Ki>_<Kd>" for one controller from the gains file.
gains_label() {
  local controller="$1"
  awk -v name="$controller" '
    { sub(/#.*/, "") }
    $1 == name && NF == 4 { print $2 "_" $3 "_" $4; found = 1; exit }
    END { if (!found) exit 1 }
  ' "$GAINS_FILE"
}

cd "$(dirname "$0")"

note="${1:-}"
if [[ -n "$note" && ! "$note" =~ ^[A-Za-z0-9_-]+$ ]]; then
  fail "the note may only contain letters, digits, '-' and '_'"
fi

[[ -f "$GAINS_FILE" ]] || fail "$GAINS_FILE not found; it records which gains this run used"
[[ -s "$STEER_DATA" ]] || fail "$STEER_DATA is missing or empty; has a run finished?"
[[ -s "$THROTTLE_DATA" ]] || fail "$THROTTLE_DATA is missing or empty; has a run finished?"

steer_label="$(gains_label steer)" || fail "no valid 'steer Kp Ki Kd' line in $GAINS_FILE"
throttle_label="$(gains_label throttle)" || fail "no valid 'throttle Kp Ki Kd' line in $GAINS_FILE"

label="steer_${steer_label}__throttle_${throttle_label}"
if [[ -n "$note" ]]; then
  label="${label}__${note}"
fi
run_dir="${RUNS_DIR}/${label}"

[[ ! -e "$run_dir" ]] || fail "$run_dir already exists; pass a note to archive a repeat run, e.g. ./archive_run.sh repeat2"

mkdir -p "$run_dir"
cp "$STEER_DATA" "$THROTTLE_DATA" "$GAINS_FILE" "$run_dir/"
echo "Archived run data in $run_dir"

if ! python3 plot_pid.py --data-dir "$run_dir" --save-dir "$run_dir" --no-show; then
  echo "Warning: the data is archived, but the plots could not be made." >&2
  echo "Fix the problem, then run:" >&2
  echo "  python3 plot_pid.py --data-dir $run_dir --save-dir $run_dir --no-show" >&2
  exit 2
fi
