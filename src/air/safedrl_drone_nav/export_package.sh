#!/usr/bin/env bash
#
# export_package.sh - clean caches, verify the evidence chain, and package the
# submission.
#
#     ./export_package.sh                 # clean + verify + tarball
#     ./export_package.sh --no-archive     # clean + verify only
#     ./export_package.sh --with-tb        # include the (large) TensorBoard logs
#
# The script is deliberately conservative: it refuses to produce a package if
# the run artefacts a reviewer would look for are missing, so a silent
# `export_package.sh` is itself evidence that the pipeline was executed.
#
set -euo pipefail

PKG_NAME="SafeDRL-DroneNav-ROS"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

MAKE_ARCHIVE=1
WITH_TB=0
for arg in "$@"; do
  case "$arg" in
    --no-archive) MAKE_ARCHIVE=0 ;;
    --with-tb)    WITH_TB=1 ;;
    -h|--help)    sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

say() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
ok()  { printf '   \033[32m✓\033[0m %s\n' "$*"; }
bad() { printf '   \033[31m✗\033[0m %s\n' "$*"; }

# --------------------------------------------------------------------------
say "1. Removing caches and build artefacts"
# --------------------------------------------------------------------------
# NOTE: only genuine caches belong here.  logs/session_*.log look like clutter
# but are per-run transcripts - i.e. evidence - so they are kept.  (An earlier
# version deleted them during packaging, which silently destroyed part of the
# run-trace chain this project is supposed to preserve.)
removed=0
for pattern in \
    "__pycache__" "*.pyc" "*.pyo" ".pytest_cache" ".mypy_cache" \
    "*.egg-info" ".ipynb_checkpoints" "build" "devel"
do
  while IFS= read -r -d '' path; do
    rm -rf "$path"; removed=$((removed + 1))
  done < <(find . -name "$pattern" -not -path "./.git/*" -print0 2>/dev/null || true)
done
ok "removed $removed cache/build entr(ies)"

# Never ship stray editor/OS junk.
find . -name ".DS_Store" -o -name "Thumbs.db" -o -name "*~" | while read -r f; do
  [ -n "$f" ] && rm -f "$f"
done
ok "removed OS/editor cruft"

# --------------------------------------------------------------------------
say "2. Verifying the run-trace evidence chain"
# --------------------------------------------------------------------------
fail=0
require_file() {
  if [ -s "$1" ]; then ok "$1"; else bad "MISSING or empty: $1"; fail=1; fi
}
require_glob() {
  # shellcheck disable=SC2086
  local hits; hits=$(ls $1 2>/dev/null | wc -l | tr -d ' ')
  if [ "$hits" -gt 0 ]; then ok "$1  ($hits file(s))"; else bad "no match: $1"; fail=1; fi
}

echo "   -- terminal transcripts --"
require_file logs/train_execution.log
require_glob "logs/terminal_snippet_*.txt"

echo "   -- per-run session transcripts --"
require_glob "logs/session_*.log"

echo "   -- training metrics --"
require_glob "logs/tb_logs/*"

echo "   -- evaluation metrics --"
require_glob "logs/eval_metrics_*.json"
require_glob "logs/eval_metrics_*_episodes.csv"

echo "   -- figures --"
require_glob "docs/figures/*.png"

echo "   -- model checkpoints --"
require_glob "checkpoints/*.zip"

echo "   -- traces --"
require_glob "logs/traces/*.npz"

# --------------------------------------------------------------------------
say "3. Summary of what will be packaged"
# --------------------------------------------------------------------------
du -sh logs docs checkpoints 2>/dev/null || true
printf '\n   artefacts:\n'
printf '     figures        : %s\n' "$(ls docs/figures/*.png 2>/dev/null | wc -l | tr -d ' ')"
printf '     checkpoints    : %s\n' "$(ls checkpoints/*.zip 2>/dev/null | wc -l | tr -d ' ')"
printf '     tb event dirs  : %s\n' "$(ls -d logs/tb_logs/*/ 2>/dev/null | wc -l | tr -d ' ')"
printf '     eval json      : %s\n' "$(ls logs/eval_metrics_*.json 2>/dev/null | wc -l | tr -d ' ')"

if [ "$fail" -ne 0 ]; then
  echo
  bad "evidence chain incomplete - run scripts/train.py + evaluate.py first"
  exit 1
fi

# --------------------------------------------------------------------------
if [ "$MAKE_ARCHIVE" -eq 1 ]; then
  say "4. Building the submission archive"
  STAMP="$(date +%Y%m%d_%H%M%S)"
  # Absolute path: the `tar` below runs inside $TMP, so a relative OUT would be
  # resolved against the temp directory and then deleted along with it.
  OUT="$(cd .. && pwd)/${PKG_NAME}_${STAMP}.tar.gz"
  TMP="$(mktemp -d)"
  mkdir -p "$TMP/$PKG_NAME"

  # `dist/` holds previous archives; excluding it prevents the tarball from
  # recursively containing itself, which would grow on every run.
  if [ "$WITH_TB" -eq 1 ]; then
    tar --exclude='.git' --exclude='./dist' -cf - . | tar -xf - -C "$TMP/$PKG_NAME"
  else
    # TensorBoard event files are large and re-generable; exclude unless asked.
    tar --exclude='.git' --exclude='./dist' --exclude='./logs/tb_logs' -cf - . \
      | tar -xf - -C "$TMP/$PKG_NAME"
    mkdir -p "$TMP/$PKG_NAME/logs/tb_logs"
    cat > "$TMP/$PKG_NAME/logs/tb_logs/README.txt" <<'EOF'
TensorBoard event files were excluded from this archive to keep it small.
Regenerate them with:

    python scripts/train.py --policy ppo_lag --use-cbf \
        --init-from checkpoints/bc_warmstart_cbf.zip --normalize-reward \
        --run-name ppo_lag_cbf

then inspect with:

    tensorboard --logdir logs/tb_logs
EOF
  fi

  # A manifest so the archive explains itself.
  {
    echo "# Submission manifest"
    echo "package    : $PKG_NAME"
    echo "created    : $(date -Iseconds)"
    echo "host       : $(hostname)"
    echo "git commit : $(git rev-parse --short HEAD 2>/dev/null || echo 'n/a')"
    echo
    echo "## Evidence index"
    echo "logs/train_execution.log          full terminal transcript (stdout+stderr)"
    echo "logs/terminal_snippet_*.txt       verbatim timestamped terminal excerpts"
    echo "logs/tb_logs/                     TensorBoard scalars ($([ "$WITH_TB" -eq 1 ] && echo included || echo excluded))"
    echo "logs/eval_metrics_*.json          structured evaluation metrics"
    echo "logs/eval_metrics_*_episodes.csv  per-episode detail"
    echo "logs/traces/*.npz                 raw per-step telemetry"
    echo "docs/figures/*.png                all report figures"
    echo "checkpoints/*.zip                 trained weights"
    echo
    echo "## Reproduce"
    echo "1. pip install -r requirements.txt"
    echo "2. python scripts/pretrain_bc.py --use-cbf           # warm start"
    echo "3. python scripts/train.py --policy ppo_lag --use-cbf \\"
    echo "       --init-from checkpoints/bc_warmstart_cbf.zip --normalize-reward"
    echo "4. python scripts/evaluate.py --model checkpoints/ppo_lag_cbf_final.zip \\"
    echo "       --use-cbf --run-name ppo_lag_cbf --figures"
    echo "5. python scripts/make_report_figures.py --policy ppo_baseline \\"
    echo "       --policy ppo_lagrangian --policy ppo_lag_cbf"
    echo "6. roslaunch safedrl_drone_nav smoke_test.launch       # ROS loop check"
  } > "$TMP/$PKG_NAME/MANIFEST.txt"

  ( cd "$TMP" && tar -czf "$OUT" "$PKG_NAME" )
  [ -s "$OUT" ] || { bad "archive was not created at $OUT"; exit 1; }
  rm -rf "$TMP"
  ok "archive: $OUT"
  printf '   size   : %s\n' "$(du -h "$OUT" | cut -f1)"
  printf '   files  : %s\n' "$(tar -tzf "$OUT" | wc -l | tr -d ' ')"
else
  say "4. Archive skipped (--no-archive)"
fi

say "Done"
echo "   The package is ready for submission."
