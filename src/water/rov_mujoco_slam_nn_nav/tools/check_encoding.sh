#!/usr/bin/env bash
# 编码/换行自检：阻止 UTF-8 BOM 与 CRLF 回归。
# 用法：在包根目录执行 `bash tools/check_encoding.sh`（CI 可直接调用）。
set -euo pipefail
fail=0
while IFS= read -r -d '' f; do
  if [ "$(head -c 3 "$f" | od -An -tx1 | tr -d ' \n')" = "efbbbf" ]; then
    echo "BOM: $f"
    fail=1
  fi
done < <(find . \( -name '*.py' -o -name '*.sh' \) -not -path '*/.git/*' -print0)
while IFS= read -r -d '' f; do
  if grep -qU $'\r' "$f"; then
    echo "CRLF: $f"
    fail=1
  fi
done < <(find . -name '*.sh' -not -path '*/.git/*' -print0)
if [ "$fail" -eq 0 ]; then
  echo "OK: no BOM / CRLF"
fi
exit "$fail"
