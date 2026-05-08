#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_NAME=$(basename "$0")
WORKDIR_DEFAULT="/var/tmp/jetson_uart_dma_iommu_universal"
PATCHED_DTB="/boot/dtb/uart_dma_fix.dtb"
EXTLINUX="/boot/extlinux/extlinux.conf"
BACKUP_EXTLINUX="/boot/extlinux/extlinux.conf.jetson_uart_dma_iommu_universal.bak"
ACTIVE_DTB_BACKUP="/boot/dtb/uart_dma_fix.preexisting.bak"
SID_GPCDMA_HEX="0x04"
UART_NODE="serial@3100000"

log() { printf '[%s] %s\n' "$SCRIPT_NAME" "$*"; }
warn() { printf '[%s] WARNING: %s\n' "$SCRIPT_NAME" "$*" >&2; }
die() { printf '[%s] ERROR: %s\n' "$SCRIPT_NAME" "$*" >&2; exit 1; }
need_root() { [[ ${EUID:-$(id -u)} -eq 0 ]] || die "Run as root: sudo $SCRIPT_NAME ..."; }
need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"; }

usage() {
  cat <<USAGE
Usage:
  sudo $SCRIPT_NAME apply [--reboot]
  sudo $SCRIPT_NAME status
  sudo $SCRIPT_NAME restore [--reboot]

Purpose:
  Patch Jetson Orin / tegra234 UARTA serial@3100000 for the R36.5 UART DMA/IOMMU issue.

What apply does:
  - Dumps the live device tree from /sys/firmware/devicetree/base
  - Locates serial@3100000
  - Ensures the node has:
      dma-names = "rx", "tx";
      dmas = <...>;
      iommus = <smmu_niso0 0x04>;
  - Compiles /boot/dtb/uart_dma_fix.dtb
  - Points /boot/extlinux/extlinux.conf at that DTB
  - Verifies the staged DTB before success

Restore:
  - Restores extlinux.conf from backup.
USAGE
}

ensure_tools() {
  need_cmd dtc
  need_cmd python3
  need_cmd grep
  need_cmd sed
  need_cmd awk
  need_cmd cp
  need_cmd mkdir
}

read_nv_tegra_release() {
  [[ -f /etc/nv_tegra_release ]] && cat /etc/nv_tegra_release || true
}

assert_supported_platform() {
  local rel model compat
  rel=$(read_nv_tegra_release || true)
  model=$(tr -d '\0' < /sys/firmware/devicetree/base/model 2>/dev/null || true)
  compat=$(tr -d '\0' < /sys/firmware/devicetree/base/compatible 2>/dev/null || true)

  [[ -n "$rel" ]] || die "This does not look like a Jetson with /etc/nv_tegra_release present."
  [[ "$rel" == *"R36"* ]] || warn "Unexpected L4T release. Current release: ${rel//$'\n'/ }"
  [[ "$rel" == *"REVISION: 5."* || "$rel" == *"R36.5"* ]] || warn "This fix is mainly for R36.5.x. Current release: ${rel//$'\n'/ }"
  [[ "$model" == *"Orin"* || "$compat" == *"tegra234"* ]] || die "Unsupported platform. This script targets Orin / tegra234-class boards. Model='$model' compatible='$compat'"
}

make_workdir() {
  WORKDIR=${WORKDIR:-$WORKDIR_DEFAULT}
  mkdir -p "$WORKDIR"
}

dump_live_dts() {
  ACTIVE_DTS="$WORKDIR/active.dts"
  dtc -I fs -O dts -o "$ACTIVE_DTS" /sys/firmware/devicetree/base >/dev/null 2>&1 || \
    die "Failed to dump live device tree"
}

python_helper() { python3 - "$@"; }

summarize_dts_to_json() {
  local dts_file="$1"
  python_helper "$dts_file" <<'PY'
import json, re, sys
from pathlib import Path

text = Path(sys.argv[1]).read_text(errors="replace")

def find_block(text, node_name):
    m = re.search(r'(^|\n)(?P<indent>\s*)' + re.escape(node_name) + r'\s*\{', text)
    if not m:
        return None
    start = m.start()
    brace_start = m.end() - 1
    depth = 0
    i = brace_start
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                semi = text.find(";", i)
                return start, semi + 1, text[start:semi + 1]
        i += 1
    return None

def get_smmu_info(text):
    sym = find_block(text, "__symbols__")
    smmu_path = None
    if sym:
        _, _, symtxt = sym
        m = re.search(r'\bsmmu_niso0\s*=\s*"([^"]+)"\s*;', symtxt)
        if m:
            smmu_path = m.group(1)

    phandle = None
    candidates = []
    if smmu_path:
        candidates.append(smmu_path.rstrip("/").split("/")[-1])
    candidates += ["iommu@12000000", "smmu@12000000"]

    for node_name in candidates:
        for m in re.finditer(r'(^|\n)\s*' + re.escape(node_name) + r'\s*\{', text):
            block = find_block(text[m.start():], node_name)
            if not block:
                continue
            _, _, node = block
            pm = re.search(r'\bphandle\s*=\s*<\s*(0x[0-9a-fA-F]+|\d+)\s*>\s*;', node)
            if pm:
                return smmu_path, pm.group(1)
    return smmu_path, phandle

block = find_block(text, "serial@3100000")
if not block:
    print(json.dumps({"found": False}))
    raise SystemExit(0)

_, _, node = block
smmu_path, smmu_phandle = get_smmu_info(text)

def prop(pattern):
    m = re.search(pattern, node, flags=re.S)
    return m.group(1).strip() if m else None

state = {
    "found": True,
    "has_dmas": bool(re.search(r'\bdmas\s*=\s*[^;]+;', node, flags=re.S)),
    "has_dma_names": bool(re.search(r'\bdma-names\s*=\s*[^;]+;', node, flags=re.S)),
    "has_iommus": bool(re.search(r'\biommus\s*=\s*<[^;]+>\s*;', node, flags=re.S)),
    "dmas_value": prop(r'\bdmas\s*=\s*([^;]+);'),
    "dma_names_value": prop(r'\bdma-names\s*=\s*([^;]+);'),
    "iommus_value": prop(r'\biommus\s*=\s*(<[^;]+>);'),
    "smmu_path": smmu_path,
    "smmu_phandle": smmu_phandle,
}
print(json.dumps(state))
PY
}

print_state_pretty() {
  python_helper "$1" <<'PY'
import json, sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    print(json.dumps(json.load(f), indent=2))
PY
}

patch_dts() {
  PATCHED_DTS="$WORKDIR/uart_dma_fix.dts"
  python_helper "$ACTIVE_DTS" "$PATCHED_DTS" "$SID_GPCDMA_HEX" <<'PY'
import re, sys
from pathlib import Path

src = Path(sys.argv[1]).read_text(errors="replace")
out = Path(sys.argv[2])
sid = sys.argv[3]
node_name = "serial@3100000"

def find_node_span(text, node_name):
    m = re.search(r'(^|\n)(?P<indent>\s*)' + re.escape(node_name) + r'\s*\{', text)
    if not m:
        raise SystemExit(f"{node_name} not found")
    start = m.start()
    open_brace = m.end() - 1
    indent = m.group("indent")
    depth = 0
    i = open_brace
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                semi = text.find(";", i)
                if semi < 0:
                    raise SystemExit("UART node closing semicolon not found")
                return start, semi + 1, text[start:semi + 1], indent
        i += 1
    raise SystemExit("UART node end not found")

def find_smmu_phandle(text):
    # Prefer __symbols__. Fall back to common Orin SMMU node.
    smmu_path = None
    try:
        _, _, sym, _ = find_node_span(text, "__symbols__")
        m = re.search(r'\bsmmu_niso0\s*=\s*"([^"]+)"\s*;', sym)
        if m:
            smmu_path = m.group(1)
    except SystemExit:
        pass

    candidates = []
    if smmu_path:
        candidates.append(smmu_path.rstrip("/").split("/")[-1])
    candidates += ["iommu@12000000", "smmu@12000000"]

    for cand in candidates:
        for m in re.finditer(r'(^|\n)\s*' + re.escape(cand) + r'\s*\{', text):
            # Parse from this node occurrence by manual local scan.
            start = m.start()
            brace = text.find("{", m.start())
            depth = 0
            i = brace
            while i < len(text):
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                    if depth == 0:
                        semi = text.find(";", i)
                        block = text[start:semi + 1]
                        pm = re.search(r'\bphandle\s*=\s*<\s*(0x[0-9a-fA-F]+|\d+)\s*>\s*;', block)
                        if pm:
                            return pm.group(1)
                        break
                i += 1
    raise SystemExit("Could not find SMMU phandle")

start, end, node, indent = find_node_span(src, node_name)
inner_indent = indent + "\t"

# Preserve existing DMA values if present; otherwise use UARTA/GPCDMA channel 8 defaults seen on affected systems.
m = re.search(r'\bdmas\s*=\s*([^;]+);', node, flags=re.S)
dmas_value = " ".join(m.group(1).split()) if m else "<0xed 0x08 0xed 0x08>"

m = re.search(r'\bdma-names\s*=\s*([^;]+);', node, flags=re.S)
dma_names_value = " ".join(m.group(1).split()) if m else '"rx", "tx"'

smmu_phandle = find_smmu_phandle(src)
iommus_value = f"<{smmu_phandle} {sid}>"

# Remove old versions of the properties from inside the node.
node2 = re.sub(r'\n[ \t]*dma-names\s*=\s*[^;]+;', '', node, flags=re.S)
node2 = re.sub(r'\n[ \t]*dmas\s*=\s*[^;]+;', '', node2, flags=re.S)
node2 = re.sub(r'\n[ \t]*iommus\s*=\s*[^;]+;', '', node2, flags=re.S)

# Insert before the final closing brace of THIS node, robust to tabs/spaces.
close_match = list(re.finditer(r'\n(?P<indent>[ \t]*)\};\s*$', node2, flags=re.S))
if not close_match:
    raise SystemExit("Could not locate final UART node closing brace")
cm = close_match[-1]
insert = (
    f"\n{inner_indent}dma-names = {dma_names_value};"
    f"\n{inner_indent}dmas = {dmas_value};"
    f"\n{inner_indent}iommus = {iommus_value};"
)
node2 = node2[:cm.start()] + insert + node2[cm.start():]

patched = src[:start] + node2 + src[end:]
out.write_text(patched)

print(f"Patched {node_name}")
print(f"  dma-names = {dma_names_value};")
print(f"  dmas = {dmas_value};")
print(f"  iommus = {iommus_value};")
PY
}

compile_and_install_dtb() {
  mkdir -p /boot/dtb
  COMPILED_DTB="$WORKDIR/uart_dma_fix.dtb"
  dtc -I dts -O dtb -o "$COMPILED_DTB" "$PATCHED_DTS" >/dev/null || die "Failed to compile patched DTS"

  [[ -f "$PATCHED_DTB" && ! -f "$ACTIVE_DTB_BACKUP" ]] && cp "$PATCHED_DTB" "$ACTIVE_DTB_BACKUP"
  cp "$COMPILED_DTB" "$PATCHED_DTB"
}

update_extlinux_to_use_patched_dtb() {
  [[ -f "$EXTLINUX" ]] || die "Missing $EXTLINUX"
  [[ ! -f "$BACKUP_EXTLINUX" ]] && cp "$EXTLINUX" "$BACKUP_EXTLINUX"

  python_helper "$EXTLINUX" "$PATCHED_DTB" <<'PY'
import re, sys
from pathlib import Path
extlinux = Path(sys.argv[1])
dtb_path = sys.argv[2]
text = extlinux.read_text()

lines = text.splitlines()
out = []
in_primary = False
inserted = False

for line in lines:
    stripped = line.strip()
    if stripped.startswith("LABEL "):
        in_primary = stripped == "LABEL primary"
    if in_primary and re.match(r'^\s*FDT\s+', line):
        out.append(re.sub(r'^\s*FDT\s+.*$', '      FDT ' + dtb_path, line))
        inserted = True
        continue
    out.append(line)
    if in_primary and re.match(r'^\s*LINUX\s+', line) and not inserted:
        out.append('      FDT ' + dtb_path)
        inserted = True

if not inserted:
    raise SystemExit("Could not insert FDT line in extlinux.conf")

extlinux.write_text("\n".join(out) + "\n")
print("extlinux.conf points to", dtb_path)
PY
}

verify_staged_dtb() {
  local staged_dts="$WORKDIR/staged.dts"
  local staged_json="$WORKDIR/staged_state.json"
  dtc -I dtb -O dts -o "$staged_dts" "$PATCHED_DTB" >/dev/null 2>&1 || die "Failed to decompile staged DTB"
  summarize_dts_to_json "$staged_dts" > "$staged_json"

  python_helper "$staged_json" <<'PY'
import json, sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    s = json.load(f)
missing = [k for k in ("has_dmas", "has_dma_names", "has_iommus") if not s.get(k)]
if not s.get("found"):
    raise SystemExit("Verification failed: serial@3100000 not found")
if missing:
    raise SystemExit("Verification failed: " + ", ".join(missing) + " false")
print("Verified staged DTB: serial@3100000 has dmas, dma-names, and iommus.")
print(json.dumps(s, indent=2))
PY
}

status_report() {
  ensure_tools
  make_workdir
  dump_live_dts
  local state_json="$WORKDIR/live_state.json"
  summarize_dts_to_json "$ACTIVE_DTS" > "$state_json"

  echo "=== extlinux FDT lines ==="
  [[ -f "$EXTLINUX" ]] && grep -nE '^LABEL |^[[:space:]]*FDT |^[[:space:]]*LINUX ' "$EXTLINUX" || true
  echo
  echo "=== Active UARTA state from live DT ==="
  print_state_pretty "$state_json"
  echo
  echo "=== UARTA mode from dmesg ==="
  dmesg | egrep '3100000.serial|ttyTHS1|DMA|PIO|iommu group|Adding to iommu group' || true
}

restore_previous() {
  need_root
  [[ -f "$BACKUP_EXTLINUX" ]] || die "No backup extlinux found at $BACKUP_EXTLINUX"
  cp "$BACKUP_EXTLINUX" "$EXTLINUX"
  log "Restored $EXTLINUX from backup."
}

apply_fix() {
  need_root
  ensure_tools
  assert_supported_platform
  make_workdir
  dump_live_dts

  local pre_json="$WORKDIR/pre_state.json"
  summarize_dts_to_json "$ACTIVE_DTS" > "$pre_json"
  echo "Pre-check state:"
  print_state_pretty "$pre_json"

  patch_dts
  compile_and_install_dtb
  update_extlinux_to_use_patched_dtb
  verify_staged_dtb

  log "Patched DTB installed at $PATCHED_DTB and extlinux.conf updated."
  log "Reboot required. After reboot, run: sudo $SCRIPT_NAME status"
}

main() {
  local mode="${1:-}"
  local do_reboot=0
  case "$mode" in
    apply|status|restore) shift ;;
    -h|--help|'') usage; exit 0 ;;
    *) usage; die "Unknown mode: $mode" ;;
  esac

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --reboot) do_reboot=1; shift ;;
      *) die "Unknown argument: $1" ;;
    esac
  done

  case "$mode" in
    apply)
      apply_fix
      [[ $do_reboot -eq 1 ]] && { log "Rebooting now..."; reboot; }
      ;;
    status)
      status_report
      ;;
    restore)
      restore_previous
      [[ $do_reboot -eq 1 ]] && { log "Rebooting now..."; reboot; }
      ;;
  esac
}

main "$@"
