#!/usr/bin/env bash
# kerfur_cores.sh - micro-ROS agents for Kerfur's ESP32 sub-cores.
# One container per core, independent of the head start script.
#
#   kerfur_cores.sh up      [core...]   start an agent for every core that is plugged in
#   kerfur_cores.sh down    [core...]   stop and remove
#   kerfur_cores.sh restart [core...]   force recreate
#   kerfur_cores.sh status
#   kerfur_cores.sh logs <core>         follow agent output
#
# A core is /dev/kerfur_<name> (udev alias). No alias = not plugged in = skipped.
# `up` is safe to run any time: a healthy agent is left alone, and an agent whose
# board re-enumerated since it started (reflash, reset, replug, reboot) is recreated.
set -u

CORES=(touch neck waist)
IMAGE=microros/micro-ros-agent:jazzy
DOMAIN=42
BAUD=115200

dev()  { echo "/dev/kerfur_$1"; }
name() { echo "uros-$1"; }

# Identity of the current USB enumeration: tty name + kernel device number.
# The device number changes every time the board re-enumerates.
usbid() {
  local t; t=$(basename "$(readlink -f "$(dev "$1")")")
  echo "$t:$(cat "/sys/class/tty/$t/device/../devnum" 2>/dev/null)"
}

exists()  { docker container inspect "$(name "$1")" >/dev/null 2>&1; }
state()   { docker container inspect -f '{{.State.Status}}' "$(name "$1")" 2>/dev/null || echo none; }
label()   { docker container inspect -f '{{index .Config.Labels "kerfur.usbid"}}' "$(name "$1")" 2>/dev/null; }

up_one() {
  local c=$1 d n id
  d=$(dev "$c"); n=$(name "$c")
  if [ ! -e "$d" ]; then
    printf '%-6s skipped   %s not present\n' "$c" "$d"; return 0
  fi
  id=$(usbid "$c")
  if exists "$c"; then
    if [ "$(state "$c")" = running ] && [ "$(label "$c")" = "$id" ]; then
      printf '%-6s up        already running on %s\n' "$c" "${id%%:*}"; return 0
    fi
    docker rm -f "$n" >/dev/null
  fi
  if docker run -d --name "$n" --restart unless-stopped --net=host \
       --device="$d:$d" -e ROS_DOMAIN_ID="$DOMAIN" --label "kerfur.usbid=$id" \
       "$IMAGE" serial --dev "$d" -b "$BAUD" >/dev/null; then
    printf '%-6s started   %s -> %s\n' "$c" "$d" "${id%%:*}"
  else
    printf '%-6s FAILED    docker run returned an error\n' "$c"; return 1
  fi
}

down_one() {
  if exists "$1"; then docker rm -f "$(name "$1")" >/dev/null && printf '%-6s removed\n' "$1"
  else printf '%-6s not running\n' "$1"; fi
}

status_one() {
  local c=$1 d st note=""
  d=$(dev "$c"); st=$(state "$c")
  if [ -e "$d" ]; then
    [ "$st" != none ] && [ "$(label "$c")" != "$(usbid "$c")" ] && note="  STALE: board re-enumerated, run 'up'"
    printf '%-6s device %-8s agent %s%s\n' "$c" "$(basename "$(readlink -f "$d")")" "$st" "$note"
  else
    printf '%-6s device %-8s agent %s\n' "$c" "absent" "$st"
  fi
}

cmd=${1:-status}; shift || true
if [ $# -gt 0 ]; then sel=("$@"); else sel=("${CORES[@]}"); fi
for c in "${sel[@]}"; do
  case " ${CORES[*]} " in *" $c "*) ;; *) echo "unknown core '$c' (have: ${CORES[*]})" >&2; exit 2 ;; esac
done

rc=0
case "$cmd" in
  up)      for c in "${sel[@]}"; do up_one "$c" || rc=1; done ;;
  down)    for c in "${sel[@]}"; do down_one "$c"; done ;;
  restart) for c in "${sel[@]}"; do exists "$c" && docker rm -f "$(name "$c")" >/dev/null; up_one "$c" || rc=1; done ;;
  status)  for c in "${sel[@]}"; do status_one "$c"; done ;;
  logs)    [ $# -eq 1 ] || { echo "usage: $0 logs <core>" >&2; exit 2; }
           exec docker logs -f --tail 50 "$(name "$1")" ;;
  *)       echo "usage: $0 {up|down|restart|status|logs} [core...]" >&2; exit 2 ;;
esac
exit $rc
