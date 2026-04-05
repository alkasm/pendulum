#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
    echo "usage: $0 <serial-port> [baud] [bind-addr]" >&2
    exit 2
fi

serial_port="$1"
baud="${2:-115200}"
bind_addr="${3:-127.0.0.1:7001}"

proxy_log="penproxy.log"
watch_log="pendulum_watch.log"

cleanup() {
    if [[ -n "${proxy_pid:-}" ]]; then
        kill "${proxy_pid}" 2>/dev/null || true
        wait "${proxy_pid}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

echo "Capturing proxy log to ${proxy_log}"
echo "Capturing watcher log to ${watch_log}"
echo "Press Ctrl-C after one balancing attempt."

rm -f "${proxy_log}" "${watch_log}"

cargo run --release -p penproxy --bin penproxy -- --bind "${bind_addr}" serial --port "${serial_port}" --baud "${baud}" \
    > "${proxy_log}" 2>&1 &
proxy_pid=$!

sleep 1

cargo run --release -p penproxy --bin pendulum_watch -- "${bind_addr}" | tee "${watch_log}"
