#!/usr/bin/env bash
set -euo pipefail

default_port="/dev/cu.usbserial-110"
default_baud="115200"
default_bind="127.0.0.1:7001"
default_excerpt_lines="20"

if [[ $# -gt 4 ]]; then
    echo "usage: $0 [serial-port] [baud] [bind-addr] [excerpt-lines]" >&2
    exit 2
fi

serial_port="${1:-$default_port}"
baud="${2:-$default_baud}"
bind_addr="${3:-$default_bind}"
excerpt_lines="${4:-$default_excerpt_lines}"

proxy_log="penproxy.log"
watch_log="pendulum_watch.log"
watch_excerpt_log="pendulum_watch_excerpt.log"

write_excerpt() {
    if [[ ! -f "${watch_log}" ]]; then
        return
    fi

    grep '^seq=' "${watch_log}" | head -n "${excerpt_lines}" > "${watch_excerpt_log}" || true

    if [[ -s "${watch_excerpt_log}" ]]; then
        echo
        echo "Saved first ${excerpt_lines} telemetry lines to ${watch_excerpt_log}"
        cat "${watch_excerpt_log}"
    else
        echo
        echo "No pendulum telemetry lines were captured in ${watch_log}" >&2
    fi
}

cleanup() {
    if [[ -n "${proxy_pid:-}" ]]; then
        kill "${proxy_pid}" 2>/dev/null || true
        wait "${proxy_pid}" 2>/dev/null || true
    fi
    write_excerpt
}

trap cleanup EXIT INT TERM

echo "Capturing proxy log to ${proxy_log}"
echo "Capturing watcher log to ${watch_log}"
echo "Using serial port ${serial_port}"
echo "Press Ctrl-C after one balancing attempt."

rm -f "${proxy_log}" "${watch_log}" "${watch_excerpt_log}"

cargo run --release -p penproxy --bin penproxy -- --bind "${bind_addr}" serial --port "${serial_port}" --baud "${baud}" \
    > "${proxy_log}" 2>&1 &
proxy_pid=$!

sleep 1

cargo run --release -p penproxy --bin pendulum_watch -- "${bind_addr}" | tee "${watch_log}"
