mod build
mod flash

default:
    @just --list

pendev *args:
    cargo r -p pendev -- {{args}}

diagnose port="/dev/cu.usbserial-110" baud="115200" bind="127.0.0.1:7001" lines="20":
    ./scripts/capture_pendulum.sh {{port}} {{baud}} {{bind}} {{lines}}

capture port="/dev/cu.usbserial-110" baud="115200" bind="127.0.0.1:7001":
    ./scripts/capture_pendulum.sh {{port}} {{baud}} {{bind}}

notebook:
    uv run --with jupyter jupyter notebook notebooks/sim.ipynb
