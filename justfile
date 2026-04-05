mod build
mod flash

default:
    @just --list

capture_pendulum port baud="115200" bind="127.0.0.1:7001":
    ./scripts/capture_pendulum.sh {{port}} {{baud}} {{bind}}

notebook:
    uv run --with jupyter jupyter notebook notebooks/sim.ipynb
