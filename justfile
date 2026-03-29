mod build
mod flash

default:
    @just --list

notebook:
    uv run --with jupyter jupyter notebook notebooks/sim.ipynb
