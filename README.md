# Artemis MuJoCo Teaching Demo

This repository contains a teaching-oriented MuJoCo demo based on the 2024 NUEDC "automatic driving car" problem. The demo keeps the competition geometry, separates route generation from control, and uses a planar MuJoCo vehicle so the code stays easy to explain in class.

## What is included

- Arena geometry reconstructed from the problem statement
- Three demo tasks that match the first three requirements
- A simulated line-sensor array that samples the black semicircle traces
- A hybrid controller that combines odometry waypoints and closed-loop line following
- A MuJoCo scene builder and a headless or interactive simulator
- Focused unit tests for geometry, controller behavior, and a smoke test for the simulator

## Architecture

- `artemis_mudri.domains.geometry`: shared geometry types and sampling helpers
- `artemis_mudri.domains.track`: arena layout and reference path modeling
- `artemis_mudri.domains.task`: task catalog loading and route planning
- `artemis_mudri.domains.vehicle`: vehicle state, sensing, and control
- `artemis_mudri.domains.simulation`: simulation summary value objects
- `artemis_mudri.application`: use-case orchestration for running a demo
- `artemis_mudri.infrastructure.mujoco`: MuJoCo config, XML rendering, model persistence, simulator backend
- `artemis_mudri.interfaces`: CLI entry points
- `assets/tasks`: JSON task declarations

## Teaching simplification

The MuJoCo model is still planar instead of contact-driven, but the controller no longer reads the reference path as an oracle. It receives only simulated line-sensor readings and odometry, then outputs chassis-neutral body motion commands through a task state machine. A chassis model then maps those commands to either differential or Ackermann actuation. This keeps the code stable for teaching while preserving the actual control logic. If you later want higher fidelity, you can replace the MuJoCo backend without changing the domain sensing and controller modules.

## Install

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

## Run

Headless run:

```bash
poetry run python -m artemis_mudri --task 1
poetry run python -m artemis_mudri --task 2 --max-time 35
poetry run python -m artemis_mudri --task 3 --summary-json /tmp/task3_summary.json
poetry run python -m artemis_mudri --task 3 --chassis ackermann --drift-mode corner
```

Interactive viewer:

```bash
poetry run python -m artemis_mudri --task 2 --render
```

Compatibility entrypoint:

```bash
artemis-demo --task 1
```

The Typer CLI prints route length, elapsed time, event timestamps, and cross-track error statistics.

## Test

```bash
python3 -m unittest discover -s tests -v
```
