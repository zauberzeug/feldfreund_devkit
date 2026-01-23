# Getting Started

## Installation

Clone the repository and install dependencies with [uv](https://docs.astral.sh/uv/):

```bash
git clone https://github.com/zauberzeug/feldfreund_devkit.git
cd feldfreund_devkit
uv sync
```

## Running the Example

The repository includes a minimal example in `main.py` that demonstrates:

- Robot simulation with keyboard control
- Straight line navigation automation
- Real-time 3D visualization

Start it with:

```bash
uv run main.py
```

Open [http://localhost:8080](http://localhost:8080) in your browser. Hold **SHIFT** and use the **arrow keys** to steer the robot, or use the automation controls to run a straight line navigation.

## Understanding the Example

```python
--8<-- "main.py"
```

The `System` class extends `feldfreund_devkit.System` which initializes the robot hardware (or simulation) based on the configuration. Key components:

- **config**: Loaded from `config/example.py` via `config_from_id('example')`
- **steerer**: Manual steering control
- **driver**: Path-following driver for automations
- **navigation**: `StraightLineNavigation` drives forward for a configurable distance
- **automator**: Manages automation lifecycle (play/pause/stop)

## Configuration

Robot configurations live in the `config/` directory. See `config/example.py`:

```python
--8<-- "config/example.py"
```

In simulation mode (when no hardware is detected), mock implementations are used automatically.

## Next Steps

- Browse the **Module Reference** in the navigation for API documentation
- Check the [Tutorials](tutorials/tutorials.md) for hardware calibration guides
- See [Troubleshooting](troubleshooting.md) for common issues
