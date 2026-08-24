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
- A single straight line navigation, driven as an automation
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
- **driver**: Follows a single spline, and holds the tunable driving parameters
- **path_driver**: Drives one `DriveSegment` at a time with the `driver`, and is what a tool asks to stop over a target
- **navigation**: `StraightLineNavigation` plans the segments to drive — here a single one, forward for a configurable distance
- **automator**: Manages automation lifecycle (play/pause/stop)

A navigation only plans; it never drives.
`drive(navigation, path_driver, speed_limit=...)` is what runs it, pulling segments from the navigation and handing each to the `path_driver`.
The speed limit is passed in per run rather than stored on the navigation, so the caller decides how fast a job may go while a navigation may still ask for less on an individual segment.
To work the ground while driving, use `drive_and_work(...)` instead and pass an `Implement`.

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
