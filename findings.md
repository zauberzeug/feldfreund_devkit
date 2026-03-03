# Backward Driving Stops ~13mm Short

## Project Setup

./feldfreund_devkit
../rosys

## Problem

`test_straight_line_backward` fails: robot arrives at x=-0.987 instead of x=-1.0 (±0.005).
Forward driving (`test_stopping_at_different_distances`) achieves 0.0015m accuracy at the same distance and speed.

## Root Cause

In `rosys/driving/driver.py`, `can_drive_backwards` controls two unrelated things:

1. Whether the robot is **allowed** to drive backward (line 191)
2. Which **carrot algorithm** is used for the stopping condition (lines 178-181)

```python
if self.parameters.can_drive_backwards:
    can_move = carrot.move(hook, distance=self.parameters.carrot_distance)
else:
    can_move = carrot.move_by_foot(self.pose)
```

In `feldfreund_devkit/navigation/waypoint_navigation.py:145`:

```python
with self.driver.parameters.set(linear_speed_limit=linear_speed_limit, can_drive_backwards=segment.backward):
```

- **Forward** (`backward=False`) → `can_drive_backwards=False` → `move_by_foot`: tracks the robot's actual position on the spline, exits only when robot reaches t=1.0. Very precise.
- **Backward** (`backward=True`) → `can_drive_backwards=True` → `move`: advances the carrot in discrete steps (dt ≈ 0.015), exits when carrot crosses t≥1.0 — which happens before the robot reaches the endpoint.

## RoSys Driver Issue — Solution

The fix is a 3-line change in `Carrot.move()` in `rosys/rosys/driving/driver.py`.
The driving loop in `drive_spline()` (lines 178-183) and `move_by_foot()` were NOT changed.

### Before (`Carrot.move()` — original)

```python
def move(self, hook: Point, distance: float) -> bool:
    dt = 0.1 * distance / self._estimated_spline_length
    while hook.distance(self.offset_point) < distance:
        self.t += dt
        if self.t >= 1.0:
            return False        # BUG: exits as soon as carrot overshoots t=1.0
    return True                 # carrot is far enough ahead, keep driving
```

**Bug**: When `self.t` overshoots 1.0 in a discrete step, `move()` immediately returns `False` (stop driving). But the carrot is ahead of the robot — the robot hasn't reached the endpoint yet. This causes the ~13mm undershoot.

### After (`Carrot.move()` — fixed)

```python
def move(self, hook: Point, distance: float) -> bool:
    dt = 0.1 * distance / self._estimated_spline_length
    while hook.distance(self.offset_point) < distance:
        self.t += dt
        if self.t >= 1.0:
            self.t = 1.0       # FIX: clamp instead of returning early
            break               # FIX: stop advancing, but don't decide to stop driving yet
    return self.spline.closest_point(hook.x, hook.y) < 1.0  # FIX: use hook's actual position on spline
```

**Three changes:**

1. **Clamp `self.t = 1.0`** instead of returning `False` — prevents the carrot from overshooting the spline endpoint
2. **`break`** instead of `return False` — exits the inner advancement loop but lets the stopping decision happen below
3. **`return self.spline.closest_point(hook.x, hook.y) < 1.0`** — the stopping condition now checks the hook's (robot's) actual position on the spline, not the carrot's discrete parameter

### Why this works

The old code conflated two things:

- **Carrot advancement** (moving the target point along the spline)
- **Stopping decision** (should the robot stop driving?)

When the carrot overshot t=1.0, the old code immediately said "stop". But the carrot leads the robot — the robot was still ~13mm behind.

The fix separates these: the carrot still advances in steps, but the stopping decision is based on where the hook (attached to the robot) actually is on the spline. The robot only stops when `closest_point(hook) >= 1.0`, meaning the robot has actually reached the end.

### `git diff` of the fix

```diff
--- a/rosys/driving/driver.py
+++ b/rosys/driving/driver.py
@@ -274,8 +274,9 @@ class Carrot:
         while hook.distance(self.offset_point) < distance:
             self.t += dt
             if self.t >= 1.0:
-                return False
-        return True
+                self.t = 1.0
+                break
+        return self.spline.closest_point(hook.x, hook.y) < 1.0
```

### Failed approaches

1. **Swapping `move` and `move_by_foot` in the driving loop**: moved the ~13mm error from backward to forward driving (17 test regressions). The problem isn't which method is called — it's the stopping logic inside `move()`.
2. **Same swap again**: identical 17 regressions.

The correct fix keeps the driving loop unchanged and fixes the stopping condition inside `Carrot.move()` itself.

## Tests

Tests can be run with `uv run pytest`

- `test_stopping_at_different_distances`: forward driving test — PASSES
- `test_straight_line_backward`: backward driving test — PASSES
- All 39 tests pass
