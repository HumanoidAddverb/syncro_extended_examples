# Spline Teaching

Pick-and-place waypoint collection (freedrive) and spline-based trajectory
execution. Supports a **variable number of waypoints** between the named
phases.

## 1. Collect waypoints (freedrive)

```bash
./collect_pick_place_waypoints [output.csv]
```

The robot homes, then enters **freedrive mode**. Hand-guide it to each
position and press a key to mark it:

| Key | Meaning |
|-----|---------|
| `h` | Mark current pose as `home` |
| `w` | Mark current pose as an intermediate waypoint (`wp_N`) |
| `p` | Mark current pose as `pick` (only one allowed) |
| `l` | Mark current pose as `place` (only one allowed) |
| `q` | Finish and save |

**Typical order:**
```
h        # home
w w w    # any number of pre-pick waypoints
p        # pick
w w w    # any number of intermediate waypoints between pick and place
l        # place
w w w    # any number of post-place waypoints (return path)
q        # finish
```

The number of `w` marks in each section is up to you — the spline smooths
across all of them within the matching phase duration.

**CSV output:**
```
waypoint_name,duration,joint0,joint1,...
home,0.0,...
wp_1,0.0,...
wp_2,0.0,...
pick,0.0,...
wp_3,0.0,...
place,0.0,...
wp_4,0.0,...
```
The `duration` column exists for backwards compatibility but is ignored by
the executor — phase durations come from `include/execute_spline.h`.

## 2. Configure phase durations

Three durations control how long each major phase takes, defined in
`include/execute_spline.h`:

```cpp
struct SegmentDurations {
    double home_to_pick  = 3.0;   // seconds
    double pick_to_place = 6.0;
    double place_to_home = 3.0;
};
```

**How they're applied:**
1. The executor groups waypoints by role:
   - `home_to_pick` = `home` + all `wp_*` before `pick` + `pick`
   - `pick_to_place` = `pick` + all `wp_*` between `pick` and `place` + `place`
   - `place_to_home` = `place` + all `wp_*` after `place` + (back to `home`)
2. The total duration is split **evenly** across the sub-segments in the
   phase. With 4 sub-segments and `home_to_pick = 3.0 s`, each hop gets
   0.75 s.
3. Each waypoint gets a cumulative timestamp; the cubic spline interpolates
   the `(time, joints)` pairs into a smooth trajectory sampled at 500 Hz
   and driven via External Velocity Control.

Longer duration = slower motion through that phase. Sub-waypoints are always
spaced evenly in time within a phase.


## 3. Execute the spline

The main execution script is **`execute_pick_place_spline`**.

```bash
./execute_pick_place_spline pick_place_waypoints.csv
```

It loads the CSV, splits the waypoints into three phases by role, builds a
cubic spline per phase, and runs them under External Velocity Control. After
the first cycle, the spline samples are cached and replayed forward/reverse
in subsequent cycles.

## Files

| File | Purpose |
|---|---|
| `collect_pick_place_waypoints.cpp` | Freedrive waypoint collector (variable count) |
| `execute_pick_place_spline.cpp` | Main spline executor (External Velocity Control) |

