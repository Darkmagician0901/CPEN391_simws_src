# CPEN 391 Milestone 1: AEB and Wall Following

**Demo video:** TODO (unlisted YouTube link)

## Package layout

```
src/milestone1/
├── config/params.yaml        # global parameter file for every node
├── launch/milestone1_py.py   # launches dist_finder, pid, safety_node
└── milestone1/
    ├── dist_finder.py        # LiDAR -> wall distance error
    ├── pid.py                # error -> steering + speed (drive request)
    └── safety_node.py        # TTC-based AEB, the only publisher on /drive
```

## Build and run

```bash
colcon build --packages-select milestone1
source install/setup.bash
# start the simulator first (see Milestone 1 part 1), then:
ros2 launch milestone1 milestone1_py.py
```

## System diagram

TODO: rqt_graph screenshot

```
/scan ──► dist_finder ──► /wall_error ──► pid ──► /drive_request ──┐
/scan, /ego_racecar/odom ─────────────────────────────────► safety_node ──► /drive
```

## Algorithms

### Safety node (AEB)
TODO

### Wall following (dist_finder + pid)
TODO

## Testing strategy
TODO
