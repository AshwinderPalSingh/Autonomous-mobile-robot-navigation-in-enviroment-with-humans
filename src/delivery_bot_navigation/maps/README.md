# Maps

This directory holds the map built in **Phase 3** with slam_toolbox.

It is intentionally empty in version control until you generate one.

## Generate

```bash
# terminal 1
ros2 launch delivery_bot_gazebo sim.launch.py
# terminal 2
ros2 launch delivery_bot_navigation slam.launch.py
# terminal 3 -- drive every aisle, close the loop at least once
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When the map looks clean in RViz (fixed frame `map`, Map display on `/map`):

```bash
ros2 run nav2_map_server map_saver_cli \
    -f ~/social_nav_ws/src/delivery_bot_navigation/maps/warehouse
```

This writes `warehouse.pgm` + `warehouse.yaml` here. Rebuild the package so the
installed `share/` copy exists for `nav.launch.py`:

```bash
cd ~/social_nav_ws && colcon build --packages-select delivery_bot_navigation
```

`nav.launch.py` then loads `maps/warehouse.yaml` by default.
