# 0
build autoware environment ```./docker_run.sh dev cpu```

download ```/SSH``` to the environment

# 1
copy these files to your ```/SSH``` directory
-  open_mcap.py
-  master_exec.bash
-  occupancy_grid_map.yaml (for map drawing)
-  occupancy_grid_map.pgm (for map drawing)

# 2
```chmod +x master_exec.bash```

```./master_exec.bash```

# 3
```cd /SSH/rosbag2_2025_09_30_12_52_32```
* This is the only directory which have valid trajectory data.



```python3 ../open_mcap.py rosbag2_2025_09_30_12_52_32_13.mcap --overwrite```

```python3 ../open_mcap.py rosbag2_2025_09_30_12_52_32_14.mcap --overwrite```

```python3 ../open_mcap.py rosbag2_2025_09_30_12_52_32_15.mcap --overwrite```

...
