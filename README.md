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

# appendix
You can see which mcap files have valid trajectory information by checking ```/SSH/outputs/log_rosbag2_2025_09_30_12_52_32.txt```

**mcap files which have 40 ~ 70 ekf_x, ekf_y range are valid.**

for example,
- rosbag2_2025_09_30_12_52_32_11.mcap is **invalid**
- rosbag2_2025_09_30_12_52_32_11.mcap is  **valid**


<img width="1306" height="1261" alt="Image" src="https://github.com/user-attachments/assets/2e3cc3c1-da84-4edd-a711-95236a921668" />
