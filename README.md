# A tool to plot the map and path of the robot headlessly!
<img width="2523" height="2433" alt="image" src="https://github.com/user-attachments/assets/9b86d160-c188-4482-a436-49d8e039eef4" />

## Features: 
- With this ros2 package, teams can plot the map and the robot path without using tool like rviz2 or Foxglove, completely headlessly on the VM. 
- Based on `slam_toolbox` for mapping.
- An image is saved to the directory `saved_maps` every 20 seconds and upon stopping the launch file a final map is also saved. This ensures that the map is saved in case the run fails in midway.
- The maps are saved as standard jpegs, so they can be transferred locally very easily and modified as per usecase.
- Optionally if the poses of the anamolies are saved in a file called `data.csv` template included below, the anamolies are marked in the map as blue points.

## Setup:

#### Clone the repo on your machine:
```bash
git clone https://github.com/DoYouEvenSheesh/criss-map-plot.git
```
#### Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
#### Build and source the package:
```bash
colcon build --symlink-install && source install/setup.bash
```
## Launch:
#### Launch slam_toolbox:
```bash
    ros2 launch slam_toolbox online_async_launch.py
    # (make sure the parameters are configured correctly with the robot namespace)
```
#### Launch map plotting tool:
```bash
ros2 launch criss-map-plot plot_map.launch.py
```

## csv template
```csv
x_coordinate,y_coordinate
x1,y1
x2,y2
```
Now, every 20 seconds a new jpeg will be saved to `saved_maps` directory and upon performing Ctrl+C a final_map will be saved. 
