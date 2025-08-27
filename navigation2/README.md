# Navigation2


## Terminal 1

Clone this project to the home directory of your machine and run the commands below

```
cd navigation2
```

In the ```nav2-rviz2-container``` file, update the ```USER``` variable to match your machine's username


```
sudo ./nav2-rviz2-container.sh

```

```
source opt/ros/humble/setup.bash
```

```
cd opt/ros/humble/share/nav2_bringup/maps/
```

```
ros2 launch nav2_bringup localization_launch.py map:=map_cam_odom_exp_170601.yaml
```

## Terminal 2


```
sudo docker exec -it ros-utilities bash
```

```
source opt/ros/humble/setup.bash
```

```
ros2 service call /reinitialize_global_localization std_srvs/Empty
```

```
rviz2
```

## Terminal 3

```
sudo docker exec -it ros-utilities bash
```

```
source opt/ros/humble/setup.bash
```

```
ros2 launch nav2_bringup navigation_launch.py
```




