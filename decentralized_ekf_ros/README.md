## decentralized extended Kalman filter
shared code space for STTR with Kinnami

## OLD STEPS (will update soon)
## Step 1: Truth Node Package
Open a new terminal and source 
```
source ~/coopnav_ws/devel/setup.bash
```
Launch package 
```
roslaunch dekf_ros truth_node.launch
```
## Step 2: Visualize Truth Position
Open a new terminal and source 
```
source ~/coopnav_ws/devel/setup.bash
```
Call the topic you want to see the location of each drone
```
rostopic echo /truth_iris0
```
```
rostopic echo /truth_iris1
```
```
rostopic echo /truth_iris2
