# Joint Trajectory Handler for ROS1

A simple recorder and player for joint trajectories.

## Features:
- Can save the joint positions of trajectories
- Compatibility with a wide variety of robots by using standard messages
- Filters joint states topic for the specified joints
- Playing trajectories can be aborted


## Usage
Adapt the configuration file in the `config` folder to your robot setup.

### Recording
In order to record launch `recorder.launch`. You can also specify a suffix for the filename to tell trajectories apart. E.g.:
```bash
mon launch joint_trajectory_handler recorder.launch filename_suffix:="foo"
```

A recording can be stopped by pressing `ctrl+c`. The trajectory is saved as `csv` file to `/tmp/<ISO 8601 timestamp>_trajectory[_filename_suffix].csv`.

The content looks likes this:
```
iiwa_joint_1,iiwa_joint_2,iiwa_joint_3,iiwa_joint_4,iiwa_joint_5,iiwa_joint_6,iiwa_joint_7,time
0.51293928467,0.174854565542,-0.1514450339982,-1.71573796814,1.0913939668,1.42528639176,-0.80127603464,0.0
0.5129400318,0.174854763517,-0.1514450986843,-1.7157379534,1.0913947968,1.42528698533,-0.80127616032,0.05
0.51285397202,0.174870621677,-0.1514381157456,-1.71572217215,1.0914091915,1.42531314796,-0.80129072156,0.1
```
The last column is the time that passed since the recording started.

### Playing
To play trajectories lauch `player.launch`. You can specify a folder with `folder:=/tmp` or directly a trajectory file with `trajectory_file:=file.csv`. For example:

```bash
mon launch joint_trajectory_handler player.launch trajectory_file:=2022-02-02T14:55:13.888918_trajectory
```
Again, `ctrl+c` aborts the execution.

### Specifying a folder with the launch file

If the player is started with the launch file and a folder is specfied it lists the files in the folder. However since the launch tools do not allow interaction it exits. A file can be chosen and passed to the launch file with `trajectory_file`.

### Specifying a folder then starting directly
**Note:** The yaml configuration is expected to be already loaded.

One can run it in the appropriate namespace and the folder specified in the parameter server like this:
```bash
export ROS_NAMESPACE=/joint_trajectory_handler
rosparam set /joint_trajectory_handler/player/folder /tmp
rosrun joint_trajectory_handler player.py namespace:=joint_trajectory_player
```

A list of files is returned and the user can choose one to play:
```
The folder contains 3 files:
0: test.csv
1: 2022-02-02T14:55:13.888918_trajectory.csv
2: 2022-02-02T15:57:31.537676_trajectory.csv
```