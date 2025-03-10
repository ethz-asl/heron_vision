# heron_vision
Vision modules for the EU Heron Project

# Setup
## Set up a ROS workspace
Make a directory called `heron_ws` with a `src` subfolder.
Check out the following repos into the folder:
```
git clone git@github.com:ethz-asl/heron_vision.git
git clone git@github.com:RobotnikAutomation/heron_msgs.git
git clone git@github.com:RobotnikAutomation/robotnik_msgs.git
```

## Download the models
Please get the models from Slack and put them in the `heron_ws/src/heron_vision/models` folder.

## How to run the docker
```
cd heron_vision/docker
./run_docker.sh -b
```
Subsequent times you can just run:
```
./run_docker.sh
```

For rebinding the paths, `-w /path/to/the/heron_ws/folder` and `-d /path/to/the/data/folder` as input to `./run_docker.sh`. **Please use the full path, so `/home/$USER/folder` rather than `~/folder`**.

To run a second bash shell into the docker, use:
```
docker exec -it heron bash
```
If it can't find some ROS commands, use:
```
source /opt/ros/noetic/setup.bash
```

# Building the code
##
```
cd ~/heron_ws
catkin build
```

After building the code for the first time, you'll have to re-source the workspace:
```
source ~/.bashrc
```

# Running the code
## Starting ROS
There should always be a roscore running. Easiest is to start it in the background:
```
roscore &
```

## Running the dataset
To run the dataset, download the dataset from the #vision channel on slack.
```
cd ~/data/path/to/the/dataset/
rosbag play 2025-02-18-15-25-24.bag
```

## Running the pothole finding node
This node will just sit there and wait for someone to call a service request
```
rosrun heron_vision pothole_finder_node.py
```

## Running the testing node
This will subscribe to the topics in the ROS Bag and collect them to call the crack service.
```
rosrun heron_vision service_tester_node.py
```
