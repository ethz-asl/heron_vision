FROM osrf/ros:noetic-desktop-full

# Install additional system dependencies

# update git and set to always point to https
RUN apt-get -qq update && apt-get install -y curl apt-utils
RUN apt-get -qq update && apt-get install -y git git-lfs

# get install tools
RUN apt-get -qq update && apt-get install -y python3-catkin-tools python3-vcstool python3-pip python-is-python3

# Install other system deps
RUN apt-get -qq update && apt-get install -y qtbase5-dev bash-completion

# pip install some stuff
RUN pip install torch

RUN pip install torchvision segmentation_models_pytorch scipy

# Set up bash
# Add sourcing of the repo to the ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo 'ROSBASH=/root/heron_ws/devel/setup.bash && [ -e "$ROSBASH" ] && source $ROSBASH' >> ~/.bashrc
