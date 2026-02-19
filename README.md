# This repository

touch .gitignore

 git init -b main

 git add .

 git commit -m "Initial commit of ROS 2 workspace"

 """ git remote add origin git@github.com:gauravkhanduri/aeronerve_workspace.git """

git push -u origin main

## github actions setup

mkdir -p .github/workflows
touch .github/workflows/ros_ci.yml
Paste this into ros_ci.yml. This script uses the ros-tooling action, which is the community standard for building ROS 2 projects in the cloud.

Push the workflow: git add .github && git commit -m "Add CI pipeline" && git push

## Run Docker

## Build the image (from workspace root)

cd /home/grv22/aero_ws
docker build -t odom_vision:test .

## Check if it built successfully

docker images | grep odom_vision

## Launch a shell inside the container

docker run -it --rm odom_vision:test bash

## Inside container — verify the package is there

ros2 pkg list | grep odom_vision
