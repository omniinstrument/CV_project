# Computer Vision Project

<div align="justify">
This project provides a minimal ROS 2 pipeline for generating a 3D reconstruction from a prerecorded dataset. All components of the reconstruction stack, point cloud generation, odometry, and TSDF fusion are already implemented.<br><br>

Your task is only to produce a metric depth map for each frame.<br>

The system will:
- Convert your depth map into a point cloud
- Fuse it with provided odometry
- Build a TSDF volume
- Export the final mesh

You implement just the depth-estimation step. Everything else is handled automatically.<br>

This assignment assesses your understanding of geometric vision and metric depth recovery from stereo or other cues. Keep the implementation clean, correct, and deterministic.
</div>

<p align="center">
  <img src="assets/neural_depth_demo.gif" width="1000" style="object-fit:fill;">
</p>

## Dependencies
A Linux Ubuntu computer, or other equivalent. 

You only need to install [Docker](https://docs.docker.com/engine/install/ubuntu/) and follow [Linux Post-Installation](https://docs.docker.com/engine/install/linux-postinstall/) to get started.

## Getting Started
Clone the repo
```shell
git clone --recursive https://github.com/omniinstrument/CV_project.git
```
Start the docker enviroment
```shell
bash scripts/start.sh
```
> This will automatically run a multi-stage docker container creation, pull all the datasets from Hugging Face too and build the ROS 2 workspace.

> Dataset download will occur only once.

## Demo
The ROS 2 bag file already has metric depth topic from the camera, you are trying to recreate or implement your own. But this is a good start and demo.
```shell
ros2 launch tsdf_saver saver.launch.py
```
At the end the launch file should automatically save a mesh(.stl) and stop/close the system after 20 secs.

> The generated mesh is saved in the [output](output) folder.

You can use RVIZ2 to visualize the results.

<p align="center">
  <img src="assets/zed_demo.gif" width="1000" style="object-fit:fill;">
</p>

### Comparing the Mesh against the Ground Truth
We are providing the ground truth mesh, and a basic code to compute the error/metrics. Use that to iterate upon your solution.
```shell
/opt/venv/bin/python /home/$(whoami)/compute_metrics.py #--view
```