# Computer Vision Project

<div align="justify">
This project provides a minimal ROS 2 pipeline for generating a 3D reconstruction from a prerecorded dataset. All components of the reconstruction stack, point cloud generation, odometry, and TSDF fusion are already implemented.<br><br>

Your task is only to produce a metric depth map for each frame.<br><br>

The system will:
- Convert your depth map into a point cloud
- Fuse it with provided odometry
- Build a TSDF volume
- Export the final mesh

You implement just the depth-estimation step. Everything else is handled automatically.<br><br>

This assignment assesses your understanding of geometric vision and metric depth recovery from stereo or other cues. Keep the implementation clean, correct, and deterministic.
</div>

```shell
git clone --recursive https://github.com/omniinstrument/CV_project.git
```

```shell
bash scripts/start.sh
```

```shell
ros2 launch tsdf_saver saver.launch.py
```

```shell
/opt/venv/bin/python /home/$(whoami)/compute_metrics.py #--view
```