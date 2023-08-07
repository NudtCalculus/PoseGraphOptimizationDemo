This project is based on eigen, ceres and sophus.

Red line --- Groudtruth pose of A.

Yellow line --- Pose with noise of A.

Purple line --- Pose after graph optimization of A.

Blue line --- Groudtruth pose of B.

Cyan line --- Pose with noise of B.

White line --- Pose after graph optimization of B.

You can run `scripts/trajectory_visualization.py` and `rviz` to visualize the poses.

## visualization results

<div align=center>    <img src = "./data/1.png"/>
</div>

<img src = "./data/3.png" />

<img src = "./data/2.png" />

## evo evaluation results

### pose with noise

- APE

  <div align=center>    <img src = "./data/4.png"/> </div>

  <div align=center>    <img src = "./data/5.png"/> </div>

- RPE

  <div align=center>    <img src = "./data/6.png"/> </div>

<div align=center>    <img src = "./data/7.png"/> </div>

### pose after optimization

- APE

  <div align=center>    <img src = "./data/8.png"/> </div>

  <div align=center>    <img src = "./data/9.png"/> </div>

- RPE

  <div align=center>    <img src = "./data/10.png"/> </div>

  <div align=center>    <img src = "./data/11.png"/> </div>
