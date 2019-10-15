# HPS-3D160-PCL

HPS-3D160 Solid-State LiDAR (Hypersen Technologies Co., Ltd.) and Point Cloud Library (PCL) examples


## Dependencies
* [Point Cloud Library](http://pointclouds.org/) (PCL)
* HPS-3D160 Solid-State LiDAR [dependencies](https://github.com/hypersen/HPS3D_SDK)

## Examples
* **hps3d-pcl-visualiser** real-time visualisation of the 3D point cloud for all the cameras connected
* **hps3d-pcl-save-heaps-clouds** saves heaps of 3D point clouds in a specific folder
* **hps3d-pcl-save-clouds** saves all the 3D point clouds in a specific folder. The program requires a path to save the files, the number of the scene, and the timestamp
* **read-visualise-cloud** reads a .pcd file and visualises the XYZ point cloud (no colour). The program requires a .pcd file, some examples are provided.
* ...


## How to Run
The following steps download, compile, and execute the **hps3d-pcl-visualiser** example
* `git clone  https://github.com/FelipMarti/HPS3D-160-PCL`
* `cd HPS3D-160-PCL`
* `mkdir build`
* `cd build`
* `cmake ..`
* `make`
* `sudo ./hps3d-pcl-visualiser`

