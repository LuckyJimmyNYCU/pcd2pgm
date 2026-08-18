# CSF (Cloth Simulation Filter)

Vendored copy of https://github.com/jianboqi/CSF, used by `pcd2pgm` for
ground-relative point-cloud slicing.

- Licence: Apache License 2.0 (see `LICENSE.txt`)
- Upstream commit: see the note below

If you use this in published work, cite the original paper:

> Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G.
> An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation.
> Remote Sensing. 2016; 8(6):501.

## What is vendored

Only the library sources are kept. The upstream Python bindings, MATLAB
wrapper, demo application and build files are not included, since the ROS 2
package builds these sources directly through its own CMakeLists.txt.

The sources are unmodified.
Upstream commit: e0322a29162c1586a0eae15dc1dd647abce1d11d (2025-08-13)
