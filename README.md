## 3D Gaussian Splatting for Real-Time Radiance Field Rendering

**Goal:** Implement a minimal 3DGS pipeline  

**Stack:** OpenGL, C++

**Concepts:** Structure from motion, Gaussian splatting, differentiable loss

### Build

```sh
cmake -S . -B build
cmake --build build --target convert_colmap viewer
```

`convert_colmap` reads COLMAP text output and writes an initial ASCII PLY:

```sh
./build/convert_colmap data/colmap/sparse data/colmap/gaussians_init.ply
```

`viewer` renders an ASCII PLY as RGB `GL_POINTS`. Pass the COLMAP sparse
directory as the second argument to draw camera frustums:

```sh
./build/viewer data/colmap/gaussians_init.ply data/colmap/sparse
```

Use `--check` to validate PLY and sparse-pose loading without opening a window:

```sh
./build/viewer --check data/colmap/gaussians_init.ply data/colmap/sparse
```
