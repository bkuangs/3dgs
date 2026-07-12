## 3D Gaussian Splatting for Real-Time Radiance Field Rendering

**Goal:** Implement a minimal 3DGS pipeline  

**Stack:** OpenGL, C++

**Concepts:** Structure from motion, Gaussian splatting, differentiable loss

### Build

```sh
cmake -S cpp -B build
cmake --build build --target convert_colmap viewer validate_geom
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

`validate_geom` re-parses the COLMAP sparse model and reprojects every tracked
observation, reporting mean/min/max pixel error (a low mean confirms poses and
intrinsics are read correctly):

```sh
./build/validate_geom data/colmap/sparse
```

See [`docs/format.md`](docs/format.md) for the COLMAP conventions and the PLY
handoff format shared between the C++ renderer and the Python trainer.
