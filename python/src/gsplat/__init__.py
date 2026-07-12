"""gsplat: Python trainer for the 3D Gaussian Splatting pipeline.

The C++ side (cpp/) owns real-time rendering. This package owns training and
writes the trained scene to a PLY that the C++ viewer consumes. The two sides
meet only at file-format boundaries (COLMAP input, PLY handoff).
"""
