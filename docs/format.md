# Data formats & conventions

This project is split across two languages that **never share code — they share
file formats**:

- **C++ (`cpp/`)** owns real-time rendering (the `viewer`) and the COLMAP→PLY
  converter (`convert_colmap`).
- **Python (`python/`, package `gsplat`)** owns training and writes the trained
  scene back out as a PLY.

They meet at exactly two boundaries, documented here. Keep this file in sync with
`cpp/src/io/parse_colmap.*`, `cpp/src/tools/convert_colmap.cpp`, and
`python/src/gsplat/`.

---

## 1. Input: COLMAP sparse model

Read by both sides from a COLMAP text export directory (default
`data/colmap/sparse/`) containing `cameras.txt`, `images.txt`, `points3D.txt`.

### Coordinate & pose convention

Poses in `images.txt` are stored **world-to-camera**. A world point `X_w` maps
to camera space as:

```
X_c = R @ X_w + t
```

- `R` comes from the quaternion, stored in COLMAP order **`[QW, QX, QY, QZ]`**
  (w first). Normalize before use.
- `t` is `[TX, TY, TZ]`.
- Camera center in world coordinates is `C = -Rᵀ @ t`.
- Do **not** invert the stored pose — it is already world-to-camera.

### `cameras.txt`

`CAMERA_ID  MODEL  WIDTH  HEIGHT  PARAMS...`

Models handled by the reprojection code (`validate_geom`, `gsplat.colmap`):

| Model           | PARAMS                | Pixel projection (normalized `x=X_c/Z`, `y=Y_c/Z`) |
|-----------------|-----------------------|----------------------------------------------------|
| `SIMPLE_PINHOLE`| `f, cx, cy`           | `u = f·x + cx`, `v = f·y + cy`                      |
| `PINHOLE`       | `fx, fy, cx, cy`      | `u = fx·x + cx`, `v = fy·y + cy`                    |
| `SIMPLE_RADIAL` | `f, cx, cy, k1`       | `r² = x²+y²`; `u = f·(1+k1·r²)·x + cx` (same for v) |

The bundled `south-building` dataset uses `SIMPLE_RADIAL`. Ignoring `k1` inflates
reprojection error from ~0.5 px to ~2.7 px, so include it.

### `images.txt`

Two lines per image:

```
IMAGE_ID  QW QX QY QZ  TX TY TZ  CAMERA_ID  NAME
POINTS2D[]  as  (X, Y, POINT3D_ID)   # POINT3D_ID == -1 means no 3D match
```

### `points3D.txt`

```
POINT3D_ID  X Y Z  R G B  ERROR  TRACK[]  as  (IMAGE_ID, POINT2D_IDX)
```

`X Y Z` are world coordinates; `R G B` are 0–255.

### Validation

`validate_geom <sparse_dir>` re-parses the model and reprojects every tracked
observation, printing mean/min/max pixel error. `python -m gsplat.colmap` does
the single-point equivalent. On the bundled dataset the mean error is ~0.52 px.

---

## 2. Handoff: the scene PLY

The trained scene is exchanged as a PLY. Python writes it; the C++ `viewer`
reads it.

### Current format (as written by `convert_colmap`)

ASCII PLY, one vertex per Gaussian, properties **in this exact order**:

```
ply
format ascii 1.0
element vertex <N>
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property float opacity
property float scale_0
property float scale_1
property float scale_2
property float rot_0
property float rot_1
property float rot_2
property float rot_3
end_header
```

Field notes:

- `x y z` — Gaussian center, **world coordinates** (same frame as `points3D`).
- `red green blue` — `uchar` 0–255 base color. The `viewer` also accepts the
  short names `r g b`.
- `opacity` — scalar.
- `scale_0..2` — per-axis scale.
- `rot_0..3` — orientation quaternion, `[w, x, y, z]` (identity = `1 0 0 0`).

`convert_colmap` currently emits placeholder values for the appearance/geometry
fields (`opacity=1`, `scale=0.01`, `rot=identity`); training will replace them.

### Reader tolerance

The C++ reader looks up properties **by name**, not by fixed offset, so extra or
reordered properties are tolerated as long as the names above are present. The
Python writer (`gsplat.ply`) must emit at least these names.

### Planned evolution (not yet implemented)

To interoperate with the standard 3DGS ecosystem, the color channel is expected
to migrate from `red/green/blue` (uchar) to spherical-harmonic coefficients
`f_dc_0..2` and `f_rest_*` (float). When that lands, update this section and the
readers/writers on **both** sides together, and bump a documented format version.

---

## 3. Ownership summary

| Concern                     | Owner        | File(s)                                   |
|-----------------------------|--------------|-------------------------------------------|
| COLMAP parsing (C++)        | C++          | `cpp/src/io/parse_colmap.*`               |
| COLMAP parsing (Python)     | Python       | `python/src/gsplat/colmap.py`             |
| PLY writing (init)          | C++          | `cpp/src/tools/convert_colmap.cpp`        |
| PLY writing (trained)       | Python       | `python/src/gsplat/ply.py` (TODO)         |
| PLY reading / rendering     | C++          | `cpp/src/viewer/main.cpp`                 |
| Reprojection validation     | C++ / Python | `cpp/src/tools/validate_geom.cpp`, `gsplat.colmap` |

The two PLY writers are the single point of coupling — the `*_roundtrip`/`--check`
paths exist to catch drift. Change the format in one place and you must change it
in the other.
