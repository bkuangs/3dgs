"""Load COLMAP sparse output into PyTorch-friendly tensors.

Covers three things:
  1. Convert COLMAP world-to-camera (w2c) poses into torch tensors.
  2. Load one image and its matching camera pose.
  3. Verify: print camera / image / point counts + a projected-point sanity check.

COLMAP conventions used here:
  * images.txt stores the *world-to-camera* transform as a quaternion (QW QX QY QZ)
    plus translation (TX TY TZ). A world point X_w maps to camera space with
        X_c = R @ X_w + t
  * cameras.txt (SIMPLE_RADIAL) params are [f, cx, cy, k1]; fx == fy == f.
  * A pixel is  u = f * X_c.x / X_c.z + cx,  v = f * X_c.y / X_c.z + cy
    (radial distortion k1 ignored for the sanity check; it is tiny here).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
@dataclass
class Camera:
    camera_id: int
    model: str
    width: int
    height: int
    params: torch.Tensor  # model-specific intrinsics, e.g. [f, cx, cy, k1]

    @property
    def K(self) -> torch.Tensor:
        """3x3 pinhole intrinsics (distortion dropped)."""
        f, cx, cy = self.params[0], self.params[1], self.params[2]
        K = torch.eye(3, dtype=torch.float64)
        K[0, 0] = f
        K[1, 1] = f
        K[0, 2] = cx
        K[1, 2] = cy
        return K


@dataclass
class Image:
    image_id: int
    camera_id: int
    name: str
    qvec: torch.Tensor          # (4,) world-to-camera quaternion [w, x, y, z]
    tvec: torch.Tensor          # (3,) world-to-camera translation
    xys: torch.Tensor           # (N, 2) 2D keypoints
    point3D_ids: torch.Tensor   # (N,) matching 3D point ids (-1 if none)

    @property
    def R(self) -> torch.Tensor:
        return quaternion_to_rotation(self.qvec)

    @property
    def world_to_camera(self) -> torch.Tensor:
        """4x4 w2c matrix:  X_c = T @ X_w  (homogeneous)."""
        T = torch.eye(4, dtype=torch.float64)
        T[:3, :3] = self.R
        T[:3, 3] = self.tvec
        return T

    @property
    def camera_to_world(self) -> torch.Tensor:
        return torch.inverse(self.world_to_camera)

    @property
    def camera_center(self) -> torch.Tensor:
        """Camera position in world coords:  C = -R^T @ t."""
        return -self.R.T @ self.tvec


def quaternion_to_rotation(q: torch.Tensor) -> torch.Tensor:
    """COLMAP quaternion [w, x, y, z] -> 3x3 rotation matrix."""
    w, x, y, z = q / torch.linalg.norm(q)
    return torch.tensor(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=torch.float64,
    )


def read_cameras(path: Path) -> dict[int, Camera]:
    cameras: dict[int, Camera] = {}
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        cam_id = int(parts[0])
        cameras[cam_id] = Camera(
            camera_id=cam_id,
            model=parts[1],
            width=int(parts[2]),
            height=int(parts[3]),
            params=torch.tensor([float(p) for p in parts[4:]], dtype=torch.float64),
        )
    return cameras


def read_images(path: Path) -> dict[int, Image]:
    images: dict[int, Image] = {}
    lines = [ln for ln in path.read_text().splitlines() if ln and not ln.startswith("#")]
    # Two lines per image: pose line, then 2D-point line.
    for i in range(0, len(lines), 2):
        h = lines[i].split()
        image_id = int(h[0])
        qvec = torch.tensor([float(v) for v in h[1:5]], dtype=torch.float64)  # w,x,y,z
        tvec = torch.tensor([float(v) for v in h[5:8]], dtype=torch.float64)
        camera_id = int(h[8])
        name = h[9]

        pts = lines[i + 1].split()
        xs = torch.tensor([float(v) for v in pts[0::3]], dtype=torch.float64)
        ys = torch.tensor([float(v) for v in pts[1::3]], dtype=torch.float64)
        ids = torch.tensor([int(v) for v in pts[2::3]], dtype=torch.long)

        images[image_id] = Image(
            image_id=image_id,
            camera_id=camera_id,
            name=name,
            qvec=qvec,
            tvec=tvec,
            xys=torch.stack([xs, ys], dim=1) if len(xs) else torch.empty(0, 2),
            point3D_ids=ids,
        )
    return images


def read_points3D(path: Path) -> dict[int, torch.Tensor]:
    """Return {point3D_id: xyz tensor}."""
    points: dict[int, torch.Tensor] = {}
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        p = line.split()
        points[int(p[0])] = torch.tensor([float(p[1]), float(p[2]), float(p[3])],
                                         dtype=torch.float64)
    return points


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------
def project(points_world: torch.Tensor, image: Image, camera: Camera) -> torch.Tensor:
    """Project (N,3) world points to (N,2) pixels through this image's w2c pose."""
    Xc = (image.R @ points_world.T).T + image.tvec           # world -> camera
    xy = Xc[:, :2] / Xc[:, 2:3]                               # perspective divide
    if camera.model == "SIMPLE_RADIAL":                      # apply k1 distortion
        f, cx, cy, k1 = camera.params
        r2 = (xy * xy).sum(dim=1, keepdim=True)
        xy = xy * (1 + k1 * r2)
    else:
        f, cx, cy = camera.params[0], camera.params[1], camera.params[2]
    return torch.stack([f * xy[:, 0] + cx, f * xy[:, 1] + cy], dim=1)


# ---------------------------------------------------------------------------
# Demo / verification
# ---------------------------------------------------------------------------
def main() -> None:
    root = Path(__file__).resolve().parents[3] / "data" / "colmap" / "sparse"
    images_dir = Path(__file__).resolve().parents[3] / "data" / "images" / "south-building" / "images"

    cameras = read_cameras(root / "cameras.txt")
    images = read_images(root / "images.txt")
    points3D = read_points3D(root / "points3D.txt")

    print(f"cameras: {len(cameras)}")
    print(f"images:  {len(images)}")
    print(f"points:  {len(points3D)}")

    # --- 2. Load one image + its matching pose -----------------------------
    first_id = sorted(images)[0]
    img = images[first_id]
    cam = cameras[img.camera_id]
    print(f"\nloaded image #{img.image_id}: {img.name} "
          f"(camera {cam.camera_id}, {cam.model}, {cam.width}x{cam.height})")
    print("world_to_camera (4x4):")
    print(img.world_to_camera)

    # Load the actual pixels if Pillow is available (optional).
    try:
        from PIL import Image as PILImage
        with PILImage.open(images_dir / img.name) as im:
            rgb = torch.from_numpy(_to_uint8_array(im))  # (H, W, 3) uint8
        print(f"image tensor: {tuple(rgb.shape)} dtype={rgb.dtype}")
    except Exception as e:  # noqa: BLE001 - image load is optional
        print(f"(skipped loading pixels: {e})")

    # --- 3. Projected-point sanity check -----------------------------------
    # Pick a keypoint in this image that has a known 3D point, reproject it,
    # and compare with the 2D pixel COLMAP recorded.
    valid = (img.point3D_ids >= 0) & torch.tensor(
        [int(pid) in points3D for pid in img.point3D_ids]
    )
    idx = int(torch.nonzero(valid)[0])
    pid = int(img.point3D_ids[idx])
    observed_uv = img.xys[idx]
    predicted_uv = project(points3D[pid].unsqueeze(0), img, cam)[0]
    err = torch.linalg.norm(predicted_uv - observed_uv).item()

    print("\nsanity check (reproject a tracked 3D point):")
    print(f"  point3D id : {pid}")
    print(f"  observed uv: {observed_uv.tolist()}")
    print(f"  projected  : {predicted_uv.tolist()}")
    print(f"  pixel error: {err:.3f} px  -> {'OK' if err < 2.0 else 'CHECK'}")


def _to_uint8_array(pil_img):
    import numpy as np
    return np.array(pil_img.convert("RGB"), dtype="uint8")


if __name__ == "__main__":
    main()
