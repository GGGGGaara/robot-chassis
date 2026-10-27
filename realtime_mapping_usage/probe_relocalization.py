"""Offline evaluation of a captured cloud. No sensor or control publication."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from dimos.mapping.relocalization.lidar.relocalize import LidarRelocalizer, MID360
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def main():
    folder = Path(__file__).resolve().parent / "localization_diagnosis"
    fixed = PointCloud2.lcm_decode((folder / "loaded_map.pc2.lcm").read_bytes()).pointcloud
    live = PointCloud2.lcm_decode((folder / "local_map.pc2.lcm").read_bytes()).pointcloud
    matcher = LidarRelocalizer(fixed, MID360)
    candidates = []
    for seed in range(4):
        o3d.utility.random.seed(seed)
        result = matcher.align(live)
        matrix = np.asarray(result.transformation)
        item = {"seed": seed, "fitness": result.fitness, "rmse": result.inlier_rmse,
                "euler_xyz_deg": Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True).tolist(),
                "transform": matrix.tolist()}
        candidates.append(item)
        print(json.dumps(item), flush=True)
    best = max(candidates, key=lambda c: c["fitness"])
    (folder / "candidates.json").write_text(json.dumps(candidates, indent=2))
    snapshot = json.loads((folder / "snapshot.json").read_text())
    current = np.linalg.inv(np.array(snapshot["transforms"]["odom->map"]))
    a = np.asarray(fixed.points)
    b = np.asarray(live.points)
    for name, placement in (("current", current), ("best", np.array(best["transform"]))):
        aligned = b @ placement[:3, :3].T + placement[:3, 3]
        # A horizontal slice suppresses floors/ceilings and exposes wall alignment.
        select_a = a[(a[:, 2] > -0.4) & (a[:, 2] < 0.8)]
        select_b = aligned[(aligned[:, 2] > -0.4) & (aligned[:, 2] < 0.8)]
        fig, ax = plt.subplots(figsize=(11, 6), dpi=140)
        ax.scatter(select_a[::2, 0], select_a[::2, 1], s=0.7, color="#777777", label="Saved map")
        ax.scatter(select_b[:, 0], select_b[:, 1], s=1, color="#ef4238", alpha=0.65, label="Live local map")
        ax.set_aspect("equal")
        ax.set_xlabel("map X (m)")
        ax.set_ylabel("map Y (m)")
        ax.set_title(f"{name} alignment (horizontal slice)")
        ax.legend()
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(folder / f"{name}_overlay.png")
        plt.close(fig)


if __name__ == "__main__":
    main()
