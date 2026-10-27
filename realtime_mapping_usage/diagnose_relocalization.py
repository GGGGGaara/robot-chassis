"""Read-only snapshot of live localization observations; publishes no messages."""
from pathlib import Path
import json
import time
import os

import lcm
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def main():
    output = Path(__file__).resolve().parent / "localization_diagnosis"
    output.mkdir(exist_ok=True)
    bus = lcm.LCM()
    latest = {}
    transforms = {}
    counts = {}

    def cloud(channel, data):
        latest[channel] = data
        counts[channel] = counts.get(channel, 0) + 1

    def tf(channel, data):
        for transform in TFMessage.lcm_decode(data).transforms:
            transforms[(transform.frame_id, transform.child_frame_id)] = transform.to_matrix()

    for channel in ("/local_map#sensor_msgs.PointCloud2", "/loaded_map#sensor_msgs.PointCloud2"):
        bus.subscribe(channel, cloud)
    bus.subscribe("/tf#tf2_msgs.TFMessage", tf)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        bus.handle_timeout(100)

    info = {"counts": counts, "transforms": {f"{a}->{b}": m.tolist() for (a, b), m in transforms.items()}}
    for channel, raw in latest.items():
        name = channel.split("#")[0].strip("/")
        (output / f"{name}.pc2.lcm").write_bytes(raw)
        msg = PointCloud2.lcm_decode(raw)
        info[name] = {"points": len(msg), "frame": msg.frame_id}

    key = ("odom", "map")
    local_raw = latest.get("/local_map#sensor_msgs.PointCloud2")
    if key in transforms and local_raw:
        premap=Path(os.environ.get('DIMOS_PREMAP',os.environ.get('TRACER_PREMAP',str(output.parent.parent/'reconstruction_20261017_113920/raytrace/global_map.pc2.lcm')))).expanduser()
        fixed = PointCloud2.lcm_decode(premap.read_bytes()).pointcloud
        live = PointCloud2.lcm_decode(local_raw).pointcloud
        placement = np.linalg.inv(transforms[key])
        result = o3d.pipelines.registration.evaluate_registration(live, fixed, 0.165, placement)
        info["current_match"] = {
            "fitness_at_0.165m": result.fitness,
            "rmse": result.inlier_rmse,
            "live_to_saved_euler_xyz_deg": Rotation.from_matrix(placement[:3, :3]).as_euler("xyz", degrees=True).tolist(),
            "live_to_saved_translation_m": placement[:3, 3].tolist(),
        }
    (output / "snapshot.json").write_text(json.dumps(info, indent=2))
    print(json.dumps(info, indent=2), flush=True)


if __name__ == "__main__":
    main()
