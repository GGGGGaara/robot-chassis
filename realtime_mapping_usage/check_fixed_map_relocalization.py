"""Read-only preflight. Does not construct/start modules or open the sensor SDK."""

import hashlib
import inspect
import os
from pathlib import Path
import sys

import numpy as np

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2, FastLio2Config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.mapping.relocalization.lidar.module import LidarConfig
from dimos.mapping.relocalization.lidar.relocalize import LidarRelocalizer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.get_all_blueprints import get_by_name


def main() -> None:
    mapping_enabled = '--mapping-enabled' in sys.argv[1:]
    config_args = [arg for arg in sys.argv[1:] if arg != '--mapping-enabled']
    blueprint = autoconnect(
        get_by_name("mid360-fastlio-ray-trace"),
        get_by_name("local-map-relocalization"),
    )
    from dimos.cli.commands.lifecycle import DEFAULT_CONFIG_PATH

    config_path = DEFAULT_CONFIG_PATH
    parsed = BlueprintConfigParser(blueprint).parse(
        config_args, config_path=config_path, environ=os.environ
    )
    g = parsed.global_config_values()
    if g["transport"] != "lcm" or g["replay"] or g["simulation"] or g["record"]:
        raise ValueError("Expected live LCM, no simulation/replay/recording")

    for cls, config_cls in ((FastLio2, FastLio2Config), (RayTracingVoxelMap, RayTracingVoxelMapConfig)):
        config = config_cls(**parsed.module_kwargs(cls.name)).model_dump()
        base = Path(inspect.getfile(cls)).resolve().parent
        cwd = Path(config["cwd"]) if config["cwd"] else Path.cwd()
        if not cwd.is_absolute():
            cwd = base / cwd
        executable = Path(config["executable"])
        if not executable.is_absolute():
            executable = cwd / executable
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise FileNotFoundError(f"Native executable missing: {executable}")
        print(f"OK executable: {executable.resolve()}", flush=True)

    reloc = parsed.module_kwargs("localmaprelocalization")
    reloc_config = LidarConfig(**reloc)
    mapper = parsed.module_kwargs("raytracingvoxelmap")
    if reloc["world_frame"] != "odom" or mapper["world_frame"] != "odom":
        raise ValueError("Both localization and local observation frames must be odom")
    expected_emit = 10 if mapping_enabled else 0
    if mapper["global_emit_every"] != expected_emit:
        raise ValueError(f"Expected global map publication interval {expected_emit}")
    print(f"OK global model publication: every {expected_emit} frames (0 means disabled)", flush=True)
    path = Path(reloc["map_file"])
    raw = path.read_bytes()
    cloud = PointCloud2.lcm_decode(raw)
    points = np.asarray(cloud.pointcloud.points)
    if not len(points) or not np.isfinite(points).all():
        raise ValueError("Premap is empty or contains invalid XYZ")
    print(f"OK premap: {path}", flush=True)
    print(f"OK points: {len(points)}, source frame: {cloud.frame_id}", flush=True)
    print(f"SHA256: {hashlib.sha256(raw).hexdigest()}", flush=True)
    print(f"Matching: min_points={reloc_config.min_local_points}, fitness_threshold={reloc_config.relocalize.fitness_threshold}, restarts={reloc_config.relocalize.ransac_restarts}", flush=True)
    print("Preparing premap features (read-only, no hardware start)...", flush=True)
    # Exercise the actual loader conversion and Open3D feature pipeline,
    # not only imports. The runtime creates its own prepared map afterward.
    prepared = LidarRelocalizer(cloud.pointcloud, reloc_config.relocalize)
    del prepared
    if hashlib.sha256(path.read_bytes()).digest() != hashlib.sha256(raw).digest():
        raise RuntimeError("Premap changed during check")
    print("CHECK PASSED: config, binaries, map decode and FPFH preparation.", flush=True)
    print("This check does not establish live relocalization success.", flush=True)


if __name__ == "__main__":
    main()
