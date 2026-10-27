"""Regressions for moving the project to another teammate's filesystem."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'tracer_navigation'))
from common import settings,ROOT,driver_fingerprint

class Portability(unittest.TestCase):
    def test_default_map_is_relative_to_checkout(self):
        with patch.dict(os.environ,{},clear=True):
            self.assertEqual(Path(settings()['navigation']['static_map_lcm']),root/'reconstruction_20261017_113920/raytrace/global_costmap.lcm')
    def test_teammate_paths_override_without_rewriting_robot_yaml(self):
        with tempfile.TemporaryDirectory(prefix='team paths ') as tmp:
            override={'TRACER_STATIC_MAP_LCM':str(Path(tmp)/'map.lcm'),'TRACER_LCM_TYPES_DIR':str(Path(tmp)/'schema')}
            with patch.dict(os.environ,override):
                nav=settings()['navigation']
                self.assertEqual(nav['static_map_lcm'],override['TRACER_STATIC_MAP_LCM'])
                self.assertEqual(nav['lcm_types_dir'],override['TRACER_LCM_TYPES_DIR'])
                self.assertFalse(settings()['mount']['verified'])
    def test_driver_hash_uses_selected_runtime_not_other_users_install(self):
        import hashlib
        with tempfile.TemporaryDirectory() as tmp:
            binary=Path(tmp)/'ws/install/tracer_navigation_driver/lib/tracer_navigation_driver/tracer_safe_base'
            binary.parent.mkdir(parents=True);binary.write_bytes(b'test-only-not-executable')
            with patch.dict(os.environ,{'TRACER_RUNTIME_ROOT':tmp}):self.assertEqual(driver_fingerprint(),hashlib.sha256(binary.read_bytes()).hexdigest())
    def test_demo_rejects_real_domain_before_node_creation(self):
        from demo_visualization import Demo
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_LOCALHOST_ONLY':'1'}):
            with self.assertRaisesRegex(RuntimeError,'isolated'):Demo()

if __name__=='__main__':unittest.main()
