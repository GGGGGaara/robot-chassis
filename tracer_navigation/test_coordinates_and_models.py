"""Geometry, target safety, metre units and model export regressions; no hardware."""
import argparse
import json
import math
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from common import settings,lcm_types,ROOT
from coordinate_geometry import relative_goal,distance_metrics,validate_anchor,motion_metrics
from distance_audit import compare_measurement,offline_audit
from mission import resolve_target,current_map_sha
from model_recorder import Recorder,registered_points,write_ply


def cloud(points,stamp=None):
    from dimos_lcm.sensor_msgs.PointField import PointField
    _,Cloud,_=lcm_types(settings());m=Cloud();stamp=time.time() if stamp is None else stamp
    m.header.frame_id='odom';m.header.stamp.sec=int(stamp);m.header.stamp.nsec=int((stamp%1)*1e9)
    m.width=len(points);m.height=1;m.point_step=12;m.row_step=12*len(points);m.is_dense=True;m.is_bigendian=False
    m.fields=[]
    for i,key in enumerate('xyz'):
        f=PointField();f.name=key;f.offset=i*4;f.datatype=7;f.count=1;m.fields.append(f)
    m.data=np.asarray(points,dtype='<f4').tobytes();m.fields_length=3;m.data_length=len(m.data);return m


class CoordinateTests(unittest.TestCase):
    def test_body_axes_rotate(self):
        np.testing.assert_allclose(relative_goal([3,4,math.pi/2],[1,2]),[1,5])

    def test_no_threefold_scale(self):
        for yaw in np.linspace(-math.pi,math.pi,21):
            goal=relative_goal([2,-5,float(yaw)],[1,0])
            self.assertAlmostEqual(math.dist(goal,[2,-5]),1.)

    def test_detour_is_not_straight_distance(self):
        m=distance_metrics([0,0],[1,0],[[0,0],[0,1],[1,1],[1,0]])
        self.assertEqual(m['planned_length_m'],3.);self.assertEqual(m['straight_distance_m'],1.)

    def test_partial_motion_separate_from_planned_length(self):
        trace=[{'elapsed_s':i*.1,'pose':[i*.01,0,0],'wheel':[.1,0]} for i in range(101)]
        m=motion_metrics(trace)
        self.assertAlmostEqual(m['wheel_travelled_m'],1.)
        self.assertAlmostEqual(m['lio_travelled_m'],1.)

    def test_nonfinite_rejected(self):
        for offset in ([float('nan'),0],[0,float('inf')],[1]):
            with self.assertRaises(ValueError):relative_goal([0,0,0],offset)

    def test_moved_anchor_rejected(self):
        with self.assertRaises(ValueError):validate_anchor([.2,0,0],[0,0,0])
        with self.assertRaises(ValueError):validate_anchor([0,0,.2],[0,0,0])
        validate_anchor([0,0,2*math.pi],[0,0,0])

    def test_target_file_frame_hash_and_anchor(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'goal.json'
            data={'frame':'map','units':'m','goal':[1,2],'map_sha256':current_map_sha(),
                  'require_anchor':True,'anchor_pose':[0,0,0]}
            args=argparse.Namespace(relative=None,xy=None,target_file=path,execute=True)
            path.write_text(json.dumps(data));self.assertEqual(resolve_target(args,[0,0,0])[0],[1,2])
            with self.assertRaises(ValueError):resolve_target(args,[1,0,0])
            for key,value in (('frame','odom'),('units','cm'),('map_sha256','wrong')):
                bad={**data,key:value};path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):resolve_target(args,[0,0,0])

    def test_measured_threefold_scale_not_accepted(self):
        self.assertFalse(compare_measurement(3.,3.,1.)['physical_scale_verified'])
        self.assertTrue(compare_measurement(1.02,1.04,1.)['physical_scale_verified'])
        self.assertFalse(compare_measurement(1.,3.,1.)['physical_scale_verified'])

    def test_existing_map_units_audit(self):
        report=offline_audit();self.assertTrue(report['passed'])
        self.assertFalse(report['physical_scale_verified'])


class ModelTests(unittest.TestCase):
    def test_frame_registration_and_ply_metres(self):
        fix=np.eye(4);fix[0,3]=5
        xyz=registered_points(cloud([[6,0,0],[7,0,0]]),fix)
        np.testing.assert_allclose(xyz,[[1,0,0],[2,0,0]])
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'model.ply';write_ply(path,xyz)
            raw=path.read_bytes();payload=raw.split(b'end_header\n')[1]
            np.testing.assert_allclose(np.frombuffer(payload,dtype='<f4').reshape(-1,3),xyz)

    def test_wrong_frame_rejected(self):
        m=cloud([[1,2,3]]);m.header.frame_id='mid360_link'
        with self.assertRaises(ValueError):registered_points(m,np.eye(4))

    def test_snapshot_commit_and_retention(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);r=Recorder(output,10)
            r.fix=(np.eye(4),time.time(),time.monotonic())
            unrelated=output/'model_user_files';unrelated.mkdir();(unrelated/'keep.txt').write_text('keep')
            for i in range(3):
                m=cloud([[1+i,0,0]],time.time()+i*.01)
                r.cloud('',m.lcm_encode());r.save()
            pointer=json.loads((output/'latest.json').read_text())
            self.assertTrue((output/pointer['snapshot']/'model_map.ply').exists())
            self.assertEqual(len(r.saved),2);self.assertTrue((unrelated/'keep.txt').exists())

    def test_stale_model_cannot_replace_good_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            r=Recorder(Path(folder),10);r.fix=(np.eye(4),time.time(),time.monotonic())
            r.cloud('',cloud([[0,0,0]],time.time()-10).lcm_encode());self.assertIsNone(r.latest)
            r.fix=(np.eye(4),time.time()-10,time.monotonic()-10)
            r.cloud('',cloud([[0,0,0]]).lcm_encode());self.assertIsNone(r.latest)


if __name__=='__main__':
    lcm_types(settings())
    suite=unittest.defaultTestLoader.loadTestsFromModule(__import__(__name__))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    (ROOT/'results/coordinate_mapping_20261023/unit_tests.json').write_text(json.dumps(
        {'offline_only':True,'tests':result.testsRun,'passed':result.wasSuccessful(),
         'failures':[str(v) for v in result.failures+result.errors]},indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
