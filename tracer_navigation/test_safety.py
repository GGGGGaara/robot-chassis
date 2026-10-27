import copy
import json
import math
import unittest
import numpy as np
from common import ROOT,CommandGate,settings,mount_matrix,filter_cloud,planar,ground_correction,near_stop_count,pulse_profile,config_fingerprint,software_fingerprint,hardware_fault_requires_rearm,stop_polygon
from commission import validate_evidence


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.config=settings();self.gate=CommandGate(self.config);self.gate.armed=True
        self.ages={k:0. for k in ('command','cloud','pose','feedback')}

    def test_every_timeout_stops(self):
        for key in ('command','cloud','pose'):
            ages=dict(self.ages);ages[key]=2
            self.assertEqual(self.gate.evaluate((.1,.1),ages)[:2],(0.,0.))
        c=copy.deepcopy(self.config);c['mount']['verified']=True;c['robot']['height_m']=1.2
        gate=CommandGate(c);gate.armed=True;ages=dict(self.ages);ages['feedback']=2
        self.assertEqual(gate.evaluate((.1,.1),ages,drive=True)[2],'feedback_timeout')

    def test_estop_disarm_and_calibration(self):
        self.gate.estop=True
        self.assertEqual(self.gate.evaluate((.1,0),self.ages)[2],'estop')
        self.gate.estop=False;self.gate.armed=False
        self.assertEqual(self.gate.evaluate((.1,0),self.ages)[2],'disarmed')
        self.gate.armed=True
        self.gate.config['mount']['verified']=False
        self.assertEqual(self.gate.evaluate((.1,0),self.ages,drive=True)[2],'calibration_unverified')

    def test_limits_reverse_nan_and_immediate_stop(self):
        for _ in range(100):v,w,_=self.gate.evaluate((10.,10.),self.ages)
        self.assertLessEqual(v,self.config['robot']['max_linear_mps']);self.assertLessEqual(w,self.config['robot']['max_angular_rps'])
        self.assertEqual(self.gate.evaluate((0.,0.),self.ages)[:2],(0.,0.))
        self.assertEqual(self.gate.evaluate((-.1,0.),self.ages)[:2],(0.,0.))
        self.assertEqual(self.gate.evaluate((float('nan'),0.),self.ages)[2],'invalid_command')

    def test_mount_floor_obstacle_and_self_filter(self):
        mount=mount_matrix(self.config)
        base=np.array([[2.,0.,0.],[2.,0.,.5],[0.,0.,.5]])
        sensor=(base-mount[:3,3])@mount[:3,:3]
        filtered=filter_cloud(sensor,self.config)
        self.assertEqual(len(filtered),2)
        self.assertTrue(np.allclose(filtered,base[:2],atol=1e-6))
        self.assertEqual(int((filtered[:,2]>=.03).sum()),1)

    def test_frame_composition_unmounts_before_planar_projection(self):
        mount=mount_matrix(self.config);base=np.eye(4);base[:2,3]=[4.,2.]
        angle=.6;base[:2,:2]=[[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]
        sensor=base@mount
        recovered=planar(sensor@np.linalg.inv(mount))
        self.assertTrue(np.allclose(recovered,base,atol=1e-9))

    def test_ground_refinement_is_small_and_flattens_without_free_yaw(self):
        a,b,c=.022,-.023,-.01
        correction=ground_correction(a,b,c)
        self.assertLess(np.linalg.norm(correction[:3,:3]-np.eye(3)),.06)
        points=np.array([[x,y,a*x+b*y+c] for x in (-2.,0.,2.) for y in (-2.,0.,2.)])
        transformed=points@correction[:3,:3].T+correction[:3,3]
        self.assertTrue(np.allclose(transformed[:,2],0.,atol=1e-9))

    def test_independent_near_stop_ignores_floor_and_outside_points(self):
        points=np.array([[.6,0,.3],[.45,.45,.3],[.76,0,.3],[.6,0,0],[0,0,1.3],[.6,.6,.3]])
        self.assertEqual(near_stop_count(points,self.config),2)

    def test_configured_stop_zone_boundary_and_shared_collision_geometry(self):
        import yaml
        c=copy.deepcopy(self.config);c['navigation']['near_stop_radius_m']=.70
        self.assertEqual(near_stop_count(np.array([[.6999,0,.2],[.7001,0,.2],[0,-.6999,.2],[0,-.7001,.2]]),c),2)
        # The actual right-rear returns near 0.62 m must remain protected.
        self.assertEqual(near_stop_count(np.array([[-.14,-.60,.9]]),c),1)
        polygon=stop_polygon(c)
        self.assertTrue(np.allclose(np.linalg.norm(polygon,axis=1),.70))
        generated=yaml.safe_load((ROOT/'generated/nav2.yaml').read_text())
        points=generated['collision_monitor']['ros__parameters']['PolygonStop']['points']
        self.assertTrue(np.allclose(points,stop_polygon(self.config).ravel()))

    def test_stop_zone_cannot_exclude_padded_chassis(self):
        for radius in (.5,float('nan'),float('inf'),-.7):
            c=copy.deepcopy(self.config);c['navigation']['near_stop_radius_m']=radius
            with self.assertRaises(ValueError):stop_polygon(c)

    def test_site_evidence_rejects_simulation_failed_stale_or_changed_tests(self):
        c=self.config;now=100000.;sha='driver';reports={}
        for direction in ('forward','left'):
            reports[direction]={'direction':direction,'test_platform':'physical_can0','passed':True,'pulse_completed':True,
                'movement_observed':True,'wheel_feedback_stopped':True,'config_sha256':config_fingerprint(c),
                'driver_sha256':sha,'timestamp':now-60,'software_sha256':software_fingerprint()}
        validate_evidence(c,sha,reports,now)
        for key,value in (('test_platform','simulation'),('passed',False),('timestamp',0),('driver_sha256','changed')):
            invalid=copy.deepcopy(reports);invalid['forward'][key]=value
            with self.assertRaises(ValueError):validate_evidence(c,sha,invalid,now)
        changed=copy.deepcopy(c);changed['mount']['pitch_deg']+=1
        with self.assertRaises(ValueError):validate_evidence(changed,sha,reports,now)

    def test_pulse_profiles_are_bounded_and_cannot_reverse(self):
        self.assertEqual(pulse_profile('forward'),(.03,0.))
        self.assertEqual(pulse_profile('left'),(0.,.1))
        self.assertEqual(pulse_profile('right'),(0.,-.1))

    def test_hardware_faults_require_rearm_but_obstacle_stop_may_resume(self):
        for reason in ('unhealthy_inputs','invalid_command','cloud_timeout','localization_timeout','feedback_timeout'):
            self.assertTrue(hardware_fault_requires_rearm(reason,True,True,10,20))
        self.assertFalse(hardware_fault_requires_rearm('command_timeout',True,True,10,20))
        self.assertTrue(hardware_fault_requires_rearm('command_timeout',True,True,21,20))
        self.assertFalse(hardware_fault_requires_rearm('near_obstacle',True,True,30,20))
        self.assertFalse(hardware_fault_requires_rearm('unhealthy_inputs',True,False,30,20))


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(SafetyTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    (ROOT/'test_safety_report.json').write_text(json.dumps({'tests':result.testsRun,'passed':result.wasSuccessful(),
        'failures':len(result.failures),'errors':len(result.errors)},indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
