"""Reference import is auxiliary; it must preserve unknown areas and metre geometry."""
from pathlib import Path
import sys
import unittest
import subprocess
import tempfile
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'tools'))
from import_floor_map import build_reference


class FloorImport(unittest.TestCase):
    def cloud(self):
        x,y=np.meshgrid(np.arange(-1,1,.1),np.arange(-1,1,.1))
        floor=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,2.)])
        return np.vstack([floor,[.05,.05,2.5]])

    def test_explicit_floor_elevation_and_obstacle(self):
        points,grid,origin=build_reference(self.cloud(),2.,.1,0.)
        self.assertAlmostEqual(float(points[0,2]),0.)
        q=np.floor((np.array([.05,.05])-origin)/.1).astype(int)
        self.assertEqual(grid[q[1],q[0]],100)
        self.assertTrue((grid==-1).any())

    def test_wrong_floor_elevation_rejected(self):
        with self.assertRaisesRegex(ValueError,'ground'):build_reference(self.cloud(),0.)

    def test_absurd_scale_rejected(self):
        with self.assertRaises(ValueError):build_reference(self.cloud()*10000,20000,.1)

    def test_cli_writes_readable_reference_assets_without_touching_original(self):
        import json
        from common import settings,lcm_types,cloud_xyz
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'points.npy';np.save(source,self.cloud());original=source.read_bytes()
            output=Path(tmp)/'floor'
            result=subprocess.run([sys.executable,str(Path(__file__).resolve().parent.parent/'tools/import_floor_map.py'),
                str(source),'--output-dir',str(output),'--ground-z','2'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            _,Cloud,Grid=lcm_types(settings())
            cloud=Cloud.lcm_decode((output/'raytrace/global_map.pc2.lcm').read_bytes())
            grid=Grid.lcm_decode((output/'raytrace/global_costmap.lcm').read_bytes())
            self.assertEqual(cloud.header.frame_id,'map');self.assertAlmostEqual(float(cloud_xyz(cloud)[0,2]),0.)
            self.assertEqual(grid.info.width*grid.info.height,len(grid.data))
            self.assertTrue(json.loads((output/'floor_reference.json').read_text())['reference_only_not_live_clearance'])
            self.assertEqual(source.read_bytes(),original)
            repeat=subprocess.run([sys.executable,str(Path(__file__).resolve().parent.parent/'tools/import_floor_map.py'),
                str(source),'--output-dir',str(output),'--ground-z','2'],capture_output=True)
            self.assertNotEqual(repeat.returncode,0)


if __name__=='__main__':unittest.main()
