"""Sanity checks of the independent oracle, including fully nested surfaces."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import trimesh
from verify import verify


class Oracle(unittest.TestCase):
    def test_collision_distance_containment_and_bounds(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            outer=trimesh.creation.box(extents=[1,1,1]);outer.export(root/'cube.obj')
            inner=trimesh.creation.box(extents=[.1,.1,.1]);inner.export(root/'small.obj')
            def check(mesh,shift,gap=.1):
                a=np.eye(4)[:3];a[:,3]=[1,1,1]
                b=np.eye(4)[:3];b[:,3]=np.array(shift)+[1,1,1]
                manifest=dict(width=3,length=3,height=3,gap=gap,bin_count=1,placements=[
                    dict(input_index=0,input=str(root/'cube.obj'),bin=0,matrix=a.flatten().tolist()),
                    dict(input_index=1,input=str(root/mesh),bin=0,matrix=b.flatten().tolist())])
                path=root/'result.json';path.write_text(json.dumps(manifest));return verify(path)
            self.assertTrue(check('cube.obj',[1.1,0,0])['valid'])
            self.assertFalse(check('cube.obj',[1.099,0,0])['valid'])
            self.assertFalse(check('cube.obj',[.9,0,0])['valid'])
            nested=check('small.obj',[0,0,0])
            self.assertTrue(any('containment' in e for e in nested['errors']),nested)
            self.assertFalse(check('cube.obj',[3,0,0])['valid'])

if __name__=='__main__':unittest.main()
