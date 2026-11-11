import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('packing_result',Path(__file__).resolve().parents[1]/'blender/mesh_packer/result.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
IDENTITY = [1,0,0,0,0,1,0,0,0,0,1,0]


class Result(unittest.TestCase):
    def setUp(self):
        self.data = {'schema_version':1,'width':2,'length':3,'height':4,'gap':.1,'bin_count':2,
                     'bins':[{'id':0,'display_offset':[0,0,0]},{'id':1,'display_offset':[2.2,0,0]}],
                     'placements':[{'input_index':1,'bin':1,'matrix':IDENTITY[:]},
                                   {'input_index':0,'bin':0,'matrix':IDENTITY[:]}]}

    def read(self):
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'result.json';file.write_text(json.dumps(self.data))
            return MODULE.load_result(file,2,(2,3,4),.1)

    def test_reorder_and_display_offsets(self):
        placements,offsets=self.read()
        self.assertEqual(placements[0],(0,IDENTITY))
        self.assertEqual(placements[1][0],1)
        self.assertEqual(placements[1][1][3],2.2)
        self.assertEqual(len(offsets),2)

    def test_reject_malformed_or_incomplete_results(self):
        changes = [lambda d:d.update(schema_version=2),lambda d:d.update(width=20),
                   lambda d:d.update(gap=.2),lambda d:d.update(bin_count=0),
                   lambda d:d['placements'].pop(),lambda d:d['placements'][0].update(input_index=0),
                   lambda d:d['placements'][0].update(bin=3),lambda d:d['placements'][0].update(matrix=[0]*11),
                   lambda d:d['placements'][0]['matrix'].__setitem__(0,float('nan')),
                   lambda d:d['bins'][0].update(id=1),lambda d:d['bins'][0].update(display_offset=[0,0]),
                   lambda d:d['placements'][0].update(input_index=True)]
        original=copy.deepcopy(self.data)
        for change in changes:
            self.data=copy.deepcopy(original);change(self.data)
            with self.assertRaises(ValueError): self.read()


if __name__ == '__main__': unittest.main()
