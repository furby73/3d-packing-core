"""End-to-end contract checks with independently measured output geometry."""
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

PACKER = str(Path(sys.argv.pop(1)).resolve())
VERTICES = list(itertools.product((10.0, 11.0), (20.0, 21.0), (30.0, 31.0)))
# itertools order: z is the fastest-moving coordinate.
FACES = ((0,1,3),(0,3,2),(4,6,7),(4,7,5),(0,4,5),(0,5,1),
         (2,3,7),(2,7,6),(0,2,6),(0,6,4),(1,5,7),(1,7,3))


def cube_file(path):
    path.write_text(''.join(f'v {x} {y} {z}\n' for x,y,z in VERTICES) +
                    ''.join('f '+' '.join(str(i+1) for i in f)+'\n' for f in FACES))


def transform(matrix, point):
    return [sum(matrix[r*4+c]*point[c] for c in range(3))+matrix[r*4+3] for r in range(3)]


class CLI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.model = self.root / ('cube with spaces.obj' if os.name == 'nt' else 'cube тест "quoted".obj')
        cube_file(self.model)
        self.original = hashlib.sha256(self.model.read_bytes()).digest()

    def tearDown(self):
        self.assertEqual(hashlib.sha256(self.model.read_bytes()).digest(), self.original)
        self.temp.cleanup()

    def run_packer(self, *args, good=True):
        result = subprocess.run([PACKER, *map(str, args)], cwd=self.root, text=True, capture_output=True, timeout=20)
        self.assertEqual(result.returncode == 0, good, result.stdout + result.stderr)
        return result

    def verify(self, manifest_path):
        data = json.loads(manifest_path.read_text())
        dims = (data['width'],data['length'],data['height'])
        self.assertEqual(data['schema_version'], 1)
        self.assertEqual(len(data['bins']), data['bin_count'])
        bounds = []
        for item in data['placements']:
            m = item['matrix']
            self.assertEqual(len(m),12)
            for r in range(3):
                for s in range(3):
                    self.assertAlmostEqual(sum(m[r*4+c]*m[s*4+c] for c in range(3)),float(r==s),places=9)
            determinant = (m[0]*(m[5]*m[10]-m[6]*m[9]) - m[1]*(m[4]*m[10]-m[6]*m[8]) + m[2]*(m[4]*m[9]-m[5]*m[8]))
            self.assertAlmostEqual(determinant,1,places=9)
            points = [transform(m,v) for v in VERTICES]
            lo = [min(p[a] for p in points) for a in range(3)]
            hi = [max(p[a] for p in points) for a in range(3)]
            for axis in range(3):
                self.assertGreaterEqual(lo[axis],-1e-8)
                self.assertLessEqual(hi[axis],dims[axis]+1e-8)
            bounds.append((item['bin'],lo,hi))
        for i,(bin_a,lo_a,hi_a) in enumerate(bounds):
            for bin_b,lo_b,hi_b in bounds[:i]:
                if bin_a != bin_b: continue
                distance = math.sqrt(sum(max(0,lo_a[k]-hi_b[k],lo_b[k]-hi_a[k])**2 for k in range(3)))
                self.assertGreaterEqual(distance+1e-8,data['gap'])
                self.assertTrue(any(min(hi_a[k],hi_b[k])-max(lo_a[k],lo_b[k])<=1e-8 for k in range(3)))
        return data

    def test_multiple_bins_and_both_output_formats(self):
        self.run_packer('-w',2,'-H',1,'-l',1,'-t',.05,'-o','layout.obj','-m','placements.txt',*[self.model]*3)
        data = self.verify(self.root/'placements.txt.json')
        self.assertEqual(data['bin_count'],2)
        self.assertEqual(data['attempted_bin_counts'],[1,2])
        self.assertFalse(data['used_fallback'])
        self.assertEqual([p['input_index'] for p in data['placements']],[0,1,2])
        rows = [[float(x) for x in row.split()] for row in (self.root/'placements.txt').read_text().splitlines()]
        self.assertEqual(len(rows),3)
        for item,row in zip(data['placements'],rows):
            expected = item['matrix'][:]
            for axis,offset in enumerate(data['bins'][item['bin']]['display_offset']):
                expected[axis*4+3] += offset
            for a,b in zip(expected,row): self.assertAlmostEqual(a,b,places=9)
        self.assertTrue((self.root/'layout.obj').is_file())
        for b in data['bins']:
            self.assertTrue(Path(b['obj']).is_file())
            for line in Path(b['obj']).read_text().splitlines():
                if line.startswith('v '):
                    point = list(map(float,line.split()[1:]))
                    for k,limit in enumerate((2,1,1)):
                        self.assertGreaterEqual(point[k],-1e-8)
                        self.assertLessEqual(point[k],limit+1e-8)

    def test_ascending_attempt_order_is_visible(self):
        run = self.run_packer('-w',1,'-H',1,'-l',1,'-t',.15,*[self.model]*3)
        data = self.verify(self.root/'output.obj.json')
        self.assertEqual(data['bin_count'],3)
        self.assertEqual(data['attempted_bin_counts'],[1,2,3])
        self.assertFalse(data['used_fallback'])
        positions = [run.stdout.index(f'Trying: {i} |') for i in (1,2,3)]
        self.assertEqual(positions,sorted(positions))

    def test_total_timeout_marks_fallback_without_claiming_untried_counts(self):
        run = self.run_packer('-w',1,'-H',1,'-l',1,'-t',1e-12,*[self.model]*3)
        data = self.verify(self.root/'output.obj.json')
        self.assertEqual(data['bin_count'],3)
        self.assertEqual(data['attempted_bin_counts'],[1])
        self.assertTrue(data['used_fallback'])
        self.assertIn('Search budget exhausted',run.stdout)

    def test_requested_gap_changes_capacity(self):
        for gap,expected_bins in ((0,1),(.2,2)):
            self.run_packer('--width',2,'--height',1,'--length',1,'--gap',gap,'-t',.03,
                            '-m','transforms.txt',self.model,self.model)
            data = self.verify(self.root/'transforms.txt.json')
            self.assertEqual(data['bin_count'],expected_bins)
        self.assertFalse((self.root/'output.obj').exists())

    def test_positive_exact_gap_and_axis_mapping(self):
        self.run_packer('--width',2.2,'--height',1,'--length',1,'--gap',.2,'-t',.05,self.model,self.model)
        self.assertEqual(self.verify(self.root/'output.obj.json')['bin_count'],1)

    def test_invalid_arguments_and_oversized_parts_do_not_write_output(self):
        cases = [[],['-w',0,'-H',1,'-l',1],['-w',1,'-H',1,'-l',1,'--gap',-1],
                 ['-w','nan','-H',1,'-l',1],['-w','1junk','-H',1,'-l',1],
                 ['-w',1,'-H',1,'-l',1,'--seed','1.5'],['-w',.1,'-H',.1,'-l',.1]]
        for args in cases:
            self.run_packer(*args,'-t',.01,self.model,good=False)
            self.assertFalse((self.root/'output.obj').exists())
            self.assertFalse((self.root/'output.obj.json').exists())

    def test_output_cannot_overwrite_input_or_another_output(self):
        self.run_packer('-w',1,'-H',1,'-l',1,'-t',.01,'-o',self.model,self.model,good=False)
        self.run_packer('-w',1,'-H',1,'-l',1,'-t',.01,'-m','same','--manifest','same',self.model,good=False)

    def test_empty_and_degenerate_input(self):
        for contents in ('v 0 0 0\n','v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n'):
            path=self.root/'invalid.obj';path.write_text(contents)
            self.run_packer('-w',10,'-H',10,'-l',10,'-t',.01,path,good=False)
            self.assertFalse((self.root/'output.obj').exists())

    def test_constructive_search_and_budget(self):
        for seconds in (.1, 1e-12):
            self.run_packer('-w',1,'-H',1,'-l',1,'-t',seconds,'--algorithm','constructive',*[self.model]*3)
            data=self.verify(self.root/'output.obj.json')
            self.assertEqual(data['algorithm'],'constructive')
            self.assertEqual(data['bin_count'],3)
            self.assertEqual(data['attempted_bin_counts'], [1,2,3] if seconds>.01 else [1])
            self.assertEqual(data['used_fallback'], seconds<.01)
            self.assertGreaterEqual(data['search_seconds'],0)
            if seconds>.01:
                self.assertGreater(data['iterations'],0)
                self.assertGreater(data['restarts'],0)
        self.run_packer('-w',1,'-H',1,'-l',1,'--algorithm','typo',self.model,good=False)

    def test_help_and_version(self):
        self.assertIn('--height',self.run_packer('-h').stdout)
        self.assertIn('3D Packing Solver',self.run_packer('-v').stdout)


if __name__ == '__main__': unittest.main()
