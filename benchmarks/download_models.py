#!/usr/bin/env python3
"""Download unmodified Stanford research meshes, convert selected LODs to unit-length OBJ."""
import hashlib
import io
import json
from pathlib import Path
import tarfile
import urllib.request
import zipfile

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'data'
SOURCES = {
    'bunny': 'https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz',
    'dragon': 'https://graphics.stanford.edu/pub/3Dscanrep/dragon/dragon_recon.tar.gz',
    'happy': 'https://graphics.stanford.edu/pub/3Dscanrep/happy/happy_recon.tar.gz',
    'drill': 'https://graphics.stanford.edu/pub/3Dscanrep/drill.tar.gz',
    'spot': 'https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/spot.zip',
}


EXPECTED_SHA256 = {
    'bunny': 'a5720bd96d158df403d153381b8411a727a1d73cff2f33dc9b212d6f75455b84',
    'dragon': '74ac1d90989c9b1732edee82d57e9ce71452144cf4355f108d8c9c616d28d02f',
    'happy': '409cd294efbfd8244e15a382b95a9423f153b7776e736c9b09f19ec9d3c10ed0',
    'drill': '2da2acaba36903a9893c920c11506e1bd01c531bdf738d037bc505b313df5e1b',
    'spot': '1d4082da4e940f6a4cfdf648fd77e1647ac613c618dbff933f1ba23fad254608',
}

SELECTION = {
    'bunny_coarse': ('bunny', 'bunny/reconstruction/bun_zipper_res4.ply'),
    'bunny_medium': ('bunny', 'bunny/reconstruction/bun_zipper_res3.ply'),
    'bunny_full': ('bunny', 'bunny/reconstruction/bun_zipper.ply'),
    'dragon': ('dragon', 'dragon_recon/dragon_vrip_res4.ply'),
    'buddha': ('happy', 'happy_recon/happy_vrip_res4.ply'),
    'drill': ('drill', 'drill/reconstruction/drill_shaft_vrip.ply'),
    'spot': ('spot', 'spot/spot_triangulated.obj'),
}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    DATA.mkdir(exist_ok=True)
    metadata = []
    for name, url in SOURCES.items():
        archive = DATA / (name + ('.zip' if name=='spot' else '.tar.gz'))
        if not archive.exists():
            print('Downloading', url, flush=True)
            urllib.request.urlretrieve(url, archive)
        if sha(archive) != EXPECTED_SHA256[name]:
            raise RuntimeError(f'Archive checksum mismatch: {archive}')
        print(name, sha(archive), flush=True)
    for name, (source, member) in SELECTION.items():
        archive = DATA / (source + ('.zip' if source=='spot' else '.tar.gz'))
        if source=='spot':
            with zipfile.ZipFile(archive) as z:
                mesh = trimesh.load(io.BytesIO(z.read(member)), file_type='obj', process=False, maintain_order=True, skip_materials=True)
        else:
            with tarfile.open(archive) as tar:
                mesh = trimesh.load(io.BytesIO(tar.extractfile(member).read()), file_type='ply', process=False)
        mesh.remove_unreferenced_vertices()
        original_bounds = mesh.bounds.tolist()
        scale = 1.0 / max(mesh.extents)
        mesh.vertices = (mesh.vertices - mesh.bounds[0]) * scale
        # No repair, hole filling, decimation, convexification or topology changes.
        target = DATA / (name + '.obj')
        with target.open('w') as out:
            for v in mesh.vertices:
                out.write('v ' + ' '.join(format(float(x), '.17g') for x in v) + '\n')
            for face in mesh.faces:
                out.write('f ' + ' '.join(str(int(x)+1) for x in face) + '\n')
        counts = np.bincount(mesh.edges_unique_inverse)
        metadata.append(dict(name=name, source_url=SOURCES[source], archive_sha256=sha(archive),
            member=member, obj_sha256=sha(target), vertices=len(mesh.vertices), triangles=len(mesh.faces),
            boundary_edges=int(np.sum(counts==1)), nonmanifold_edges=int(np.sum(counts>2)),
            watertight=bool(mesh.is_watertight), winding_consistent=bool(mesh.is_winding_consistent),
            extents=mesh.extents.tolist(), scale=scale, original_bounds=original_bounds,
            volume=float(abs(mesh.volume)) if mesh.is_watertight else None,
            zero_area_faces=int(np.sum(mesh.area_faces<=0))))
        print(name, len(mesh.vertices), len(mesh.faces), mesh.extents.tolist(), 'closed', mesh.is_watertight, flush=True)
    (ROOT / 'models.json').write_text(json.dumps(metadata, indent=2) + '\n')



if __name__ == '__main__':
    main()
