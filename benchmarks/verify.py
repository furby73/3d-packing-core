"""Independent exported-placement checks using FCL and trimesh ray containment."""
from functools import lru_cache
import json
from pathlib import Path

import fcl
import numpy as np
import trimesh


@lru_cache(maxsize=32)
def geometry(path):
    mesh = trimesh.load(path, process=False, force='mesh')
    model = fcl.BVHModel()
    model.beginModel(len(mesh.vertices), len(mesh.faces))
    model.addSubModel(np.asarray(mesh.vertices), np.asarray(mesh.faces, dtype=np.int32))
    model.endModel()
    # One vertex of every surface component; with no crossing surfaces, this
    # suffices to detect an entire component nested inside a closed surface.
    components = trimesh.graph.connected_components(mesh.edges_unique, nodes=np.arange(len(mesh.vertices)))
    representatives = np.array([mesh.vertices[component[0]] for component in components])
    return mesh, model, representatives


def verify(path):
    data = json.loads(Path(path).read_text())
    dims = np.array([data['width'], data['length'], data['height']])
    eps = 1e-8 * max(dims)
    errors, objects = [], []
    placements = data['placements']
    if sorted(p['input_index'] for p in placements) != list(range(len(placements))):
        errors.append('input index coverage')
    if sorted(set(p['bin'] for p in placements)) != list(range(data['bin_count'])):
        errors.append('bin coverage')
    for p in placements:
        mesh, model, representatives = geometry(str(Path(p['input']).resolve()))
        matrix = np.array(p['matrix']).reshape(3,4)
        r, t = matrix[:,:3], matrix[:,3]
        if not np.isfinite(matrix).all() or not np.allclose(r@r.T, np.eye(3), atol=1e-9) or abs(np.linalg.det(r)-1)>1e-9:
            errors.append(f'nonrigid matrix {p["input_index"]}')
        vertices = np.asarray(mesh.vertices) @ r.T + t
        if vertices.min() < -eps or (vertices.max(axis=0)>dims+eps).any():
            errors.append(f'out of bounds {p["input_index"]}')
        objects.append((mesh, fcl.CollisionObject(model, fcl.Transform(r,t)), r,t,representatives))
    minimum = None
    pair_count, containment_pairs, open_pairs = 0,0,0
    for i,a in enumerate(objects):
        for j,b in enumerate(objects[:i]):
            if placements[i]['bin'] != placements[j]['bin']:
                continue
            pair_count += 1
            collided = fcl.collide(a[1], b[1], fcl.CollisionRequest(), fcl.CollisionResult())
            distance = fcl.distance(a[1], b[1], fcl.DistanceRequest(), fcl.DistanceResult())
            minimum = distance if minimum is None else min(minimum,distance)
            if collided and data['gap']>eps:
                errors.append(f'surface collision {j},{i}')
            if distance+eps < data['gap']:
                errors.append(f'clearance {j},{i}: {distance}')
            # Zero-gap surface contact is allowed by the packer; benchmark runs
            # use strictly positive gaps to avoid conflating touch and penetration.
            closed_pair = True
            for inner,outer in ((a,b),(b,a)):
                if not outer[0].is_watertight:
                    closed_pair = False
                    continue
                points = ((inner[4]@inner[2].T+inner[3])-outer[3]) @ outer[2]
                if outer[0].contains(points).any():
                    errors.append(f'containment {j},{i}')
            if closed_pair: containment_pairs += 1
            else: open_pairs += 1
    return dict(valid=not errors, errors=errors, checked_pairs=pair_count,
                closed_pairs=containment_pairs, open_pairs=open_pairs, minimum_surface_distance=minimum)


if __name__=='__main__':
    import sys
    result=verify(sys.argv[1])
    print(json.dumps(result,indent=2))
    sys.exit(not result['valid'])
