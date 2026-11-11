"""Run with blender --background --factory-startup --python tests/blender_smoke.py -- /path/to/packer."""
import sys
from pathlib import Path
import time

import addon_utils
import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'blender'))
import mesh_packer

mesh_packer.register()
entry = bpy.context.preferences.addons.new()
entry.module = 'mesh_packer'
entry.preferences.packer_path = str(Path(sys.argv[sys.argv.index('--')+1]).resolve())

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
objects=[]
for x in (5,10,15):
    bpy.ops.mesh.primitive_cube_add(size=1,location=(x,3,2))
    objects.append(bpy.context.object)
# Include a hierarchy; applying world-space transforms must not move a packed child twice.
objects[1].parent=objects[0]
bpy.context.view_layer.update()
original_meshes=[ob.data for ob in objects]
original_vertices=[[tuple(v.co) for v in ob.data.vertices] for ob in objects]
settings=bpy.context.scene.mesh_packer
settings.algorithm='constructive'
settings.width=2.2;settings.length=1;settings.height=1;settings.gap=.2;settings.time_limit=.1
result=bpy.ops.object.pack_meshes('EXEC_DEFAULT')
assert result=={'FINISHED'},result
bpy.context.view_layer.update()
assert len({ob['mesh_packer_bin'] for ob in objects})==2
for i,ob in enumerate(objects):
    assert ob.data==original_meshes[i]
    assert [tuple(v.co) for v in ob.data.vertices]==original_vertices[i]
    bounds=[ob.matrix_world @ v.co for v in ob.data.vertices]
    offset=(ob['mesh_packer_bin']-1)*(settings.width+max(settings.gap,settings.width*.1))
    for p in bounds:
        assert -1e-5<=p.x-offset<=settings.width+1e-5,tuple(p)
        assert -1e-5<=p.y<=settings.length+1e-5,tuple(p)
        assert -1e-5<=p.z<=settings.height+1e-5,tuple(p)
guides=[ob for ob in bpy.data.objects if ob.get('mesh_packer_container')]
assert len(guides)==2
assert objects[1].parent==objects[0]

# A long-running search can be cancelled without changing object matrices.
before=[ob.matrix_world.copy() for ob in objects]
settings.time_limit=10
job=mesh_packer.PackJob(bpy.context)
time.sleep(.03)
job.cancel()
assert job.process.poll() is not None
assert not Path(job.work_dir).exists()
assert all(ob.matrix_world==m for ob,m in zip(objects,before))

# Oversized input must report failure and preserve every object.
settings.width=.1;settings.length=.1;settings.height=.1;settings.time_limit=.01
try:
    outcome=bpy.ops.object.pack_meshes('EXEC_DEFAULT')
    assert outcome=={'CANCELLED'},outcome
except RuntimeError as error:
    assert 'No fitting orientation' in str(error),str(error)
assert all(ob.matrix_world==m for ob,m in zip(objects,before))
print('BLENDER_SMOKE_OK: settings, subprocess, two bins, gap, hierarchy, original meshes, cancellation, failure rollback')
