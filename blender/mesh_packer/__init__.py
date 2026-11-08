"""Runs the 3D Packing Solver from Blender on every visible mesh in the scene."""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading

import bpy
import numpy as np
from mathutils import Matrix

EXECUTABLE_NAME = "packer.exe" if sys.platform == "win32" else "packer"
TRANSFORMS_FILE = "transforms.txt"
PROGRESS_LINE = re.compile(r"Prog: (\d+)%")


class MeshPackerPreferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    packer_path: bpy.props.StringProperty(
        name="Solver Executable",
        description="Packer executable to run instead of the bundled one. Leave empty to use the bundled one",
        subtype='FILE_PATH',
    )

    def draw(self, context):
        self.layout.prop(self, "packer_path")


class MeshPackerSettings(bpy.types.PropertyGroup):
    time_limit: bpy.props.FloatProperty(
        name="Time Limit",
        description="How long the solver searches for a tighter packing",
        default=10.0,
        min=0.1,
        soft_max=600.0,
        unit='TIME_ABSOLUTE',
    )


def find_packer(context):
    prefs = context.preferences.addons[__package__].preferences
    if prefs.packer_path:
        path = bpy.path.abspath(prefs.packer_path)
    else:
        path = os.path.join(os.path.dirname(__file__), "bin", EXECUTABLE_NAME)
    if not os.path.isfile(path):
        raise RuntimeError(f"Packing solver not found at {path}. Set its path in the add-on preferences")
    if not os.access(path, os.X_OK):
        # Installing an extension from a zip drops the executable bit on Linux and macOS
        os.chmod(path, 0o755)
    return path


def select_visible_meshes(context):
    view_layer = context.view_layer
    meshes = [ob for ob in view_layer.objects if ob.type == 'MESH' and ob.visible_get() and ob.is_editable]
    selected = set(meshes)
    for ob in view_layer.objects:
        ob.select_set(ob in selected)
    if meshes and view_layer.objects.active not in selected:
        view_layer.objects.active = meshes[0]
    return meshes


def export_obj(context, ob, path):
    """Writes the evaluated mesh in world space. Returns False when it has no faces to pack."""
    ob_eval = ob.evaluated_get(context.evaluated_depsgraph_get())
    mesh = ob_eval.to_mesh()
    try:
        mesh.calc_loop_triangles()
        co = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
        mesh.vertices.foreach_get("co", co)
        tris = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
        mesh.loop_triangles.foreach_get("vertices", tris)
    finally:
        ob_eval.to_mesh_clear()

    if len(tris) == 0:
        return False

    matrix = np.array(ob.matrix_world, dtype=np.float64)
    co = co.reshape(-1, 3) @ matrix[:3, :3].T + matrix[:3, 3]
    with open(path, "w", encoding="ascii") as f:
        np.savetxt(f, co, fmt="v %.9g %.9g %.9g")
        np.savetxt(f, tris.reshape(-1, 3) + 1, fmt="f %d %d %d")
    return True


def hierarchy_depth(ob):
    depth = 0
    while ob.parent is not None:
        ob = ob.parent
        depth += 1
    return depth


class PackJob:
    """Exports the meshes, runs the solver in the background and reads back its placements"""

    def __init__(self, context):
        objects = select_visible_meshes(context)
        packer = find_packer(context)
        self.work_dir = tempfile.mkdtemp(prefix="mesh_packer_")
        try:
            self.objects = []
            files = []
            for ob in objects:
                filename = f"{len(files)}.obj"
                if export_obj(context, ob, os.path.join(self.work_dir, filename)):
                    files.append(filename)
                    self.objects.append((ob.name, ob.matrix_world.copy()))
            if not files:
                raise RuntimeError("No visible meshes to pack")

            # Relative paths keep the command line short when packing many objects
            self.process = subprocess.Popen(
                [packer, "-t", f"{context.scene.mesh_packer.time_limit:g}", "-m", TRANSFORMS_FILE, *files],
                cwd=self.work_dir,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0,
            )
        except BaseException:
            self.cleanup()
            raise

        self.progress = None
        self.last_line = ""
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()

    def _read_output(self):
        for line in self.process.stdout:
            self.last_line = line.strip()
            match = PROGRESS_LINE.match(self.last_line)
            if match:
                self.progress = int(match.group(1)) / 100

    def read_placements(self):
        self.process.wait()
        self._reader.join()
        if self.process.returncode != 0:
            raise RuntimeError(f"Packing solver failed: {self.last_line}")
        with open(os.path.join(self.work_dir, TRANSFORMS_FILE), encoding="ascii") as f:
            rows = [[float(x) for x in line.split()] for line in f]
        return [Matrix((row[0:4], row[4:8], row[8:12], (0.0, 0.0, 0.0, 1.0))) for row in rows]

    def cancel(self):
        self.process.kill()
        self.process.wait()
        self.cleanup()

    def cleanup(self):
        shutil.rmtree(self.work_dir, ignore_errors=True)


class OBJECT_OT_pack_meshes(bpy.types.Operator):
    """Select every visible mesh and pack them as tightly as possible without overlaps"""

    bl_idname = "object.pack_meshes"
    bl_label = "Pack All Meshes"
    bl_options = {'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    def execute(self, context):
        # Blocking run, used by scripts and in background mode
        try:
            job = PackJob(context)
        except (OSError, RuntimeError) as ex:
            self.report({'ERROR'}, str(ex))
            return {'CANCELLED'}
        return self.finish(job)

    def invoke(self, context, event):
        try:
            self._job = PackJob(context)
        except (OSError, RuntimeError) as ex:
            self.report({'ERROR'}, str(ex))
            return {'CANCELLED'}

        wm = context.window_manager
        self._workspace = context.workspace
        self._timer = wm.event_timer_add(0.2, window=context.window)
        wm.modal_handler_add(self)
        self.update_status()
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type == 'ESC' and event.value == 'PRESS':
            self.stop_modal(context)
            self._job.cancel()
            self.report({'WARNING'}, "Packing cancelled")
            return {'CANCELLED'}

        if event.type == 'TIMER':
            if self._job.process.poll() is None:
                self.update_status()
            else:
                self.stop_modal(context)
                return self.finish(self._job)
        return {'PASS_THROUGH'}

    def update_status(self):
        progress = self._job.progress
        state = "loading meshes" if progress is None else f"{progress:.0%}"
        self._workspace.status_text_set(f"Packing {len(self._job.objects)} meshes: {state}  (Esc to cancel)")

    def stop_modal(self, context):
        context.window_manager.event_timer_remove(self._timer)
        self._workspace.status_text_set(None)

    def finish(self, job):
        try:
            placements = job.read_placements()
        except (OSError, RuntimeError) as ex:
            self.report({'ERROR'}, str(ex))
            return {'CANCELLED'}
        finally:
            job.cleanup()

        targets = []
        for (name, matrix), placement in zip(job.objects, placements):
            ob = bpy.data.objects.get(name)
            if ob is not None:
                targets.append((ob, placement @ matrix))
        # Parents first, so children are placed relative to their parent's new transform
        for ob, matrix in sorted(targets, key=lambda target: hierarchy_depth(target[0])):
            ob.matrix_world = matrix

        self.report({'INFO'}, f"Packed {len(targets)} meshes")
        return {'FINISHED'}


class VIEW3D_PT_mesh_packer(bpy.types.Panel):
    bl_label = "Mesh Packer"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Packing"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(context.scene.mesh_packer, "time_limit")
        layout.operator(OBJECT_OT_pack_meshes.bl_idname, icon='PACKAGE')


def draw_object_menu(self, context):
    self.layout.separator()
    self.layout.operator(OBJECT_OT_pack_meshes.bl_idname, icon='PACKAGE')


classes = (
    MeshPackerPreferences,
    MeshPackerSettings,
    OBJECT_OT_pack_meshes,
    VIEW3D_PT_mesh_packer,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.mesh_packer = bpy.props.PointerProperty(type=MeshPackerSettings)
    bpy.types.VIEW3D_MT_object.append(draw_object_menu)


def unregister():
    bpy.types.VIEW3D_MT_object.remove(draw_object_menu)
    del bpy.types.Scene.mesh_packer
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
