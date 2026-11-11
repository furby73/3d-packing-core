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

from .result import load_result

EXECUTABLE_NAME = "packer.exe" if sys.platform == "win32" else "packer"
TRANSFORMS_FILE = "transforms.txt"
MANIFEST_FILE = "packing.json"
PROGRESS_LINE = re.compile(r"Prog: (\d+)%")
TRYING_LINE = re.compile(r"Trying: (\d+)")


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
    algorithm: bpy.props.EnumProperty(
        name="Algorithm",
        items=[("anneal", "Simulated Annealing", "Repair conflicts using annealing"),
               ("constructive", "Constructive Backtracking", "Build valid layouts with randomized insertion and backtracking")],
        default="anneal",
    )

    time_limit: bpy.props.FloatProperty(
        name="Time Limit",
        description="Time spent searching for fewer containers, excluding mesh preparation",
        default=10.0,
        min=0.1,
        soft_max=600.0,
        unit='TIME_ABSOLUTE',
    )


    width: bpy.props.FloatProperty(
        name="Width (X)", description="Maximum container width in scene units",
        default=10.0, min=0.000001, unit='LENGTH', precision=4,
    )
    length: bpy.props.FloatProperty(
        name="Length (Y)", description="Maximum container length in scene units",
        default=10.0, min=0.000001, unit='LENGTH', precision=4,
    )
    height: bpy.props.FloatProperty(
        name="Height (Z)", description="Maximum container height in scene units",
        default=10.0, min=0.000001, unit='LENGTH', precision=4,
    )
    gap: bpy.props.FloatProperty(
        name="Part Gap", description="Minimum surface distance between parts; no extra wall clearance",
        default=0.0, min=0.0, unit='LENGTH', precision=4,
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

            settings = context.scene.mesh_packer
            self.dimensions = (settings.width, settings.length, settings.height)
            self.gap = settings.gap
            # Relative paths keep the command line short when packing many objects
            self.process = subprocess.Popen(
                [packer, "--algorithm", settings.algorithm, "-t", f"{settings.time_limit:.17g}",
                 "--width", f"{settings.width:.17g}", "--length", f"{settings.length:.17g}",
                 "--height", f"{settings.height:.17g}", "--gap", f"{settings.gap:.17g}",
                 "-m", TRANSFORMS_FILE, "--manifest", MANIFEST_FILE, *files],
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
        self.trying_bins = 0
        self.last_line = ""
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()

    def _read_output(self):
        for line in self.process.stdout:
            self.last_line = line.strip()
            match = PROGRESS_LINE.match(self.last_line)
            if match:
                self.progress = int(match.group(1)) / 100
            attempt = TRYING_LINE.search(self.last_line)
            if attempt:
                self.trying_bins = int(attempt.group(1))

    def read_placements(self):
        self.process.wait()
        self._reader.join()
        if self.process.returncode != 0:
            raise RuntimeError(f"Packing solver failed: {self.last_line}")
        placements, self.bin_offsets = load_result(
            os.path.join(self.work_dir, MANIFEST_FILE), len(self.objects), self.dimensions, self.gap,
        )
        return [(container, Matrix((row[0:4], row[4:8], row[8:12], (0.0, 0.0, 0.0, 1.0))))
                for container, row in placements]

    def cancel(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait()
        self._reader.join()
        self.cleanup()

    def cleanup(self):
        shutil.rmtree(self.work_dir, ignore_errors=True)


class OBJECT_OT_pack_meshes(bpy.types.Operator):
    """Pack visible meshes into as few containers as possible with the requested part gap"""

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
        return self.finish(context, job)

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
                return self.finish(context, self._job)
        return {'PASS_THROUGH'}

    def update_status(self):
        progress = self._job.progress
        state = "loading meshes" if progress is None else f"{progress:.0%}"
        if self._job.trying_bins:
            state += f", trying {self._job.trying_bins} containers"
        self._workspace.status_text_set(f"Packing {len(self._job.objects)} meshes: {state}  (Esc to cancel)")

    def stop_modal(self, context):
        context.window_manager.event_timer_remove(self._timer)
        self._workspace.status_text_set(None)

    def finish(self, context, job):
        try:
            placements = job.read_placements()
            targets = []
            for (name, original), (container, placement) in zip(job.objects, placements):
                ob = bpy.data.objects.get(name)
                if ob is None:
                    raise RuntimeError(f"Object {name} was removed or renamed during packing")
                if any(abs(ob.matrix_world[r][c] - original[r][c]) > 1e-6
                       for r in range(4) for c in range(4)):
                    raise RuntimeError(f"Object {name} moved during packing; run packing again")
                targets.append((ob, original, placement @ original, container, ob.get("mesh_packer_bin")))
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as ex:
            self.report({'ERROR'}, str(ex))
            return {'CANCELLED'}
        finally:
            job.cleanup()

        guides = None
        try:
            # Apply parents first to preserve the saved world-space placements.
            for ob, _, matrix, container, _ in sorted(targets, key=lambda item: hierarchy_depth(item[0])):
                ob.matrix_world = matrix
                ob["mesh_packer_bin"] = container + 1
                context.view_layer.update()
            guides = bpy.data.collections.new("Packing Containers")
            context.scene.collection.children.link(guides)
            for container, offset in sorted(job.bin_offsets.items()):
                guide = bpy.data.objects.new(f"Packing Bin {container + 1}", None)
                guides.objects.link(guide)
                guide.empty_display_type = 'CUBE'
                guide.empty_display_size = 1.0
                guide.location = tuple(offset[i] + job.dimensions[i] / 2 for i in range(3))
                guide.scale = tuple(d / 2 for d in job.dimensions)
                guide.show_in_front = True
                guide["mesh_packer_container"] = container + 1
        except Exception as ex:
            for ob, original, _, _, previous_bin in sorted(targets, key=lambda item: hierarchy_depth(item[0])):
                ob.matrix_world = original
                if previous_bin is None:
                    ob.pop("mesh_packer_bin", None)
                else:
                    ob["mesh_packer_bin"] = previous_bin
                context.view_layer.update()
            if guides is not None:
                for guide in list(guides.objects):
                    bpy.data.objects.remove(guide, do_unlink=True)
                bpy.data.collections.remove(guides)
            self.report({'ERROR'}, f"Could not apply packing: {ex}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Packed {len(targets)} meshes into {len(job.bin_offsets)} containers")
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
        settings = context.scene.mesh_packer
        layout.prop(settings, "width")
        layout.prop(settings, "length")
        layout.prop(settings, "height")
        layout.prop(settings, "gap")
        layout.prop(settings, "time_limit")
        layout.prop(settings, "algorithm")
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
