# 3D Packing Solver

Packs OBJ meshes using simulated annealing.

## Linux

Install CMake, a C++17 compiler, and Git, then build:

```bash
cmake -S . -B build
cmake --build build -j
```

Run:

```bash
./build/packer [-t seconds] [-o output.obj] [-m transforms.txt] input1.obj [input2.obj ...]
```

## Windows PowerShell

With CMake, Visual Studio C++ tools, and Git installed:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Run:

```powershell
.\build\Release\packer.exe [-t seconds] [-o output.obj] [-m transforms.txt] input1.obj [input2.obj ...]
```

`-t` defaults to 60 seconds. `-o` defaults to `output.obj`. Use `-h` for help or `-v` for the version.
`-m <file>` writes each input's placement as a 3x4 matrix per line instead of the merged mesh.

## Blender extension

Install the `mesh_packer` zip from the release via *Edit > Preferences > Get Extensions > Install from Disk*, then
click **Pack All Meshes** in the 3D Viewport sidebar (`N`) under **Packing**.
