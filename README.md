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
./build/packer [-t seconds] [-o output.obj] input1.obj [input2.obj ...]
```

## Windows PowerShell

With CMake, Visual Studio C++ tools, and Git installed:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Run:

```powershell
.\build\Release\packer.exe [-t seconds] [-o output.obj] input1.obj [input2.obj ...]
```

`-t` defaults to 60 seconds. `-o` defaults to `output.obj`. Use `-h` for help.
