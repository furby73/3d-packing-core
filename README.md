# 3D Packing Solver

Pack OBJ meshes into as few containers as possible, with a configurable minimum
surface distance between parts. Every container has the requested maximum width,
length and height. Parts may rotate and translate; their geometry is never scaled,
inflated or simplified.

The solver tries **one container, then two, then three**, stopping at the first
feasible result. Select the search with `--algorithm` (also available in Blender):

- `anneal` (default): simulated annealing moves and rotates parts and can transfer
  them between any of the currently allowed containers. Proposals usually target
  a part involved in a collision or clearance violation; transfers prefer less
  populated containers.
- `constructive`: randomized insertion with bounded backtracking. Each prefix
  satisfies all constraints. It varies insertion order, tries wall/face coordinates
  and nearby jittered positions, and searches both axis and free rotations. On a
  dead end it undoes up to three insertions (at most 32 backtracks per restart),
  then starts a new order. It has no temperature or overlap-acceptance rule.

A quick first-fit decreasing layout supplies a valid fallback and an upper bound on
the number of containers. It uses at most 20% of the time budget, apart from a geometry
query already in progress. The remaining time is shared among the smaller
container counts that have not yet been tried; the known feasible upper bound
needs no search time. When an attempt ends without a solution, its best partial
annealing layout seeds the next count; the part involved in the most conflicts moves into the
new container. At the upper bound, the already feasible greedy layout is available.
Annealing permits temporary conflicts during search, but never exports them. If the total
budget expires early, the valid fallback is returned and explicitly marked.
An unsuccessful attempt means its allotted search time expired, not that that
container count is proven impossible. Global optimality is not guaranteed.

## Build and test

Requires CMake 3.14+, a C++17 compiler, Git and, for the CLI and extension-protocol
tests, Python 3. CMake downloads tinyobjloader on the first build.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
ctest --test-dir build -C Release --output-on-failure
```

On Linux/macOS the executable is `build/packer`. With Visual Studio it is
`build/Release/packer.exe`.

## Packing

```sh
./build/packer --width 200 --length 200 --height 150 --gap 2 \
    -t 30 -o packed.obj model-a.obj model-b.obj model-b.obj
```

- `--width` / `-w`: X dimension of each container, required and positive.
- `--length` / `-l`: Y dimension, required and positive.
- `--height` / `-H`: Z dimension, required and positive. `-h` remains help.
- `--gap` / `-g`: minimum distance between part surfaces, nonnegative; default 0.
- `-t`: improvement budget in seconds; default 60. Loading, orientation preparation,
  final validation and output are additional. A single geometry query may finish
  after the budget expires.
- `--algorithm`: `anneal` (default) or `constructive`.
- `--seed`: unsigned 32-bit random seed; default 42. Wall-clock scheduling means the
  same seed does not guarantee identical results on different runs or machines.
- `-o`: combined preview OBJ, default `output.obj` unless `-m` is given.
- `-m`: optional text file with one row-major 3x4 display transform per input.
- `--manifest`: JSON output path; default `<transforms-file>.json` when `-m` is used,
  otherwise `<output-obj>.json`.
- `-h` / `--help` and `-v` / `--version`: help and version.

All sizes use the OBJ coordinate units. For models expressed in millimetres, the
example requests a 200 x 200 x 150 mm container and 2 mm between parts. The gap is
between **surfaces**, not centres or bounding boxes. There is no additional wall
clearance: each part must lie inside `[0,width] x [0,length] x [0,height]`.

At zero gap, face contact between solid bounding boxes is permitted. Other ambiguous
surface contact is conservatively treated as a collision. Comparisons use double
precision with a relative tolerance of about 1e-9 of the part size. Solid containment
requires closed meshes; open meshes can be packed as surfaces, but their interior is
not well-defined. Degenerate triangles and empty meshes are rejected.

The initial fit search considers the 24 axis-aligned rotations, then samples free
rotations if necessary. Both search algorithms also try free rotations. Failure to find a fitting
orientation reports an error; it is not a mathematical proof that no orientation fits.
No colliding fallback is exported.

## Output containers and transforms

For one container, `packed.obj` is in container-local coordinates. For multiple
containers, the solver writes:

- `packed.obj`: a preview of all containers arranged beside one another along X.
- `packed-bin-001.obj`, `packed-bin-002.obj`, etc.: individual container contents,
  each in its own local coordinates, suitable for importing separately into a slicer.
- `packed.obj.json`: container assignments and local transforms for all inputs.

The preview's offset is for display only and is not part of the individual print jobs.
Input meshes are never overwritten. A nonzero exit code indicates invalid input,
no fitting orientation, or an I/O failure; clients must check it before using outputs.

The JSON schema is version 1. It includes `width`, `length`, `height`, `gap`,
`bin_count`, `attempted_bin_counts`, `used_fallback`, `bins` and `placements`.
Additional diagnostics include `algorithm`, `baseline_bin_count`,
`baseline_timed_out`, `baseline_seconds`, `search_seconds`, `preparation_seconds`,
`iterations` and `restarts`. Search time includes baseline construction but excludes
orientation preparation, final validation and I/O. Iterations count annealing
proposals or constructive candidate checks; these are different work units.
`attempted_bin_counts` records the ascending search order; `used_fallback` indicates
that the total budget ended before a successful attempt. Bin IDs and input indexes
are zero-based.
Each placement has `input_index`, `input`, `bin` and `matrix` (12 numbers, row-major
`[R | t]`), mapping an original input vertex to **bin-local** coordinates:

```text
local_vertex = R * original_vertex + t
preview_vertex = local_vertex + bins[bin].display_offset
```

`bins[].obj` is the absolute path of the corresponding local OBJ, or null when mesh
output is disabled. The `-m` text matrices already include the display offset and
retain one line per input in the original order. An OBJ client applies the matrix
to file coordinates. A Blender client that exported world-space coordinates
left-multiplies the original object world matrix by the display matrix.

## Clearance calculation

Clearance uses triangle-to-triangle distance on the original geometry, accelerated
by BVH distance bounds and early exit. It includes vertex-to-face and edge-to-edge
distances, surface intersections and connected-component containment checks. This
avoids an offset/remeshing stage and preserves concavities and small details. The
BVHs are built once and queried in relative local coordinates.

## Blender extension

Install the matching `mesh_packer` ZIP via **Edit > Preferences > Get Extensions >
Install from Disk**. In the 3D Viewport sidebar (`N`), open **Packing**:

1. Set **Width (X)**, **Length (Y)**, **Height (Z)** and **Part Gap**.
2. Choose the algorithm, set the improvement time budget and click **Pack All Meshes**.
3. Inspect the container outlines and packed models. Containers are displayed beside
   one another; each model's `mesh_packer_bin` property identifies its container
   (one-based). Outlines are empties in a new **Packing Containers** collection, so
   they are not included in subsequent packing runs.

The add-on exports evaluated meshes in world coordinates using Blender's scene
units, passes those same units to the solver, and applies only rigid transforms.
Original mesh data, parent relationships and modifiers remain in place. Progress is
shown in the status bar together with the container count being attempted; Esc
cancels the background process. Ctrl+Z undoes the result.
A failed run does not apply placements. Moving or renaming an object while a job is
running causes its result to be rejected. Update the executable and add-on together;
old solvers do not support the required container arguments and output manifest.

Build a local installable ZIP:

```sh
python blender/build_extension.py --binary build/packer --version 0.1.0 \
    --platform linux-x64 --output-dir dist
```

Run the integration smoke test with an installed Blender executable:

```sh
blender --background --factory-startup --python-exit-code 1 \
    --python tests/blender_smoke.py -- /absolute/path/to/build/packer
```

## Downloaded-model benchmarks

See [benchmarks/README.md](benchmarks/README.md) for source attribution, normalization,
independent validation, reproduction commands and the measured comparison.
